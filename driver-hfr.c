#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>

#include "spi-context.h"
#include "logging.h"
#include "miner.h"
#include "util.h"

#include "driver-hfr.h"

static const uint8_t cmd_prem[] = { 0x55, 0xaa, 0x55, 0xaa, 0x5d };

static const int bbb_gpio_ports[] = {
		30,				//chip 0
};

#define HFR_MAX_SPI_BUSES 1
#define HFR_WORK_FIFO_LENGTH 8
#define HFR_MAX_CHIPS_PER_BUS 16
#define HFR_SPI_RETRY_COUNT 3
#define HFR_SPI1_CHIP_COUNT 1
#define HFR_SPI_BAUD_RATE (5000 * 1000)
#define HFR_CORES_PER_CHIP 512
#define HFR_DEFAULT_DEVICE_DIFF 2
#define HFR_BAD_CORE_THRESHOLD 4

struct hfr_chip
{
	bool enabled;
	uint8_t receive_fifo_length;
	uint8_t send_fifo_length;
	uint8_t last_work_id;
	int receive_fifo_avail;
	struct work *works[HFR_WORK_FIFO_LENGTH * 2];
	int core_errors[HFR_CORES_PER_CHIP];
};

struct hfr_spi_bus
{
	uint32_t device_diff;
	struct spi_ctx *spi_ctx;
	uint8_t tx_buf[MAX_CMD_LENGTH];
	uint8_t rx_buf[MAX_CMD_LENGTH];

	int selected_chip;
	bool (*init)(struct hfr_spi_bus *);
	bool (*select)(struct hfr_spi_bus *, int);
	void *select_data;

	int chip_count;
	struct hfr_chip chips[HFR_MAX_CHIPS_PER_BUS];
};

struct hfr_result
{
	uint8_t midstate0;
	uint8_t midstate1;
	uint32_t nonce;
};

static void dumpbin(int level, char *prefix, uint8_t *buff, int len)
{
	char *hex = bin2hex(buff, len);
	applog(level, "%s:%s", prefix, hex);
	free(hex);
}

static bool gpio_init(struct hfr_spi_bus *bus, const int gpio_ports[])
{
	//linux gpio fs driver
	/*
	int export = open("/sys/class/gpio/export", O_WRONLY);
	if (export == -1)
	{
		applog(LOG_ERR, "GPIO export file open failed:%s", strerror(errno));
		return false;
	}
	*/

	bus->select_data = calloc(bus->chip_count, sizeof(int));
	int *fds = bus->select_data;

	int chip_id;
	for (chip_id=0; chip_id<bus->chip_count; chip_id++)
	{
		int port = gpio_ports[chip_id];
		char buf[100];

		/*
		sprintf(buf, "%d\n", gpio_ports[chip_id]);
		int size = strlen(buf) + 1;
		if (write(export, buf, size) != size)
		{
			applog(LOG_ERR, "GPIO port %d export failed:%s", port, strerror(errno));
			goto cleanup;
		}
		*/

		//set direction and active high
		sprintf(buf, "/sys/class/gpio/gpio%d/direction", port);
		int fd;
		fd = open(buf, O_WRONLY);
		if (fd == -1)
		{
			applog(LOG_ERR, "GPIO export file open failed:%s", strerror(errno));
			goto cleanup;
		}
		if (write(fd, "high\n", 5) != 5)
		{
			applog(LOG_ERR, "GPIO write direction file %d failed:%s", port, strerror(errno));
			close(fd);
			goto cleanup;
		}

		//get value fd
		sprintf(buf, "/sys/class/gpio/gpio%d/value", port);
		fd = open(buf, O_WRONLY);
		if (fd == -1)
		{
			applog(LOG_ERR, "GPIO value file %d open failed:%s", port, strerror(errno));
			goto cleanup;
		}
		fds[chip_id] = fd;
	}

	//close(export);
	return true;

	cleanup:;
	for (chip_id=0; chip_id<bus->chip_count; chip_id++)
	{
		if (fds[chip_id])
			close(fds[chip_id]);
	}
	free(fds);
	bus->select_data = NULL;
	//close(export);
	return false;
}

static bool gpio_select(struct hfr_spi_bus *bus, int chip_id)
{
	int *fds = bus->select_data;
	if ((bus->selected_chip != -1) && (write(fds[chip_id], "1\n", 2) != 2))
	{
		applog(LOG_ERR, "deselect chip %d.%d failed:%s", bus->spi_ctx->config.bus, chip_id, strerror(errno));
		return false;
	}
	if (write(fds[chip_id], "0\n", 2) != 2)
	{
		applog(LOG_ERR, "select chip %d.%d failed:%s", bus->spi_ctx->config.bus, chip_id, strerror(errno));
		return false;
	}
	return true;
}


static uint8_t *hfr_spi_transfer(struct hfr_spi_bus *bus, int chip_id, uint8_t cmd, uint8_t *data, int req_size, int resp_size)
{
	if (bus->selected_chip != chip_id)
	{
		if (!bus->select(bus, chip_id))
		{
			applog(LOG_ERR, "Select chip %d.%d failed", bus->spi_ctx->config.bus, chip_id);
			return NULL;
		}
		bus->selected_chip = chip_id;
	}

	//zero
	memset(bus->tx_buf, 0, sizeof(bus->tx_buf));
	memset(bus->rx_buf, 0, sizeof(bus->rx_buf));
	uint8_t *buf = bus->tx_buf;

	//header
	memcpy(buf, cmd_prem, sizeof(cmd_prem));
	buf += sizeof(cmd_prem);

	//command
	*(buf++) = cmd;

	//command && data
	if (req_size > 0)
	{
		memcpy(buf, data, req_size);
	}

	//transfer
	int length = sizeof(cmd_prem) + sizeof(cmd) * 2 + req_size + resp_size;
	int i;
	for (i=0; i<HFR_SPI_RETRY_COUNT; i++)
	{
		dumpbin(LOG_DEBUG, "SPI Send:", bus->tx_buf, length);
		
		if (!spi_transfer(bus->spi_ctx, bus->tx_buf, bus->rx_buf, sizeof(cmd_prem) + sizeof(cmd) + req_size + /* ack */ sizeof(cmd) + resp_size))
		{
			applog(LOG_ERR, "SPI%d transfer error", bus->spi_ctx->config.bus);
			return NULL;
		}
		dumpbin(LOG_DEBUG,"SPI Recv:", bus->rx_buf, length);

		uint8_t *ack = bus->rx_buf + sizeof(cmd_prem) + sizeof(cmd) + req_size;
		if (cmd == *ack)
		{
			if (i > 0)
				applog(LOG_WARNING, "SPI%d transfer succeeded after %d retries", bus->spi_ctx->config.bus, i);
			return ack + 1;
		}
		applog(LOG_WARNING, "SPI%d ACK error:received [%02x], expected:[%02x], retrying", bus->spi_ctx->config.bus, *ack, cmd);
	}
	applog(LOG_ERR, "SPI%d failed after %d retries", bus->spi_ctx->config.bus, i);
	return NULL;
}


static bool write_reg(struct hfr_spi_bus *bus, int chip_id, uint8_t address, uint32_t data)
{
	uint8_t send_data[5];

	send_data[0] = address;
	*((uint32_t *)(send_data + 1)) = htonl(data);

	applog(LOG_DEBUG, "Writing register [%02x] on chip %d.%d [%08x]", address, bus->spi_ctx->config.bus, chip_id, data);
	uint8_t *resp = hfr_spi_transfer(bus, chip_id, CMD_WRITE_REG, send_data, sizeof(send_data), 0);

	if (resp == NULL)
	{
		applog(LOG_ERR, "Writing register [%02x] on chip %d.%d [%08x] failed", address, bus->spi_ctx->config.bus, chip_id, data);
		return false;
	}

	return true;
}

static bool read_reg(struct hfr_spi_bus *bus, int chip_id, uint8_t address, uint32_t *data)
{
	uint8_t *resp = hfr_spi_transfer(bus, chip_id, CMD_READ_REG, &address, 1, 4);

	if (resp == NULL)
	{
		applog(LOG_ERR, "Read register [%02x] on chip %d.%d failed", address, bus->spi_ctx->config.bus, chip_id);
		return false;
	}

	*data = ntohl(*((uint32_t *)resp));
	applog(LOG_DEBUG, "Read register [%02x] on chip %d.%d [%08x]", address, bus->spi_ctx->config.bus, chip_id, *data);

	return true;
}

static bool reset_chip(struct hfr_spi_bus *bus, int chip_id)
{
	if (!write_reg(bus, chip_id, 0x0, 0x1))
	{
		applog(LOG_ERR, "reset chip %d.%d failed", bus->spi_ctx->config.bus, chip_id);
		return false;
	}
	return true;
}

static bool set_difficulty(struct hfr_spi_bus *bus, int chip_id, uint32_t diff)
{
	if (!write_reg(bus, chip_id, 0x3, 0xfffffffful / diff))
	{
		applog(LOG_ERR, "set chip %d.%d difficulty  failed", bus->spi_ctx->config.bus, chip_id);
		return false;
	}
	applog(LOG_DEBUG, "set chip %d.%d difficulty data to [%08x]", bus->spi_ctx->config.bus, chip_id, diff);
	return true;
}

static bool write_work(struct hfr_spi_bus *bus, int chip_id, struct work *work)
{
	uint8_t data[44];
	memcpy(data, work->midstate, 32);
	memcpy(data + 32, work->data + 64, 12);
	dumpbin(LOG_DEBUG, "sending work:", data, sizeof(data));
	uint8_t *resp = hfr_spi_transfer(bus, chip_id, CMD_WRITE_WORK, data, sizeof(data), 0);

	if (resp == NULL)
	{
		applog(LOG_ERR, "send work to chip %d.%d failed", bus->spi_ctx->config.bus, chip_id);
		return false;
	}

	return true;
}

static bool read_nonce(struct hfr_spi_bus *bus, int chip_id, struct hfr_result *result)
{
	uint8_t *resp = hfr_spi_transfer(bus, chip_id, CMD_READ_NONCE, NULL, 0, 6);
	if (resp == NULL)
	{
		applog(LOG_ERR, "read nonce from chip %d.%d failed", bus->spi_ctx->config.bus, chip_id);
		return false;
	}

	result->midstate0 = resp[0];
	result->midstate1 = resp[1];
	result->nonce = ntohl(*((uint32_t *)(resp + 2)));

	applog(LOG_DEBUG, "read nonce [%08x] from chip %d.%d failed", result->nonce, bus->spi_ctx->config.bus, chip_id);

	return true;
}

static bool enable_core(struct hfr_spi_bus *bus, int chip_id, uint32_t core, bool enabled)
{
	applog(LOG_WARNING, "%s chip %d.%d core %d", enabled ? "enabling" : "disabling", bus->spi_ctx->config.bus, chip_id, core);

	uint8_t addr = (uint8_t)(core >> 5) + 0x5ul;
	uint32_t data;
	if (!read_reg(bus, chip_id, addr, &data))
	{
		applog(LOG_ERR, "enabling chip %d.%d core %d read register failed", bus->spi_ctx->config.bus, chip_id, core);
		return false;
	}

	if (enabled)
		data |= 1ul << core;
	else
		data &= ~(1ul << core);

	if (!write_reg(bus, chip_id, addr, data))
	{
		applog(LOG_ERR, "enabling chip %d.%d core %d write register failed", bus->spi_ctx->config.bus, chip_id, core);
		return false;
	}
	
	return true;
}

static void update_chip_status(struct hfr_spi_bus *bus, int chip_id)
{
	uint32_t data;
	struct hfr_chip *chip = &bus->chips[chip_id];

	if (!read_reg(bus, chip_id, 0x4, &data))
	{
		applog(LOG_WARNING, "update status for chip %d.%d failed", bus->spi_ctx->config.bus, chip_id);
	}
	chip->receive_fifo_length = data & 0xff;
	chip->send_fifo_length = (data >> 8) & 0x7;
	chip->receive_fifo_avail = HFR_WORK_FIFO_LENGTH - chip->receive_fifo_length;
}

static void dumpwork(struct work *work)
{
	if (opt_debug)
	{
		uint8_t data[44];
		memcpy(data, work->midstate, 32);
		memcpy(data + 32, work->data + 64, 12);

		char *send = bin2hex(data, sizeof(data));
		char *hash = bin2hex(work->hash, sizeof(work->hash));
		applog(LOG_WARNING, "WorkDump - data[%s] nonce[%08x] hash[%s] diff[%d]", send, work->nonce, hash, (int)share_diff(work));
		free(hash);
		free(send);
	}
}

void hfr_detect(bool hotplug)
{
	if (hotplug)
		return;

	struct cgpu_info *cgpu = malloc(sizeof(*cgpu));
	memset(cgpu, 0, sizeof(*cgpu));

	cgpu->drv = &hfr_drv;
	cgpu->name = "HFR";
	cgpu->threads = 1;

	cgpu->device_data = calloc(sizeof(struct hfr_spi_bus), 1);

	struct hfr_spi_bus *bus = cgpu->device_data;
	int i;
	//SPI1 for BBB
	struct spi_config cfg = default_spi_config;
	cfg.mode = SPI_MODE_3;
	cfg.bus = 1;
	cfg.speed = HFR_SPI_BAUD_RATE;

	if ((bus->spi_ctx = spi_init(&cfg)) == NULL)
	{
		applog(LOG_ERR, "SPI1 init failed");
		goto cleanup;
	}

	bus->device_diff = HFR_DEFAULT_DEVICE_DIFF;
	bus->chip_count = HFR_SPI1_CHIP_COUNT;

	if (!gpio_init(bus, bbb_gpio_ports))
	{
		applog(LOG_ERR, "GPIO for SPI1 init failed");
		goto cleanup;
	}

	bus->select = gpio_select;
	bus->selected_chip = -1;

	for (i=0; i<bus->chip_count; i++)
	{
		bus->chips[i].enabled = true;

		if (!reset_chip(bus, i))
		{
			applog(LOG_WARNING, "SPI1 chip %d init failed, ignored", i);
			bus->chips[i].enabled = false;
			continue;
		}

		if (!set_difficulty(bus, i, bus->device_diff))
		{
			applog(LOG_WARNING, "SPI1 chip %d set difficulty failed, ignored", i);
			bus->chips[i].enabled = false;
			continue;
		}

		uint8_t addr;
		for (addr = 0x5; addr <= 0x14; addr++)
		{
			if (!write_reg(bus, i, addr, 0xffffffffu))
			{
				applog(LOG_ERR, "SPI1 chip %d enable core reg %d failed", i, addr);
				return;
			}
		}
	}

	add_cgpu(cgpu);
	return;
	
	cleanup:;
	free(bus);
	free(cgpu);
	return;
}


static int64_t hfr_scanwork(struct thr_info *thr)
{
	struct cgpu_info *cgpu = thr->cgpu;
	struct hfr_spi_bus *bus = cgpu->device_data;
	int32_t nonces = 0;

	applog(LOG_INFO, "HFR running scanwork");

	cgsleep_ms(300);


	uint8_t i;
	int chip_id;
	for (chip_id=0; chip_id < bus->chip_count; chip_id++)
	{
		struct hfr_chip *chip = &bus->chips[chip_id];
		
		if (!chip->enabled)
			continue;

		update_chip_status(bus, chip_id);


		for (i=0; i<chip->send_fifo_length; i++)
		{
			struct hfr_result result;
			if (!read_nonce(bus, chip_id, &result))
			{
				applog(LOG_ERR, "read nonce from chip %d.%d failed", bus->spi_ctx->config.bus, chip_id);
				continue;
			}

			struct work *work = NULL;
			int j;
			for (j=0; j< sizeof(chip->works)/sizeof(chip->works[0]); j++)
			{
				if (chip->works[j] && chip->works[j]->midstate[0] == result.midstate0 && chip->works[j]->midstate[1] == result.midstate1)
				{
					work = chip->works[j];
				}
			}

			if (work == NULL)
			{
				applog(LOG_ERR, "work from chip %d.%d not found, discarded. %02x-%02x-%08x", bus->spi_ctx->config.bus, chip_id, result.midstate0, result.midstate1, result.nonce);
				continue;
			}

			if (!submit_nonce(thr, work, result.nonce))
			{
				uint32_t core = result.nonce >> 23;
				chip->core_errors[core]++;
				applog(LOG_ERR, "Invalid nonce from chip %d.%d core %d, discarded [%d]. %02x-%02x-%08x", bus->spi_ctx->config.bus, chip_id, core, chip->core_errors[core], result.midstate0, result.midstate1, result.nonce);
				if (chip->core_errors[core] > HFR_BAD_CORE_THRESHOLD)
				{
					enable_core(bus, chip_id, core, false);
				}
				continue;
			}
			else
			{
				applog(LOG_DEBUG, "Submitting result from chip %d.%d [%02x-%02x-%08x] of diff %d", bus->spi_ctx->config.bus, chip_id, result.midstate0, result.midstate1, result.nonce, (int)share_diff(work));
				dumpwork(work);
				nonces++;
			}
		}
	}
	return ((int64_t)nonces << 32) * bus->device_diff;
}

static bool hfr_queue_full(struct cgpu_info *cgpu)
{
	struct hfr_spi_bus *bus = cgpu->device_data;

	int chip_id;

	for (chip_id=0; chip_id<bus->chip_count; chip_id++)
	{
		struct hfr_chip *chip = &bus->chips[chip_id];

		if (!chip->enabled)
			continue;

		if (chip->receive_fifo_avail > 0)
		{
			struct work *work = get_queued(cgpu);
			if (!work)
				return false;

			if (chip->works[chip->last_work_id])
			{
				work_completed(cgpu, chip->works[chip->last_work_id]);
				chip->works[chip->last_work_id] = NULL;
			}
			if (!write_work(bus, chip_id, work))
			{
				applog(LOG_ERR, "write work to chip %d.%d failed", bus->spi_ctx->config.bus, chip_id);
			}
			else
			{
				chip->works[chip->last_work_id] = work;
				chip->last_work_id = (chip->last_work_id + 1) % (sizeof(chip->works)/sizeof(chip->works[0]));
			}
			//always decrease avail to prevent stuck with one sick chip
			chip->receive_fifo_avail--;
			return false;
		}
	}
	return true;
}

static void hfr_flush_work(struct cgpu_info *cgpu)
{
	struct hfr_spi_bus *bus = cgpu->device_data;

	int chip_id;
	for (chip_id=0; chip_id<bus->chip_count; chip_id++)
	{
		struct hfr_chip *chip = &bus->chips[chip_id];
		if (!chip->enabled)
			continue;

		if (!reset_chip(bus, chip_id))
		{
			applog(LOG_ERR, "Reset chip %d.%d failed", bus->spi_ctx->config.bus, chip_id);
		}

		chip->last_work_id = 0;
		chip->receive_fifo_length = 0;
		chip->send_fifo_length = 0;
		chip->receive_fifo_avail = 0;

		int i;
		for (i=0; i<sizeof(chip->works)/sizeof(chip->works[0]); i++)
		{
			if (chip->works[i])
			{
				work_completed(cgpu, chip->works[i]);
				chip->works[i] = NULL;
			}
		}
	}
}

static void hfr_get_statline_before(char *buf, size_t len, struct cgpu_info *cgpu)
{
//        struct hfr_spi_bus *bus = cgpu->device_data;
//        tailsprintf(buf, len, " ");
}


struct device_drv hfr_drv = {
	.drv_id = DRIVER_hfr,
	.dname = "HFR",
	.name = "HFR",
	.drv_detect = hfr_detect,

	.hash_work = hash_queued_work,
	.scanwork = hfr_scanwork,
	.queue_full = hfr_queue_full,
	.flush_work = hfr_flush_work,
	.get_statline_before = hfr_get_statline_before,
	.max_diff = 2,
};

/*
static void test(struct hfr_info *info)
{
	uint8_t addr;
	for (addr = 0x5; addr <= 0x14; addr++)
	{
		if (!write_reg(info, addr, (uint32_t)0xfffffffff))
		{
			applog(LOG_ERR, "enable core reg %02x failed", addr);
			return;
		}
	}

	struct work work = {0};

	for (addr = 0; addr < 32; addr++)
		work.midstate[addr] = addr;
	for (addr = 64; addr < 64 + 12; addr++)
		work.data[addr] = addr;

	if (!write_work(info, &work))
	{
		applog(LOG_ERR, "write work error");
		return;
	}

	update_queue_status(info);

	applog(LOG_ERR, "rx buffer:%d, tx buffer:%d", info->receive_fifo_length, info->send_fifo_length);

	struct hfr_result result;
	if (!read_nonce(info, &result))
	{
		applog(LOG_ERR, "read result failed");
		return;
	}

	applog(LOG_ERR, "m0:%02x, m1:%02x, nonce:%08x", result.midstate0, result.midstate1, result.nonce);

	applog(LOG_ERR, "===========test for every core");
	uint16_t core;
	for (core = 0; core < 512; core++)
	{
		applog(LOG_ERR, "==================== core %d ================", core);

		work.midstate[0] = core & 0xff;
		work.midstate[1] = core >> 8;
		work.data[64 + 8] = work.midstate[0] ^ work.midstate[1];

		if (!write_work(info, &work))
		{
			applog(LOG_ERR, "write work error");
			return;
		}
		do
		{
			cgsleep_ms(50);
			update_queue_status(info);
	  	applog(LOG_ERR, "rx buffer:%d, tx buffer:%d", info->receive_fifo_length, info->send_fifo_length);
		}while (info->send_fifo_length == 0);

		if (!read_nonce(info, &result))
		{
			applog(LOG_ERR, "===============read result failed");
			return;
		}

		if ((result.midstate0 != (uint8_t)~work.midstate[0]) || (result.midstate1 != (uint8_t)~work.midstate[1]))
		{
			applog(LOG_ERR, "===============N failed, %02x, %02x, %02x, %02x,", result.midstate0, result.midstate1, ~work.midstate[0], ~work.midstate[1]);
			return;
		}

		if (result.nonce != ntohl(*((uint32_t *)(work.data + 64 + 8))))
		{
			applog(LOG_ERR, "===============nonce failed");

			do
			{
				cgsleep_ms(50);
				update_queue_status(info);
		  	applog(LOG_ERR, "rx buffer:%d, tx buffer:%d", info->receive_fifo_length, info->send_fifo_length);
			}while (info->send_fifo_length == 0);
			if (!read_nonce(info, &result))
			{
				applog(LOG_ERR, "===============read result failed");
				return;
			}
			applog(LOG_ERR, "======== reread nonce: %02x, %02x, %08x", result.midstate0, result.midstate1, result.nonce);

			return;
		}
	}


	return;
}

static void test2(struct hfr_info *info)
{
	uint8_t addr;
	for (addr = 0x5; addr <= 0x14; addr++)
	{
		if (!write_reg(info, addr, (uint32_t)0xfffffffff))
		{
			applog(LOG_ERR, "enable core reg %02x failed", addr);
			return;
		}
	}

	static struct work work[512] = {0};

	uint16_t core;
	for (core = 0; core < 512; core++)
	{
		for (addr = 0; addr < 32; addr++)
			work[core].midstate[addr] = addr;
		for (addr = 64; addr < 64 + 12; addr++)
			work[core].data[addr] = addr;
		work[core].midstate[0] = core & 0xff;
		work[core].midstate[1] = core >> 8;
		work[core].data[64 + 8] = work[core].midstate[0] ^ work[core].midstate[1];
	}

	int send = 0;
	int recv = 0;
	int i;
	while(recv < 512)
	{
				cgsleep_ms(50);
		update_queue_status(info);
		applog(LOG_ERR, "rx buffer:%d, tx buffer:%d", info->receive_fifo_length, info->send_fifo_length);

		for (i=0;(i<8-info->receive_fifo_length) && (send < 512); i++)
		{
			if (!write_work(info, &work[send++]))
			{
				applog(LOG_ERR, "write work error:%d", i);
				return;
			}
		}

		struct hfr_result result;
		for (i=0; (i<info->send_fifo_length) && (recv < 512); i++)
		{
			if (!read_nonce(info, &result))
			{
				applog(LOG_ERR, "===============read result failed");
				return;
			}

			struct work *w = &work[recv++];

			if ((result.midstate0 != (uint8_t)~w->midstate[0]) || (result.midstate1 != (uint8_t)~w->midstate[1]))
			{
				applog(LOG_ERR, "===============N failed, %02x, %02x, %02x, %02x,", result.midstate0, result.midstate1, ~w->midstate[0], ~w->midstate[1]);
				return;
			}

			if (result.nonce != ntohl(*((uint32_t *)(w->data + 64 + 8))))
			{
				applog(LOG_ERR, "===============nonce failed");
				return;
			}
		}
	}
}
*/
