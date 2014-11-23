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

struct hfr_info
{
	int dummy;
	uint8_t tx_buf[MAX_CMD_LENGTH];
	uint8_t rx_buf[MAX_CMD_LENGTH];
	struct spi_ctx *spi_ctx;
	
	int gpio;

	//per chip
	uint8_t receive_fifo_length;
	uint8_t send_fifo_length;
	uint8_t last_work_id;
	int avail;
	struct work *works[256];
};

struct hfr_result
{
	uint8_t midstate0;
	uint8_t midstate1;
	uint32_t nonce;
};

int hfr_queue_size = 8;

/********** temporary helper for hexdumping SPI traffic */
static void applog_hexdump(char *prefix, uint8_t *buff, int len, int level)
{
	char *hex = bin2hex(buff, len);
	applog(level, "%s:%s", prefix, hex);
	free(hex);
}

static void hexdump(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_DEBUG);
}

static void hexdump_error(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_ERR);
}

static uint8_t *hfr_spi_transfer(struct hfr_info *info, uint8_t cmd, uint8_t *data, int req_size, int resp_size)
{
	//zero
	memset(info->tx_buf, 0, sizeof(info->tx_buf));
	memset(info->rx_buf, 0, sizeof(info->rx_buf));
	uint8_t *buf = info->tx_buf;

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
	for (i=0; i<3; i++)
	{
		hexdump("sending:", info->tx_buf, length);
		
		//write(info->gpio, "0", 1);
		
		if (!spi_transfer(info->spi_ctx, info->tx_buf, info->rx_buf, sizeof(cmd_prem) + sizeof(cmd) + req_size + /* ack */ sizeof(cmd) + resp_size))
		{
			applog(LOG_ERR, "spi transfer error");
			//write(info->gpio, "1", 1);
			return NULL;
		}
			//write(info->gpio, "1", 1);
		hexdump("recving:", info->rx_buf, length);

		uint8_t *ack = info->rx_buf + sizeof(cmd_prem) + sizeof(cmd) + req_size;
		if (cmd != *ack)
		{
			applog(LOG_ERR, "command ack error: [%02x], expected:[%02x], retrying", *ack, cmd);
		}
		else
		{
			if (i > 0)
			{
				applog(LOG_ERR, "tried %d times", i+1);
			}
			return ack + 1;
		}
	}
	applog(LOG_ERR, "command failed after %d reties", i);
	return NULL;
}


static bool write_reg(struct hfr_info *info, uint8_t address, uint32_t data)
{
	uint8_t send_data[5];

	send_data[0] = address;
	*((uint32_t *)(send_data + 1)) = htonl(data);

//	applog(LOG_ERR, "write:[%08x]", htonl(data));
	uint8_t *resp = hfr_spi_transfer(info, CMD_WRITE_REG, send_data, sizeof(send_data), 0);

	if (resp == NULL)
	{
		applog(LOG_ERR, "write reg [0x%02x] error", address);
		return false;
	}

	return true;
}

static bool read_reg(struct hfr_info *info, uint8_t address, uint32_t *data)
{
	uint8_t *resp = hfr_spi_transfer(info, CMD_READ_REG, &address, 1, 4);

	if (resp == NULL)
	{
		applog(LOG_ERR, "read reg [0x%02x] error", address);
		return false;
	}

	*data = ntohl(*((uint32_t *)resp));
//	applog(LOG_ERR, "read:[%08x]", *data);
	return true;
}

static bool reset_chip(struct hfr_info *info)
{
	if (!write_reg(info, 0x0, 0x1))
	{
		applog(LOG_ERR, "reset error");
		return false;
	}
	return true;
}

static bool set_difficulty(struct hfr_info *info, uint32_t diff)
{
	if (!write_reg(info, 0x3, diff))
	{
		applog(LOG_ERR, "set difficulty error");
		return false;
	}
	applog(LOG_ERR, "set difficulty data to 0x%08x", diff);
	return true;
}

static bool write_work(struct hfr_info *info, struct work *work)
{
	uint8_t data[44];
	memcpy(data, work->midstate, 32);
	memcpy(data + 32, work->data + 64, 12);
	hexdump("sending work:", data, sizeof(data));
	uint8_t *resp = hfr_spi_transfer(info, CMD_WRITE_WORK, data, sizeof(data), 0);

	if (resp == NULL)
	{
		applog(LOG_ERR, "write work error");
		return false;
	}

	return true;
}

static bool read_nonce(struct hfr_info *info, struct hfr_result *result)
{
	uint8_t *resp = hfr_spi_transfer(info, CMD_READ_NONCE, NULL, 0, 6);
	if (resp == NULL)
	{
		applog(LOG_ERR, "read nonce error");
		return false;
	}

	result->midstate0 = resp[0];
	result->midstate1 = resp[1];
	result->nonce = ntohl(*((uint32_t *)(resp + 2)));

	return true;
}

static bool enable_core(struct hfr_info *info, uint16_t core, bool enabled)
{
		uint8_t addr = (uint8_t)(core >> 5) + 0x5ul;
		uint32_t data;
		if (!read_reg(info, addr, &data))
		{
			applog(LOG_ERR, "enable_core read reg %02x failed", addr);
			return false;
		}
		
		if (enabled)
			data |= 1ul << core;
		else
			data &= ~(1ul << core);
	
		applog(LOG_ERR, "enable data:[%02x][%08x]", addr, data);
		
		if (!write_reg(info, addr, data))
		{
			applog(LOG_ERR, "enable core reg %02x failed", addr);
			return false;
		}
		
		return true;
}

static bool update_queue_status(struct hfr_info *info)
{
	uint32_t data;
	if (!read_reg(info, 0x4, &data))
		return false;
	info->receive_fifo_length = data & 0xff;
	info->send_fifo_length = (data >> 8) & 0x7;
	info->avail = 8 - info->receive_fifo_length;
	return true;
}

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

void hfr_detect(bool hotplug)
{
	if (hotplug)
		return;

	struct cgpu_info *cgpu = malloc(sizeof(*cgpu));
	memset(cgpu, 0, sizeof(*cgpu));

	cgpu->drv = &hfr_drv;
	cgpu->name = "HFR";
	cgpu->threads = 1;

	cgpu->device_data = calloc(sizeof(struct hfr_info), 1);

	struct hfr_info *info = cgpu->device_data;

	struct spi_config cfg = default_spi_config;
	cfg.mode = SPI_MODE_3;
	cfg.bus = 1;
	cfg.speed = 5000 * 1000;
	info->spi_ctx = spi_init(&cfg);
	if (info->spi_ctx == NULL)
	{
		applog(LOG_ERR, "spi init error");
		free(info);
		free(cgpu);
		return;
	}
	
	//info->gpio = open("/sys/class/gpio/gpio60/value", O_WRONLY);
	
	if (!reset_chip(info))
	{
		applog(LOG_ERR, "reset chip error");
		spi_exit(info->spi_ctx);
		free(info);
		free(cgpu);
		return;
	}
	
	uint32_t data;
	
	if (!set_difficulty(info, 0xfffffffful / (uint32_t)hfr_drv.max_diff))
	{
		applog(LOG_ERR, "set diff error");
		spi_exit(info->spi_ctx);
		free(info);
		free(cgpu);
		return;
	}
	
	uint8_t addr;
	for (addr = 0x5; addr <= 0x14; addr++)
	{
		if (!write_reg(info, addr, (uint32_t)0xfffffffff))
		{
			applog(LOG_ERR, "enable core reg %02x failed", addr);
			return;
		}
	}
	
	//enable_core(info, 31, false);
	//enable_core(info, 30, false);
//	write_reg(info, 0x5, 0xfffffff);
	
//	enable_core(info, 0, false);
//	enable_core(info, 1, false);

	add_cgpu(cgpu);
}




static int64_t hfr_scanwork(struct thr_info *thr)
{
	struct cgpu_info *cgpu = thr->cgpu;
	struct hfr_info *info = cgpu->device_data;
	int32_t nonces = 0;

	applog(LOG_INFO, "HFR running scanwork");

	cgsleep_ms(50);
	if (!update_queue_status(info))
	{
		applog(LOG_ERR, "update_queue_status error");
		return 0;
	}

	//applog(LOG_ERR, "rx buffer:%d, tx buffer:%d", info->receive_fifo_length, info->send_fifo_length);

	uint8_t i;
	
	if (info->send_fifo_length > 0)
		applog(LOG_ERR, "send fifo:%d", info->send_fifo_length);
	
	for (i=0; i<info->send_fifo_length; i++)
	{
		struct hfr_result result;
		if (!read_nonce(info, &result))
		{
			applog(LOG_ERR, "read_nonce error");
			return 0;
		}
		struct work *work = NULL;
		
		int j;
		for (j=0;j<256;j++)
		{
			if (info->works[j] && info->works[j]->midstate[0] == result.midstate0 && info->works[j]->midstate[1] == result.midstate1)
			{
				work = info->works[j];
			}
		}
		
		if (work == NULL)
		{
			applog(LOG_ERR, "work not found, discarded. %02x, %02x, %08x", result.midstate0, result.midstate1, result.nonce);
			continue;
		}
		if (!submit_nonce(thr, work, result.nonce))
		{
			applog(LOG_WARNING, "invalid nonce %02x, %02x, %08x", result.midstate0, result.midstate1, result.nonce);
			continue;
		}
		else
		{
			applog(LOG_ERR, "=============submitted!!!!!!      %02x, %02x, %08x", result.midstate0, result.midstate1, result.nonce);
			char *hex = bin2hex(work->hash, sizeof(work->hash));
			applog(LOG_ERR, "============= diff:%d, hash:%s", (uint32_t)share_diff(work), hex);
			free(hex);
		}
		nonces++;
	}

	return ((int64_t)nonces << 32) * (uint32_t)hfr_drv.max_diff;
}

static bool hfr_queue_full(struct cgpu_info *cgpu)
{
	struct hfr_info *info = cgpu->device_data;

	if (info->avail <= 0)
	{
		return true;
	}

	int i;
	//for (i=0; i<hfr_queue_size - info->receive_fifo_length; i++)
	{
		struct work *work = get_queued(cgpu);
		if (!work)
		{
			applog(LOG_ERR, "get queued failed");
			return false;
		}
		if (info->works[info->last_work_id])
		{
			work_completed(cgpu, info->works[info->last_work_id]);
			info->works[info->last_work_id] = NULL;
		}
		if (!write_work(info, work))
		{
			applog(LOG_ERR, "write_work failed");
			return false;
		}
		info->works[info->last_work_id] = work;
		info->last_work_id++;
		info->avail--;
	}

	return info->avail <= 0;
}

static void hfr_flush_work(struct cgpu_info *cgpu)
{
	struct hfr_info *info = cgpu->device_data;

	reset_chip(info);

	info->last_work_id = 0;
	info->receive_fifo_length = 0;
	info->send_fifo_length = 0;
	int i;
	for (i=0; i<sizeof(info->works)/sizeof(info->works[0]); i++)
	{
		if (info->works[i])
		{
			work_completed(cgpu, info->works[i]);
			info->works[i] = NULL;
		}
	}

}

static void hfr_get_statline_before(char *buf, size_t len,
                                   struct cgpu_info *cgpu)
{
        struct hfr_info *info = cgpu->device_data;
        tailsprintf(buf, len, "rx:%.2d", info->receive_fifo_length);
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
	.max_diff = 4,
};
