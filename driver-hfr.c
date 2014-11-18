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

	//per chip
	uint8_t receive_fifo_length;
	uint8_t send_fifo_length;
	uint8_t last_work_id;
	struct work *works[256];
};

struct hfr_result
{
	uint8_t work_id;
	uint32_t nonce;
};

int hfr_queue_size = 10;

static uint8_t *write_command(struct hfr_info *info, uint8_t cmd, uint8_t *data, int req_size, int resp_size)
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
	if (!spi_transfer(info->spi_ctx, info->tx_buf, info->rx_buf, sizeof(cmd_prem) + sizeof(cmd) + req_size + resp_size))
	{
		applog(LOG_ERR, "spi transfer error");
		return NULL;
	}

	uint8_t ack = info->rx_buf[sizeof(cmd_prem) + sizeof(cmd)];
	if (cmd != ack)
	{
		applog(LOG_ERR, "command ack error: [%02x], expected:[%02x]", ack, cmd);
		return NULL;
	}

	return info->rx_buf + sizeof(cmd_prem) + sizeof(cmd) + sizeof(ack);
}


static bool write_reg(struct hfr_info *info, uint8_t address, uint32_t data)
{
	uint8_t send_data[5];

	send_data[0] = address;
	*((uint32_t *)(send_data + 1)) = htonl(data);

	uint8_t *resp = write_command(info, CMD_WRITE_REG, send_data, sizeof(send_data), 0);

	if (resp == NULL)
	{
		applog(LOG_ERR, "write reg [0x%02x] error", address);
		return false;
	}

	return true;
}

static bool read_reg(struct hfr_info *info, uint8_t address, uint32_t *data)
{
	uint8_t *resp = write_command(info, CMD_READ_REG, &address, 1, 4);

	if (resp == NULL)
	{
		applog(LOG_ERR, "read reg [0x%02x] error", address);
		return false;
	}

	*data = ntohl(*((uint32_t *)resp));

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

static bool write_work(struct hfr_info *info, struct work *work, uint8_t work_id)
{
	uint8_t data[44 + 1];
	memcpy(data, work->midstate, 32);
	memcpy(data + 32, work->data, 12);
	memcpy(data + 44, &work_id, 1);
	uint8_t *resp = write_command(info, CMD_WRITE_WORK, data, sizeof(data), 0);

	if (resp == NULL)
	{
		applog(LOG_ERR, "write work error");
		return false;
	}

	return true;
}

static bool read_nonce(struct hfr_info *info, struct hfr_result *result)
{
	uint8_t *resp = write_command(info, CMD_READ_NONCE, NULL, 0, 6);
	if (resp == NULL)
	{
		applog(LOG_ERR, "read nonce error");
		return false;
	}

	memcpy(result, resp, sizeof(*resp));

	return true;
}

/********** temporary helper for hexdumping SPI traffic */
static void applog_hexdump(char *prefix, uint8_t *buff, int len, int level)
{
	static char line[256];
	char *pos = line;
	int i;
	if (len < 1)
		return;

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++) {
		if (i > 0 && (i % 32) == 0) {
			applog(LOG_DEBUG, "%s", line);
			pos = line;
			pos += sprintf(pos, "\t");
		}
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(level, "%s", line);
}

static void hexdump(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_DEBUG);
}

static void hexdump_error(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_ERR);
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
	cfg.speed = 500 * 1000;
	info->spi_ctx = spi_init(&cfg);
	if (info->spi_ctx == NULL)
	{
		applog(LOG_ERR, "spi init error");
		free(info);
		free(cgpu);
		return;
	}

	if (!reset_chip(info))
	{
		applog(LOG_ERR, "reset chip error");
		spi_exit(info->spi_ctx);
		free(info);
		free(cgpu);
		return;
	}

	add_cgpu(cgpu);
}

static bool update_queue_status(struct hfr_info *info)
{
	uint32_t data;
	if (!read_reg(info, 0x4, &data))
		return false;
	info->receive_fifo_length = data & 0xff;
	info->send_fifo_length = (data >> 8) & 0x7;
	return true;
}

static int64_t hfr_scanwork(struct thr_info *thr)
{
	struct cgpu_info *cgpu = thr->cgpu;
	struct hfr_info *info = cgpu->device_data;
	int32_t nonces = 0;

	applog(LOG_INFO, "HFR running scanwork");

	if (!update_queue_status(info))
	{
		applog(LOG_ERR, "update_queue_status error");
		return 0;
	}

	uint8_t i;
	for (i=0; i<info->send_fifo_length; i++)
	{
		struct hfr_result result;
		if (!read_nonce(info, &result))
		{
			applog(LOG_ERR, "read_nonce error");
			return 0;
		}
		struct work *work = info->works[result.work_id];
		if (work == NULL)
		{
			applog(LOG_ERR, "work id %d error, discarded", result.work_id);
			continue;
		}
		if (!submit_nonce(thr, work, result.nonce))
		{
			applog(LOG_WARNING, "invalid nonce 0x%08x", result.nonce);
			continue;
		}
		nonces++;
	}

	return (int64_t)nonces << 32;
}

static bool hfr_queue_full(struct cgpu_info *cgpu)
{
	struct hfr_info *info = cgpu->device_data;

	if (!update_queue_status(info))
	{
		applog(LOG_ERR, "update_queue_status failed");
		return false;
	}

	int i;
	for (i=0; i<hfr_queue_size - info->receive_fifo_length; i++)
	{
		struct work *work = get_queued(cgpu);
		if (info->works[info->last_work_id])
		{
			work_completed(cgpu, info->works[info->last_work_id]);
		}
		info->works[info->last_work_id] = work;
		if (!write_work(info, work, info->last_work_id))
		{
			applog(LOG_ERR, "write_work failed");
			return false;
		}
		info->last_work_id++;
	}

	return true;
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

struct device_drv hfr_drv = {
	.drv_id = DRIVER_hfr,
	.dname = "HFR",
	.name = "HFR",
	.drv_detect = hfr_detect,

	.hash_work = hash_queued_work,
	.scanwork = hfr_scanwork,
	.queue_full = hfr_queue_full,
	.flush_work = hfr_flush_work,
};
