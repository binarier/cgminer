/*
 * Copyright 2013 binarier <binarier@clambtc.com>
 * Copyright 2013 Con Kolivas <kernel@kolivas.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "logging.h"

#include "miner.h"
#include "usbutils.h"
#include "uthash.h"

#include "driver-clam.h"

struct device_drv clam_drv;

int opt_clam_clock = CLAM_DEFAULT_CLOCK;
int opt_clam_core_limit = CLAM_MAX_CORE_COUNT;
int opt_clam_chip_start = 0;
int opt_clam_chip_end = 32;
bool opt_clam_test_only = false;
uint8_t golden_midstate[] = { 0x46,0x79,0xba,0x4e,0xc9,0x98,0x76,0xbf,0x4b,0xfe,0x08,0x60,0x82,0xb4,0x00,0x25,0x4d,0xf6,0xc3,0x56,0x45,0x14,0x71,0x13,0x9a,0x3a,0xfa,0x71,0xe4,0x8f,0x54,0x4a};
uint8_t golden_data[] = {0x87, 0x32, 0x0b, 0x1a, 0x14, 0x26, 0x67, 0x4f, 0x2f, 0xa7, 0x22, 0xce};

static void flush_buffer(struct cgpu_info *cgpu)
{
	char buf[512];
	int amount;

	do {
		usb_read_once(cgpu, buf, 512, &amount, C_CLAM_FLUSH_BUFFER);
	} while (amount);
}

static bool clam_read_result(struct cgpu_info *cgpu, struct clam_result *result, int ms_timeout)
{
	int read;
	
	int err = usb_read_timeout(cgpu, (char *)result, sizeof(*result), &read, ms_timeout, C_CLAM_READ_DATA);
	if (err != LIBUSB_SUCCESS || read != sizeof(*result))
	{
		applog(LOG_ERR, "[Clam Debug] read error:%d, read:%d", err, read);
		return false;
	}
	//change endian
	result->result = be32toh(result->result);
	return true;
}

static bool clam_write(struct cgpu_info *cgpu, void *data, int size, enum usb_cmds cmd)
{
	int written;
	int err;
	
	err = usb_write(cgpu, data, size, &written, cmd);
	if (err != LIBUSB_SUCCESS || written != size)
	{
		applog(LOG_ERR, "[Clam] usb write error:%d, written %d", err, written);
		return false;
	}

	return true;
}
static bool flush_result(struct cgpu_info *cgpu)
{
	applog(LOG_DEBUG, "[Clam] flush result");
	int ret = usb_transfer(cgpu, CLAM_TYPE_OUT, CLAM_REQUEST_FLUSH_RESULT, 0, 0, C_CLAM_FLUSH_RESULT);
	if (ret != LIBUSB_SUCCESS)
	{
		applog(LOG_ERR, "[Clam] flush result failed, %d", ret);
		return false;
	}
	return true;
}

static bool write_work(struct cgpu_info *cgpu, unsigned char *midstate, unsigned char *data)
{
	if (opt_debug)
	{
		char *hex_midstate;
		char *hex_data;
		hex_midstate = bin2hex(midstate, 32);
		hex_data = bin2hex(data, 12);
		applog(LOG_DEBUG, "[Clam] send work : midstate[%s], data[%s]", hex_midstate, hex_data);
		free(hex_midstate);
		free(hex_data);
	}

	//reverse bye order according to chip spec
	unsigned char buf[32 + 12];
	unsigned char *p = buf;
	int i;
	for (i=0;i<32;i++)
		*(p++) = midstate[31-i];
	for (i=0;i<12;i++)
		*(p++) = data[11-i];

	if (unlikely(!clam_write(cgpu, buf, sizeof(buf), C_CLAM_WRITE_WORK)))
	{
		applog(LOG_ERR, "[Clam] work write error");
		return false;
	}
	return true;
}


static bool reset_channel(struct cgpu_info *cgpu, uint8_t channel_id)
{
	struct clam_info *info = cgpu->device_data;
	int ret = usb_transfer(cgpu, CLAM_TYPE_OUT, CLAM_REQUEST_RESET_CHANNEL, 0, channel_id, C_CLAM_RESET_CHANNEL);
	if (ret != LIBUSB_SUCCESS)
	{
		applog(LOG_ERR, "[Clam] reset controller failed, %d", ret);
		return false;
	}
	ret = usb_transfer(cgpu, CLAM_TYPE_IN, CLAM_REQUEST_CLOCK, opt_clam_clock, channel_id, C_CLAM_CLOCK);
	if (ret != LIBUSB_SUCCESS)
	{
		applog(LOG_ERR, "[Clam] channel %d : set clock error - [%d], ignored.", channel_id, ret);
		return false;
	}
	info->channels[channel_id].cont_hw = 0;	
	info->channels[channel_id].cont_timeout = 0;	
	return true;
}

static bool reset_controller(struct cgpu_info *cgpu)
{
	int ret = usb_transfer(cgpu, CLAM_TYPE_OUT, CLAM_REQUEST_RESET_CONTROLLER, 0, 0, C_CLAM_RESET_CONTROLLER);
	if (ret != LIBUSB_SUCCESS)
	{
		applog(LOG_ERR, "[Clam] reset controller failed, %d", ret);
		return false;
	}
	return true;
}
static bool clam_init(struct cgpu_info *cgpu)
{
	struct clam_info *info = cgpu->device_data;
	char buf[64 + 1];
	int read;

	//identify version
	int ret = usb_transfer_read(cgpu, CLAM_TYPE_IN, CLAM_REQUEST_VERSION, 0, 0, buf, sizeof(buf), &read, C_CLAM_VERSION);
	if (ret != LIBUSB_SUCCESS)
		return false;
	buf[read] = 0;
	if (strncmp(buf, "CLAM", 4) != 0)
	{
		applog(LOG_ERR, "[Clam] device version not match - [%s], ignored.", buf);
		return false;
	}
	applog(LOG_NOTICE, "[Clam] found device [%s]", buf);

	//identify channels
	ret = usb_transfer_read(cgpu, CLAM_TYPE_IN, CLAM_REQUEST_CHANNELS, 0, 0, buf, sizeof(buf), &read, C_CLAM_CHANNELS);
	if (ret != LIBUSB_SUCCESS || read != 4)
	{
		applog(LOG_ERR, "[Clam] read device channels error - [%s], ignored.", buf);
		return false;
	}
	info->channel_count = *(uint32_t *)buf;

	//identify chip and cores
	int channel_id;
	for (channel_id = 0; channel_id<info->channel_count; channel_id++)
	{
		ret = usb_transfer_read(cgpu, CLAM_TYPE_IN, CLAM_REQUEST_CORES, 0, channel_id, buf, sizeof(buf), &read, C_CLAM_CHANNELS);
		if (ret != LIBUSB_SUCCESS)
		{
			applog(LOG_ERR, "[Clam] channel %d : read chip information error - [%d], ignored.", channel_id, ret);
			return false;
		}

		info->channels[channel_id].chip_count = read;
		memcpy(info->channels[channel_id].core_map, buf, read);

		int j, k;
		info->channels[channel_id].core_count = 0;
		for (j = 0; j<read; j++)
		{
			uint8_t mask = 1;
			for (k=0; k<8; k++)
			{
				if (info->channels[channel_id].core_map[j] & mask)
					info->channels[channel_id].core_count++;
				mask <<= 1;
			}
		}
		applog(LOG_NOTICE, "[Clam] channel %d, %d chips, %d cores, core map : %s", channel_id, read, info->channels[channel_id].core_count, bin2hex(buf, read));

		ret = usb_transfer(cgpu, CLAM_TYPE_OUT, CLAM_REQUEST_CLOCK, opt_clam_clock, channel_id, C_CLAM_CLOCK);
		if (ret != LIBUSB_SUCCESS)
		{
			applog(LOG_ERR, "[Clam] channel %d : set clock error - [%d], ignored.", channel_id, ret);
			return false;
		}
	}

	if (opt_clam_test_only)
		return false;

	flush_result(cgpu);
	return true;
}

static void clam_reinit(struct cgpu_info  *cgpu)
{
	reset_controller(cgpu);
	clam_init(cgpu);
}

static bool clam_detect_one(struct libusb_device *dev, struct usb_find_devices *found)
{
	struct cgpu_info *cgpu;
	struct clam_info *info = calloc(1, sizeof(*info));

	cgpu = usb_alloc_cgpu(&clam_drv, 1);

	cgpu->drv = &clam_drv;
	cgpu->device_data = info;
	cgpu->threads = 1;

	if (!usb_init(cgpu, dev, found))
	{
		applog(LOG_ERR, "[Clam] usb init failed");
		free(info);
		usb_free_cgpu(cgpu);
		return false;
	}
	
	//usb_buffer_enable(cgpu);


	if (!clam_init(cgpu))
	{
		usb_uninit(cgpu);
		free(info);
		cgpu->device_data = NULL;
		usb_free_cgpu(cgpu);
		return false;
	}
	add_cgpu(cgpu);
	return true;
}

static void clam_detect(void)
{
	usb_detect(&clam_drv, clam_detect_one);
}

//int cn = 0;
//int ct = 0;
//int cw = 0;

static int64_t clam_scanwork(struct thr_info *thr)
{
	struct cgpu_info *cgpu = thr->cgpu;
	struct clam_info *info = cgpu->device_data;
	struct timeval tv_timeout;

	if (cgpu->usbinfo.nodev)
		return -1;
/*
	if (cw % 50 == 0)
	{
		uint32_t n,r;
		usb_transfer_read(cgpu, CLAM_TYPE_IN, CLAM_REQUEST_WORK_QUEUE, 0, 0, &n, sizeof(n), &r, C_CLAM_CHANNELS);

		applog(LOG_ERR, "[Clam Debug] cw:%d, cn:%d, ct:%d, workqueue:%d", cw, cn, ct, n);
	}
*/
	struct clam_result result;
	if (unlikely(!clam_read_result(cgpu, &result, 2000)))	// 2s is long enough for single core to respond or timeout
	{
		applog(LOG_ERR, "[Clam] controller failure, reset all");

		thr->work_restart = true;
		return 0;
	}
	else
	{
		struct channel_info *ch_info = &info->channels[result.channel_id];
		if (result.type == CLAM_RESULT_TYPE_NONCE)
		{
		//	cn++;
			applog(LOG_DEBUG, "[Clam] nonce found [%08x]", result.result);

			//try submit
			int i;
			bool found = false;
			for (i = 0; i< info->array_top - 1; i++)
			{
				if (test_nonce(info->work_array[i], result.result))
				{
					//if (i > 5)
						//applog(LOG_ERR, "[Clam] Submit for work %d, %08x", i, nonce);
					if (!submit_nonce(thr, info->work_array[i], result.result))
						applog(LOG_ERR, "[Clam] unexpceted submit failure.");
					found = true;
					ch_info->cont_timeout = 0;
					ch_info->cont_hw = 0;

					break;
				}
			}
			if (!found)
			{
				//must submit for the HW count
				submit_nonce(thr, info->work_array[0], result.result);
				ch_info->cont_hw++;
				applog(LOG_DEBUG, "[Clam] HW error, reset all, %08x", result.result);
/*
				if (ch_info->cont_hw > 20)
				{
					applog(LOG_ERR, "[Clam] continous HW error, reset channel %d", result.channel_id);
					reset_channel(cgpu, result.channel_id);
				}
*/
			}

			//estimate the hashes
			int64_t last_nonce = ch_info->last_nonce;
			int64_t new_nonce = result.result;
			int64_t range_width = 0xffff / ch_info->core_count + 1;
			range_width <<= 16;
			int64_t hashes = (new_nonce % range_width + range_width - last_nonce % range_width) % range_width;
			hashes *= ch_info->core_count;

			ch_info->last_nonce = result.result;
			info->controller_queue_size--;

			return hashes;
		}
		else if (result.type == CLAM_RESULT_TYPE_TIMEOUT)
		{
			//applog(LOG_ERR, "[Clam Debug] got timeout");
			info->controller_queue_size -= 2;
			ch_info->cont_timeout++;
/*
			if (ch_info->cont_timeout > 40)
			{
				applog(LOG_ERR, "[Clam] continous timeout, reinit channel %d", result.channel_id);
				reset_channel(cgpu, result.channel_id);
			}
*/
			ch_info->last_nonce = 0;
		//	ct++;
			return 0x100000000 / 2;
		}
		else
		{
			applog(LOG_ERR, "[Clam] Protocol error : %08x", result.type);
			return 0;
		}
	}
}

static bool clam_queue_full(struct cgpu_info *cgpu)
{
	//send work to the device
	struct clam_info *info = cgpu->device_data;
	struct work *work = get_queued(cgpu);

	if (cgpu->usbinfo.nodev)
		return false;

	if (!unlikely(write_work(cgpu, work->midstate, work->data + 64)))
	{
		applog(LOG_ERR, "[Clam] send work error, discarding current work.");
		work_completed(cgpu, work);
		return false;
	}
	info->work_count++;
	info->controller_queue_size++;

	if (info->array_top == WORK_ARRAY_SIZE)
	{
		//full
		work_completed(cgpu, info->work_array[info->array_top - 1]);
	}
	else
	{
		info->array_top++;
	}
	int i;
	for (i = info->array_top - 1; i > 0 ;i--)
		info->work_array[i] = info->work_array[i-1];
	info->work_array[0] = work;

	if (info->work_count % 1000 == 0)
	{
		//recalibrate queue size
		uint32_t n,r;
		int ret = usb_transfer_read(cgpu, CLAM_TYPE_IN, CLAM_REQUEST_WORK_QUEUE, 0, 0, (char *)&n, sizeof(n), &r, C_CLAM_CHANNELS);
		if (ret == LIBUSB_SUCCESS && r == 4)
		{
			info->controller_queue_size = n;
		}
	}	
		

	return info->controller_queue_size >= CONTROLLER_QUEUE_TARGET_SIZE;
}

static void clam_flush_work(struct cgpu_info *cgpu)
{
	struct clam_info *info = cgpu->device_data;
	applog(LOG_DEBUG, "[Clam] flush work");
	int ret = usb_transfer(cgpu, CLAM_TYPE_OUT, CLAM_REQUEST_FLUSH_WORK, 0, 0, C_CLAM_FLUSH_WORK);
	if (ret != LIBUSB_SUCCESS)
		applog(LOG_ERR, "[Clam] flush work failed, %d", ret);
	info->controller_queue_size = 0;
	//flush_buffer(cgpu);
}

static void clam_thread_shutdown(struct thr_info *thr)
{
	struct cgpu_info *cgpu = thr->cgpu;
	cgsleep_ms(2000);
	flush_result(cgpu);
	applog(LOG_NOTICE, "[Clam] shutdown");
}

char *set_clam_clock(char *arg)
{
	int clock;
	if (sscanf(arg, "%d", &clock) < 1)
		return "no clock value passed";
	if (clock < CLAM_MIN_CLOCK || clock > CLAM_MAX_CLOCK)
		return "invalid clam clock value";
	opt_clam_clock = clock;
	return NULL;
}

static struct api_data *clam_api_stats(struct cgpu_info *cgpu)
{
        struct clam_info *info = cgpu->device_data;
        struct api_data *root = NULL;
        int i;
        char buf1[50], buf2[100];

        for (i=0; i<info->channel_count; i++)
        {
        	sprintf(buf1, "Channel%d", i);
        	sprintf(buf2, "%d chips %d cores core map %s",
        			info->channels[i].chip_count,
        			info->channels[i].core_count,
        			bin2hex(info->channels[i].core_map, info->channels[i].chip_count));
        	root = api_add_string(root, buf1, buf2, true);
        }

        return root;
}


struct device_drv clam_drv = {
	.drv_id = DRIVER_CLAM,
	.dname = "clam",
	.name = "CM",
	.drv_detect = clam_detect,
	.get_api_stats = clam_api_stats,
	.hash_work = hash_queued_work,
	.scanwork = clam_scanwork,
	.queue_full = clam_queue_full,
	.reinit_device = clam_reinit,
	.flush_work = clam_flush_work,
	.thread_shutdown = clam_thread_shutdown
};
