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
int opt_clam_queue = 60;
bool opt_clam_test_only = false;

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
	result->nonce = be32toh(result->nonce);
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

void update_queue_avail(struct cgpu_info* cgpu)
{
	struct clam_info* info = cgpu->device_data;
	uint32_t n, read;
	int ret = usb_transfer_read(cgpu, CLAM_TYPE_IN, CLAM_REQUEST_WORK_QUEUE, 0, CLAM_REQUEST_INDEX(0, 0), (char *)&n, sizeof(n), &read, C_CLAM_WORK_QUEUE);
	if (ret != LIBUSB_SUCCESS || read != sizeof(n))
	{
		applog(LOG_ERR, "[Clam] read work queue failed, %d", ret);
	}
	else
	{
		info->controller_queue_available = le32toh(n);
	}
}
static bool write_work(struct cgpu_info *cgpu, uint32_t sequence, unsigned char *midstate, unsigned char *data)
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
	struct clam_work cw;
	int i;
	for (i=0;i<32;i++)
		cw.midstate[i] = midstate[31-i];
	for (i=0;i<12;i++)
		cw.data[i] = data[11-i];
	cw.sequence = htole32(sequence);

	if (unlikely(!clam_write(cgpu, &cw, sizeof(cw), C_CLAM_WRITE_WORK)))
	{
		applog(LOG_ERR, "[Clam] work write error");
		return false;
	}
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

	memset(info, 0, sizeof(*info));

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
	strcpy(info->firmware_version, buf+10);
	applog(LOG_NOTICE, "[Clam] found device [%s]", buf);

	//identify channels
	ret = usb_transfer_read(cgpu, CLAM_TYPE_IN, CLAM_REQUEST_CHANNELS, 0, CLAM_REQUEST_INDEX(0, 0), buf, sizeof(buf), &read, C_CLAM_CHANNELS);
	if (ret != LIBUSB_SUCCESS || read != 4)
	{
		applog(LOG_ERR, "[Clam] read device channels error - [%s], ignored.", buf);
		return false;
	}
	info->channel_count = le32toh(*(uint32_t *)buf);

	//identify chip and cores
	int channel_id;
	for (channel_id = 0; channel_id<info->channel_count; channel_id++)
	{
		uint32_t clock = htole32(opt_clam_clock);
		ret = usb_transfer(cgpu, CLAM_TYPE_OUT, CLAM_REQUEST_CLOCK, clock, CLAM_REQUEST_INDEX(0, channel_id), C_CLAM_CLOCK);
		if (ret != LIBUSB_SUCCESS)
		{
			applog(LOG_ERR, "[Clam] channel %d : set clock error - [%d], ignored.", channel_id, ret);
			return false;
		}

		ret = usb_transfer(cgpu, CLAM_TYPE_OUT, CLAM_REQUEST_RESET_CHANNEL, 0, CLAM_REQUEST_INDEX(0, channel_id), C_CLAM_RESET_CHANNEL);
		if (ret != LIBUSB_SUCCESS)
		{
			applog(LOG_ERR, "[Clam] channel %d : reset error - [%d], ignored.", channel_id, ret);
			return false;
		}

		ret = usb_transfer_read(cgpu, CLAM_TYPE_IN, CLAM_REQUEST_CORES, 0, CLAM_REQUEST_INDEX(0, channel_id), buf, sizeof(buf), &read, C_CLAM_CORES);
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
	}

	if (opt_clam_test_only)
		return false;

	flush_result(cgpu);
	update_queue_avail(cgpu);
	return true;
}

static void clam_reinit(struct cgpu_info  *cgpu)
{
	//clean the queued work
	struct clam_info *info = cgpu->device_data;
/*	int i;
	for (i=0;i<info->array_top;i++)
		work_completed(cgpu, info->work_array[i]);
	info->array_top = 0;
*/
	reset_controller(cgpu);
	clam_init(cgpu);
}

static struct cgpu_info *clam_detect_one(struct libusb_device *dev, struct usb_find_devices *found)
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
		return usb_free_cgpu(cgpu);
	}
	
//	char buf[64];
//	int read;
//	int i;
//	int ret = usb_transfer_read(cgpu, CLAM_TYPE_IN, CLAM_REQUEST_VERSION, 0, 0, buf, sizeof(buf), &read, C_CLAM_VERSION);
//	if (ret != LIBUSB_SUCCESS)
//	{
//		applog(LOG_ERR, "[Clam] usb error:%d", ret);
//		return false;
//	}
//	buf[read] = 0;
//
//	applog(LOG_ERR, "version:%s", buf);
//
//	ret = usb_transfer_read(cgpu, CLAM_TYPE_IN, CLAM_REQUEST_CHANNELS, 0, 0, buf, sizeof(buf), &read, C_CLAM_VERSION);
//	if (ret != LIBUSB_SUCCESS)
//	{
//		applog(LOG_ERR, "[Clam] usb error:%d", ret);
//		return false;
//	}
//	buf[read] = 0;
//
//	int channels = *((uint32_t *)buf);
//	applog(LOG_ERR, "channels:%d", *((uint32_t *)buf));
//
//	ret = usb_transfer_read(cgpu, CLAM_TYPE_IN, CLAM_REQUEST_CORES, 0, 0, buf, sizeof(buf), &read, C_CLAM_VERSION);
//	if (ret != LIBUSB_SUCCESS)
//	{
//		applog(LOG_ERR, "[Clam] usb error:%d", ret);
//		return false;
//	}
//	buf[read] = 0;
//	applog(LOG_ERR, "channel0map:%s", bin2hex(buf, read));
//
//	ret = usb_transfer(cgpu, CLAM_TYPE_OUT, CLAM_REQUEST_RESET_CHANNEL, 0, 0, C_CLAM_VERSION);
//	if (ret != LIBUSB_SUCCESS)
//	{
//		applog(LOG_ERR, "[Clam] usb error:%d", ret);
//		return false;
//	}
//
//	cgsleep_ms(2000);
//	ret = usb_transfer_read(cgpu, CLAM_TYPE_IN, CLAM_REQUEST_CORES, 0, 0, buf, sizeof(buf), &read, C_CLAM_VERSION);
//	if (ret != LIBUSB_SUCCESS)
//	{
//		applog(LOG_ERR, "[Clam] usb error:%d", ret);
//		return false;
//	}
//	buf[read] = 0;
//	applog(LOG_ERR, "channel0map:%s", bin2hex(buf, read));
//
//
//	if (1==1)
//		return false;
//	struct clam_work work;
//	struct clam_result2 result;
//	*((uint32_t *)work.midstate) = 0x12345678;
//	int wrote,read;
//	int i;
//	struct clam_work *ws = calloc(50, sizeof(struct clam_work));
//	for (i=0;i<10000;i++)
//	{
//		int ret = usb_write_timeout(cgpu, (char *)ws, sizeof(struct clam_work) * 50, &wrote, 10000, C_CLAM_CHANNELS);
//		if (i%1000 == 0)applog(LOG_ERR, "%d:wrote:%d", i, wrote);
//		if (ret != LIBUSB_SUCCESS)
//		{
//			applog(LOG_ERR, "[Clam] usb error:%d", ret);
//			return false;
//		}

//		ret = usb_read_timeout(cgpu, (char *)&result, sizeof(result), &read, 1000, C_CLAM_CHANNELS);
//		if (ret != LIBUSB_SUCCESS)
//		{
//			applog(LOG_ERR, "[Clam] usb error:%d", ret);
//			return false;
//		}

	//	applog(LOG_ERR, "read:%d, dat:%08x", read, result.nonce);
//	}

//	if (1==1)
//		return false;

//	reset_controller(cgpu);

	usb_buffer_clear(cgpu);

	if (!clam_init(cgpu))
		goto failed;

	if (!add_cgpu(cgpu))
		goto failed;

	return cgpu;

	failed:
	usb_uninit(cgpu);
	free(info);
	cgpu->device_data = NULL;
	return usb_free_cgpu(cgpu);
}

static void clam_detect(bool hotplug)
{
	usb_detect(&clam_drv, clam_detect_one);
}

static int64_t clam_scanwork(struct thr_info *thr)
{
	struct cgpu_info *cgpu = thr->cgpu;
	struct clam_info *info = cgpu->device_data;
	struct timeval tv_timeout;

	if (cgpu->usbinfo.nodev)
		return -1;
	struct clam_result result;
	uint32_t read;
	int err = usb_read_timeout(cgpu, (char *)&result, sizeof(result), &read, 10000, C_CLAM_READ_DATA);
	if (err == LIBUSB_ERROR_TIMEOUT)
	{
		//cgsleep_us(0xffffffff / 8 / 32 / opt_clam_clock * opt_clam_queue / 2);
		applog(LOG_WARNING, "[Clam] Read timeout");
		return 0;
	}
	if (err != LIBUSB_SUCCESS)
	{
		applog(LOG_ERR, "[Clam] controller failure, reset all, read:%d, err:%d", read, err);

		thr->work_restart = true;
		return 0;
	}
	if (usb_buffer_size(cgpu) > 0)
	{
		applog(LOG_ERR, "[Clam Debug] buffer size : %d", usb_buffer_size(cgpu));
	}
	
	if (result.miner_id == 0xff)
	{
		//queue sync
		info->controller_queue_available = le32toh(result.work_queue_avail);
		return 0;
	}
	else
	{
		uint32_t sequence = le32toh(result.work_sequnece);
		info->controller_queue_available = le32toh(result.work_queue_avail);
		if (sequence + WORK_ARRAY_SIZE < info->work_counter)
		{
			applog(LOG_WARNING, "[Clam] work array is not big enough");
			return 0;
		}
		else
		{
			uint32_t index = sequence % WORK_ARRAY_SIZE;
			uint32_t nonce = be32toh(result.nonce);
			uint32_t nonce2 = le32toh(result.nonce);
//			applog(LOG_ERR, "[Clam Debug] index:%d, %s, %s", index,
//					test_nonce(info->work_array[index], nonce) ? "true" : "false",
//					test_nonce(info->work_array[index], nonce2) ? "true" : "false"
//						);
			submit_nonce(thr, info->work_array[index], nonce);
		//	applog(LOG_ERR, "[Clam Debug] queue %d!", info->controller_queue_available);
			return 0xfffffffful;
		}
	}
//	struct channel_info *ch_info = &info->channels[result.channel_id];
/*
		if (result[r].type == CLAM_RESULT_TYPE_NONCE)
		{
			uint32_t nonce = be32toh(result[r].result);
			info->controller_queue_size = result[r].queue_size;
			//applog(LOG_ERR, "[Clam Debug] queue size:%d", info->controller_queue_size);
			applog(LOG_DEBUG, "[Clam] nonce found [%08x]", nonce);
			//try submit
			int i;
			bool found = false;
			for (i = 0; i< info->array_top - 1; i++)
			{
				if (test_nonce(info->work_array[i], nonce))
				{
					//if (i > 5)
						//applog(LOG_ERR, "[Clam] Submit for work %d, %08x", i, nonce);
					if (!submit_nonce(thr, info->work_array[i], nonce))
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
				submit_nonce(thr, info->work_array[0], nonce);
				ch_info->cont_hw++;
				applog(LOG_DEBUG, "[Clam] HW error, reset all, %08x", nonce);
				if (ch_info->cont_hw > 20)
				{
					applog(LOG_WARNING, "[Clam] continous HW error, reset channel %d", result[r].channel_id);
					ch_info->cont_hw = 0;
					//thr->work_restart = true;
				}
			}
			else
			{
				total_hashes += 0xffffffff;
			}
		}
		else
		{
			//applog(LOG_ERR, "[Clam] Protocol error : %08x", result[r].type);
		}
		*/
//	}
}

static bool clam_queue_full(struct cgpu_info *cgpu)
{
	//send work to the device
	struct clam_info *info = cgpu->device_data;

	if (cgpu->usbinfo.nodev)
		return false;

	if (!info->controller_queue_available)
		return true;

	uint32_t index = info->work_counter % WORK_ARRAY_SIZE;

	if (info->work_array[index])
	{
		work_completed(cgpu, info->work_array[index]);
		info->work_array[index] = NULL;
	}

	struct work *work = get_queued(cgpu);

	if (!unlikely(write_work(cgpu, info->work_counter, work->midstate, work->data + 64)))
	{
		applog(LOG_ERR, "[Clam] send work error, discarding current work.");
		work_completed(cgpu, work);
		return false;
	}
	info->work_array[index] = work;
	info->work_counter++;
	info->controller_queue_available--;

	return info->controller_queue_available == 0;
}

static void clam_flush_work(struct cgpu_info *cgpu)
{
	struct clam_info *info = cgpu->device_data;
	applog(LOG_NOTICE, "[Clam] flush work queue");
	int ret = usb_transfer(cgpu, CLAM_TYPE_OUT, CLAM_REQUEST_FLUSH_WORK, 0, 0, C_CLAM_FLUSH_WORK);
	if (ret != LIBUSB_SUCCESS)
		applog(LOG_ERR, "[Clam] flush failed, %d", ret);

	update_queue_avail(cgpu);
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
	.drv_id = DRIVER_clam,
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
