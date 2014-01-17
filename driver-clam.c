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

// used for functional core detection
// adopted from driver-icarus.c
// Block 171874 nonce = (0xa2870100) = 0x000187a2
const char *golden_midstate = "4a548fe471fa3a9a1371144556c3f64d2500b4826008fe4bbf7698c94eba7946";
const char *golden_data = "ce22a72f4f6726141a0b3287";
const uint32_t golden_nonce = 0x000187a2;

int opt_clam_clock = CLAM_DEFAULT_CLOCK;
int opt_clam_core_limit = CLAM_MAX_CORE_COUNT;
int opt_clam_chip_start = 0;
int opt_clam_chip_end = 32;
bool opt_clam_no_test = false;

static void flush_buffer(struct cgpu_info *bitfury)
{
	char buf[512];
	int amount;

	do {
		usb_read_once(bitfury, buf, 512, &amount, C_CLAM_FLUSH_BUFFER);
	} while (amount);
}

static bool clam_read(struct cgpu_info *cgpu, int ms_timeout,uint32_t *header, uint32_t *result)
{
	uint32_t buf[2];
	int read;
	
	int err = usb_read_timeout(cgpu, (char *)buf, sizeof(buf), &read, ms_timeout, C_CLAM_READ_DATA);
	if (err != LIBUSB_SUCCESS || read != sizeof(buf))
	{
		applog(LOG_ERR, "[Clam Debug] read error:%d, read:%d", err, read);
		return false;
	}
	//change endian
	*header = buf[0];
	*result = be32toh(buf[1]);
	return true;
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

static bool set_timeout(struct cgpu_info *cgpu, uint8_t channel_id, uint32_t timeout_us)
{
	//set the chip timeout for controller
	uint8_t buf[1 + 1 + 4];
	buf[0] = 'T';
	buf[1] = channel_id;
	*((uint32_t *)(buf + 2)) = timeout_us;
	return clam_write(cgpu, buf, sizeof(buf), C_CLAM_SET_TIMEOUT);
}

static bool reset_controller(struct cgpu_info *cgpu)
{
	return clam_write(cgpu, "Z", 1, C_CLAM_RESET_CONTROLLER);
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
	unsigned char buf[32 + 12 + 1];
	unsigned char *p = buf;
	*(p++) = 'W';
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

static bool write_register(struct cgpu_info *cgpu, uint8_t channel_id, uint8_t chip_id, uint8_t address, uint8_t value)
{
	applog(LOG_DEBUG, "[Clam] write register [%02x]/[%02x] : %02x", chip_id, address, value);
	//write chip id
	char cmd[5], *p = cmd;
	*(p++) = 'S';
	*(p++) = channel_id;
	*(p++) = chip_id;
	*(p++) = (address << 1) | 0x01;	//write mode;
	*(p++) = value;

	if (unlikely(!clam_write(cgpu, cmd, sizeof(cmd), C_CLAM_WRITE_REGISTER)))
	{
		applog(LOG_ERR, "[Clam] write register value %02x to %02x/%02x failed", value, chip_id, address);
		return false;
	}
	return true;
}


static bool request_register(struct cgpu_info *cgpu, uint8_t channel_id, uint8_t chip_id, uint8_t address)
{
	//register read access sequence
	//extracted this function from read_register to support chip detect operation
	char cmd[4], *p = cmd;
	*(p++) = 'R';
	*(p++) = channel_id;
	*(p++) = chip_id;
	*(p++) = address << 1;	//read mode;

	if (unlikely(!clam_write(cgpu, cmd, sizeof(cmd), C_CLAM_READ_REGISTER)))
	{
		applog(LOG_ERR, "[Clam] request chip_id [%02x] failed", chip_id);
		return false;
	}

	return true;
}


static bool read_register(struct cgpu_info *cgpu, uint8_t channel_id, uint8_t chip_id, uint8_t address, uint8_t *result)
{
	uint32_t data, header;

	applog(LOG_DEBUG, "[Clam Debug] read register [%02x]/[%02x]", chip_id, address);
	if (unlikely(!request_register(cgpu, channel_id, chip_id, address)))
		return false;

	if (unlikely(!clam_read(cgpu, DEVTIMEOUT, &header, &data)))
	{
		applog(LOG_ERR, "[Clam] read register value failed");
		return false;
	}
	if (unlikely((header & 0xffffff) != 0))
	{
		applog(LOG_ERR, "[Clam] read register value error: header - %08x", header);
		return false;
	}
	applog(LOG_DEBUG, "[Clam Debug] register data [%08x]", data);
	*result = (data >> ((3 - (address & 0x3)) * 8)) & 0xff;
	return true;
}

static bool set_pll_simple(struct cgpu_info *cgpu, uint8_t channel_id, uint8_t chip_id, int frequency)
{
	//default method to modify only M value
	uint8_t od = CLAM_PLL_DEFAULT_OD;
	uint8_t n = CLAM_PLL_DEFAULT_N;

	if (frequency >= 360)
		od = 1;

	uint8_t m = (frequency << od) / (CLAM_PLL_DEFAULT_XIN / n);
	applog(LOG_DEBUG, "[Clam] set PLL M/N/OD value to %02x/%02x/%02x", m, n, od);
	if (unlikely(!write_register(cgpu, channel_id, chip_id, CLAM_REG_PLL1, m)))
		return false;
	uint8_t pll2 = (n << 4) | ((od & 0x3) << 2);
	if (unlikely(!write_register(cgpu, channel_id, chip_id, CLAM_REG_PLL2, pll2)))
		return false;
	//PLL should be reset after registers have been set
	if (unlikely(!write_register(cgpu, channel_id, chip_id, CLAM_REG_PLL2, pll2 | 0x01)))
		return false;

	opt_clam_clock = CLAM_PLL_DEFAULT_XIN / n * m >> od;
	applog(LOG_ERR, "[Clam Debug] set PLL to %d MHz", opt_clam_clock);
	return true;

}

static bool send_test_work(struct cgpu_info *cgpu)
{
	unsigned char midstate[32];
	unsigned char data[12];
	hex2bin(midstate, golden_midstate, sizeof(midstate));
	hex2bin(data, golden_data, sizeof(data));

	if (unlikely(!write_work(cgpu, midstate, data)))
	{
		applog(LOG_ERR, "[Clam] write test work failed");
		return false;
	}

	uint32_t header, nonce;
	if (unlikely(!clam_read(cgpu, DEVTIMEOUT, &header, &nonce)))
	{
		applog(LOG_ERR, "[Clam] read test nonce failed");
		return false;
	}
	if (header == 0xffffffff)
	{
		applog(LOG_ERR, "[Clam] test work timeout");
		return false;
	}
	if (nonce != golden_nonce)
	{
		applog(LOG_ERR, "[Clam] returned nonce [%08x] do not match the golden nonce [%08x]", nonce, golden_nonce);
		return false;
	}
	else
	{
		applog(LOG_DEBUG, "[Clam] test work OK");
	}

	return true;
}

static bool set_core_range(struct cgpu_info *cgpu, uint8_t channel_id, uint8_t chip_id, int core, uint16_t low, uint16_t high)
{
	applog(LOG_DEBUG, "[%02x]-[%02x] scan range:%04x0000 - %04xffff", chip_id, core, low, high);
	if (!write_register(cgpu, channel_id, chip_id, CLAM_REG_CORE0_RANGE_LOW + core * 4, low & 0xff) ||
		!write_register(cgpu, channel_id, chip_id, CLAM_REG_CORE0_RANGE_LOW + core * 4 + 1, low >> 8) ||
		!write_register(cgpu, channel_id, chip_id, CLAM_REG_CORE0_RANGE_LOW + core * 4 + 2, high & 0xff) ||
		!write_register(cgpu, channel_id, chip_id, CLAM_REG_CORE0_RANGE_LOW + core * 4 + 3, high >> 8))
	{
		applog(LOG_ERR, "[%02x]-[%02x] set scan range failed", chip_id, core);
		return false;
	}

	if (unlikely(!write_register(cgpu, channel_id, chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_RANGE_INITIAL)))
	{
		return false;
	}
	return true;
}

static bool set_core_mask(struct cgpu_info *cgpu, uint8_t channel_id, uint8_t chip_id, uint8_t core_mask)
{
	applog(LOG_DEBUG, "[%02x] set core mask %02x", chip_id, core_mask);
	return write_register(cgpu, channel_id, chip_id, CLAM_REG_CORE_MASK, core_mask);
}

static bool assign_cores(struct cgpu_info *cgpu, uint8_t channel_id)
{
	struct clam_info *c = cgpu->device_data;
	struct channel_info *info = &c->channels[channel_id];
	//set ranges
	int range_width = 0x10000 / info -> core_count;
	int range = 0;
	int i;
	for (i=0; i<sizeof(info->core_map); i++)
	{
		int j;
		for (j=0;j<opt_clam_core_limit;j++)
		{
			if (1<<j & info->core_map[i])
			{
				if (unlikely(!set_core_range(cgpu, channel_id, i, j, range, range + range_width - 1)))
					return false;

				range += range_width;
			}
		}
	}
	return true;
}

static bool reset_all(struct cgpu_info *cgpu, uint8_t channel_id)
{
	if (unlikely(!write_register(cgpu, channel_id, CLAM_CHIP_ID_ALL, CLAM_REG_GENERAL_CONTROL, CLAM_GC_RESET)))
	{
		applog(LOG_ERR, "[Clam] reset all failed");
		return false;
	}

	return true;
}

static bool detect_cores(struct cgpu_info *cgpu, uint8_t channel_id)
{
	struct clam_info *c_info = cgpu->device_data;
	struct channel_info *info = &c_info->channels[channel_id];

	//assume the max number of chips, reset all chips and read chip_id, wait response to detect chips
	int i;

	//loop all the chips as independent and reset all

	if (unlikely(!write_register(cgpu, channel_id, CLAM_CHIP_ID_ALL, CLAM_REG_GENERAL_CONTROL,
			CLAM_GC_LOOP_DOWN_M|
			CLAM_GC_LOOP_DOWN_S|
			CLAM_GC_LOOP_UP_M|
			CLAM_GC_LOOP_UP_S|
			CLAM_GC_RESET)))		//all loop bits
	{
		applog(LOG_ERR, "[Clam] reset all for detection failed");
		return false;
	}

	uint8_t chip_ids[CLAM_MAX_CHIP_COUNT];

	for (i = opt_clam_chip_start; i < opt_clam_chip_end; i++)
	{
		//send read chip_id reg command
		if (unlikely(!request_register(cgpu, channel_id, i, CLAM_REG_CHIP_ID)))
		{
			applog(LOG_ERR, "[Clam] send read chip_id command failed [%02x]", i);
			return false;
		}
	}

	//read all returned data until timeout
	while(42)
	{
		uint32_t header, result;
		if (!clam_read(cgpu, DEVTIMEOUT, &header, &result))
			break;
		uint8_t chip_id = (result >> ((3 - (CLAM_REG_CHIP_ID & 0x3)) * 8)) & 0xff;
		chip_ids[info->chip_count++] = chip_id;
		applog(LOG_NOTICE, "[Clam] Chip 0x%02x found!", chip_id);
	}
	applog(LOG_NOTICE, "[Clam] %d chips detected", info->chip_count);
//	if (info->chip_count == 0)
//		return false;

	//test for inter-chip communications
	//assume the chip ids is ordered as physical position

	//        down          up        down          up
	//            |--------|              |--------|
	//----<<----MR|        |MR----<<----MR|        |MR----<<----
	//---->>----MA|chip i+1|MA---->>----MA| chip i |MA---->>----
	//---->>----SR|        |SR---->>----SR|        |SR---->>----
	//----<<----SA|        |SA----<<----SA|        |SA----<<----
	//            |--------|              |--------|

	for (i = 0; i<info->chip_count-1; i++)
	{
		uint8_t chip_id = chip_ids[i];
		uint8_t next_chip_id = chip_ids[i+1];
		uint8_t result;
		applog(LOG_DEBUG, "[Clam] test for chip 0x%02x -> 0x%02x lines", chip_id, next_chip_id);

		if (unlikely(!write_register(cgpu, channel_id, chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_LOOP_DOWN_S|CLAM_GC_LOOP_UP_S|CLAM_GC_LOOP_UP_M))||
			unlikely(!read_register(cgpu, channel_id, chip_id, CLAM_REG_CHIP_ID, &result))||
			unlikely(chip_id != result)||
			unlikely(!write_register(cgpu, channel_id, chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_LOOP_DOWN_S|CLAM_GC_LOOP_UP_S|CLAM_GC_LOOP_UP_M|CLAM_GC_LOOP_DOWN_M)))	//restore loops
		{
			applog(LOG_ERR, "[Clam] M line test failed. chip 0x%02x -> 0x%02x", chip_id, next_chip_id);
			return false;
		}

		if (unlikely(!write_register(cgpu, channel_id, next_chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_LOOP_UP_M|CLAM_GC_LOOP_DOWN_M|CLAM_GC_LOOP_DOWN_S))||
			unlikely(!read_register(cgpu, channel_id, next_chip_id, CLAM_REG_CHIP_ID, &result))||
			unlikely(next_chip_id != result)||
			unlikely(!write_register(cgpu, channel_id, next_chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_LOOP_DOWN_S|CLAM_GC_LOOP_UP_S|CLAM_GC_LOOP_UP_M|CLAM_GC_LOOP_DOWN_M)))	//restore loops
		{
			applog(LOG_ERR, "[Clam] S line test failed. chip 0x%02x -> 0x%02x", chip_id, next_chip_id);
			return false;
		}

	}
	applog(LOG_NOTICE, "[Clam] Line test passed.");

	if (!unlikely(write_register(cgpu, channel_id, CLAM_CHIP_ID_ALL, CLAM_REG_GENERAL_CONTROL, CLAM_GC_RESET)))
		return false;

	//detect functional cores
	//first disable all chips' cores
	if (unlikely(!set_core_mask(cgpu, channel_id, CLAM_CHIP_ID_ALL, 0)))
		return false;

	//uint32_t timeout_us = 0xffffffff / opt_clam_clock / golden_nonce * 20;//one core doube time
	if (unlikely(!set_timeout(cgpu, channel_id, 100000 )))
		return false;
	//then enable every single core to try golden nonce
	for (i=0; i< info->chip_count; i++)
	{
		uint8_t chip_id = chip_ids[i];

		uint8_t mask = 1;
		int j;
		for (j=0;j<opt_clam_core_limit;j++)
		{
			if (!opt_clam_no_test)
			{
				//send test work to try the hash core
				//core must be enabled BEFORE core range being set
				if (unlikely(!write_register(cgpu, channel_id, chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_RESET))||	//reset first
					unlikely(!set_core_mask(cgpu, channel_id, chip_id, mask))||			//enable the core only
					unlikely(!set_core_range(cgpu, channel_id, chip_id, j, 0x0000, 0xffff))||	//set the full range
					unlikely(!send_test_work(cgpu)))							//send test golden work
				{
					applog(LOG_ERR, "[Clam Debug] chip [%02x] core [%02x] test failed, ignored", chip_id, j);
				}
				else
				{
					//funtional core found
					applog(LOG_DEBUG, "[Clam Debug] Funtional core found:[%02x]/[%02x]", chip_id, j);
					info->core_count++;
					info->core_map[chip_id] |= mask;
				}
			}
			else
			{
					info->core_count++;
					info->core_map[chip_id] |= mask;
			}
			mask <<=1;

		}

		//disable cores
		if (unlikely(!set_core_mask(cgpu, channel_id, chip_id, 0)))
			return false;
	}

	applog(LOG_NOTICE, "[Clam] %d functional cores found.", info->core_count);

	//enable all functional cores
	for (i=0;i<info->chip_count;i++)
	{
		int chip_id = chip_ids[i];

		applog(LOG_DEBUG, "[Clam] chip 0x%02x map:0x%02x", chip_id, info->core_map[chip_id]);
		if (unlikely(!set_core_mask(cgpu, channel_id, chip_id, info->core_map[chip_id])))
			return false;

		/*
		if (!info->core_map[chip_id])
		{
			//bypass
			if (unlikely(!write_register(cgpu, channel_id, chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_CHIP_BYPASS)))
				return false;
			info->chip_bypass[i] = true;
		}
		*/
	}

	return true;
}

static bool clam_reinit_channel(struct cgpu_info *cgpu, uint8_t channel_id)
{
	struct clam_info *info = cgpu->device_data;

	//flush_buffer(cgpu);

	reset_controller(cgpu);
	if (unlikely(!set_pll_simple(cgpu, channel_id, CLAM_CHIP_ID_ALL, opt_clam_clock)))
	{
		applog(LOG_ERR, "[Clam] set PLL error");
		return false;
	}

	if (!assign_cores(cgpu, channel_id))
	{
		applog(LOG_ERR, "[Clam] assign cores error");
		return false;
	}
	reset_all(cgpu, channel_id);
	info->channels[channel_id].cont_timeout = 0;
	return true;
}

static void clam_reinit(struct cgpu_info  *cgpu)
{
	uint8_t channel_id;
	for (channel_id = 0; channel_id< 4; channel_id++)
	{
		clam_reinit_channel(cgpu, channel_id);
	}
}

static bool clam_init_channel(struct cgpu_info *cgpu, uint8_t channel_id)
{
	struct clam_info *info = cgpu->device_data;
	if (cgpu->usbinfo.nodev)
		return false;

	applog(LOG_NOTICE, "[Clam] start to initilise CLAM Miner, channel %d", channel_id);
	
	//usb_buffer_enable(cgpu);
	//flush_buffer(cgpu);
/*
	if (unlikely(!set_pll_simple(cgpu, channel_id, CLAM_CHIP_ID_ALL, opt_clam_clock)))
	{
		applog(LOG_ERR, "[Clam] set PLL error");
		return false;
	}

	if (unlikely(!detect_cores(cgpu, channel_id)))
		return false;
	if (info->channels[channel_id].core_count == 0)
		return true;
*/
/*	if (!info->channels[channel_id].core_count)
	{
		applog(LOG_ERR, "[Clam] no functional core found");
		return false;
	}
*/
	//set ranges
//	if (unlikely(!assign_cores(cgpu, channel_id)))
//		return false;
//
//	uint32_t timeout_us = 0xffffffff / info->channels[channel_id].core_count / opt_clam_clock;	//base timeout
//	//timeout_us += (44 + 4)/* work + return nonce */ * (1000000 * 10 / 115200) /* us per byte */;
//	if (unlikely(!set_timeout(cgpu, channel_id, timeout_us)))
//		return false;
//
//	uint8_t enable_cmd[2];
//	enable_cmd[0] = 'E';
//	enable_cmd[1] = channel_id;
//	clam_write(cgpu, enable_cmd, sizeof(enable_cmd), C_CLAM_CHANNEL_ENABLE);
	return true;
}

static bool clam_thread_init(struct thr_info *thr)
{
	struct cgpu_info *cgpu = thr->cgpu;
	struct clam_info *info = cgpu->device_data;

	if (cgpu->usbinfo.nodev)
	{
		return false;
	}
	//reset_controller(cgpu);
	uint8_t i;
//	int cores = 0;
//	for (i=0;i<4;i++)
//	{
//		if (!clam_init_channel(cgpu, i))
//			return false;
//		cores += info->channels[i].core_count;
//	}
//	if (!cores)
//	{
//		applog(LOG_ERR, "[Clam] no functional core found");
//		return false;
//	}

	return true;
}


static bool clam_detect_one(struct libusb_device *dev, struct usb_find_devices *found)
{
	struct cgpu_info *cgpu;
	struct clam_info *info = calloc(1, sizeof(*info));

	cgpu = usb_alloc_cgpu(&clam_drv, 1);

	cgpu->drv = &clam_drv;
	cgpu->device_data = info;
	cgpu->threads = 1;
	add_cgpu(cgpu);

	if (!usb_init(cgpu, dev, found))
	{
		applog(LOG_ERR, "[Clam] usb init failed");
		usb_uninit(cgpu);
		free(info);
		usb_free_cgpu(cgpu);
		return false;
	}
	
	char buf[100];
	int read;
	memset(buf, 0, sizeof(buf));
	int ret = usb_transfer_read(cgpu, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN, 0x10, 0, 0, buf, 100, &read, C_CLAM_READ_DATA);
	applog(LOG_ERR, "ret:%d", ret);
	applog(LOG_ERR, "read:%d", read);
	applog(LOG_ERR, "version:%s", buf);

	return true;
}

static void clam_detect(void)
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

	uint32_t header, nonce;
	if (unlikely(!clam_read(cgpu, 2000, &header, &nonce)))	// 2s is long enough for single core to respond or timeout
	{
		applog(LOG_ERR, "[Clam] controller failure, reset all");

		thr->work_restart = true;
		return 0;
	}
	else
	{
		uint8_t channel_id = header >> 24;
		struct channel_info *ch_info = &info->channels[channel_id];
		header &= 0xffffff;
		if (header == 0)
		{
			applog(LOG_DEBUG, "[Clam] nonce found [%08x]", nonce);
			ch_info->cont_timeout = 0;

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

					break;
				}
			}
			if (!found)
			{
				//must submit for the HW count
				submit_nonce(thr, info->work_array[0], nonce);
				applog(LOG_DEBUG, "[Clam] HW error, reset all, %08x", nonce);
			}

			//estimate the hashes
			int range = 0x10000/8/8;
			int64_t hashes;
			hashes = (nonce >> 16) % range - (ch_info->last_nonce >> 16) % range;
			hashes = (hashes + range) % range;
			hashes <<= 16;
			hashes = hashes + (nonce & 0xffff) - (ch_info->last_nonce & 0xffff);
			hashes *= ch_info->core_count;

			ch_info->last_nonce = nonce;
			info->controller_queue_size--;

			return hashes;
		}
		else if (header == 0xffffff)
		{
			//applog(LOG_ERR, "[Clam Debug] got timeout");
			info->controller_queue_size -= 2;
			ch_info->cont_timeout++;

			if (ch_info->cont_timeout > 20)
			{
				applog(LOG_ERR, "[Clam] continous timeout, reinit channel %d", channel_id);
				clam_reinit_channel(cgpu, channel_id);
			}
			return 0x100000000;
		}
		else
		{
			applog(LOG_ERR, "[Clam] Protocol error : %08x", header);
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

	return info->controller_queue_size >= CONTROLLER_QUEUE_TARGET_SIZE;
}

static void clam_flush_work(struct cgpu_info *cgpu)
{
	applog(LOG_DEBUG, "[Clam] flush work and reset all");
	char cmd = 'F';
	clam_write(cgpu, &cmd, 1, C_QUEFLUSH);
}

static void clam_thread_shutdown(struct thr_info *thr)
{
	struct cgpu_info *cgpu = thr->cgpu;
	reset_controller(cgpu);
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

struct device_drv clam_drv = {
	.drv_id = DRIVER_CLAM,
	.dname = "clam",
	.name = "CM",
	.drv_detect = clam_detect,
	.hash_work = hash_queued_work,
	.scanwork = clam_scanwork,
	.queue_full = clam_queue_full,
	.reinit_device = clam_reinit,
	.thread_init = clam_thread_init,
	.flush_work = clam_flush_work,
	.thread_shutdown = clam_thread_shutdown
};
