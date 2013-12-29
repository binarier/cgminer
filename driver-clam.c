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
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/mman.h>

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

static int reg_ms_timeout = 10;
static int test_ms_timeout = 100;

int opt_clam_clock = CLAM_DEFAULT_CLOCK;
int opt_clam_core_limit = CLAM_MAX_CORE_COUNT;
int opt_clam_chip_start = 0;
int opt_clam_chip_end = CLAM_MAX_CHIP_COUNT;
bool opt_clam_test = false;

static void flush_buffer(struct cgpu_info *bitfury)
{
	char buf[512];
	int amount;

	do {
		usb_read_once(bitfury, buf, 512, &amount, C_BF1_FLUSH);
	} while (amount);
}

static bool clam_read(struct cgpu_info *cgpu, int ms_timeout, uint32_t *result)
{
	uint32_t buf;
	int r;
	
	struct timeval tv_now, tv_start;
	cgtime(&tv_start);
	
	int err = usb_read_timeout(cgpu, (char *)&buf, 4, &r, ms_timeout, C_BF1_FLUSH);
	if (err != LIBUSB_SUCCESS || r == 0)
	{
		//applog(LOG_ERR, "read error:%d", err);
		return false;
	}
	*result = be32toh(buf);
	return true;
}

static bool clam_write(struct cgpu_info *cgpu, void *data, int size)
{
	int n;
	int err;
	
	if ((err = usb_write(cgpu, data, size, &n, C_CLAM_WRITE_WORK)) != 0)
	{
		applog(LOG_ERR, "[Clam] usb write error:%d", err);
		return false;
	}
	if (n != size)
	{
		applog(LOG_ERR, "[Clam] write error, read size %d, expected %d", n, size);
		return false;
	}

	return true;
}

static void set_rts(struct cgpu_info *cgpu)
{
	int ret;
  if ((ret = libusb_control_transfer(cgpu->usbdev->handle, FTDI_TYPE_OUT, FTDI_REQUEST_MODEM, SIO_SET_RTS_LOW, usb_interface(cgpu), NULL, 0, 1000)) !=LIBUSB_SUCCESS)
	{
		applog(LOG_ERR, "[Clam] set rts error %d", ret);
	}
}

static void clear_rts(struct cgpu_info *cgpu)
{
	int ret;
  if ((ret = libusb_control_transfer(cgpu->usbdev->handle, FTDI_TYPE_OUT, FTDI_REQUEST_MODEM, SIO_SET_RTS_HIGH, usb_interface(cgpu), NULL, 0, 1000)) !=LIBUSB_SUCCESS)
	{
		applog(LOG_ERR, "[Clam] clear rts error %d", ret);
	}
}

static bool clear_dtr(struct cgpu_info *cgpu)
{
	int ret;
  if ((ret = libusb_control_transfer(cgpu->usbdev->handle, FTDI_TYPE_OUT, FTDI_REQUEST_MODEM, SIO_SET_DTR_HIGH, usb_interface(cgpu), NULL, 0, 1000)) !=LIBUSB_SUCCESS)
	{
		applog(LOG_ERR, "[Clam] clear dtr error %d", ret);
		return false;
	}
  return true;
}
static bool set_dtr(struct cgpu_info *cgpu)
{
	int ret;
  if ((ret = libusb_control_transfer(cgpu->usbdev->handle, FTDI_TYPE_OUT, FTDI_REQUEST_MODEM, SIO_SET_DTR_LOW, usb_interface(cgpu), NULL, 0, 1000)) !=LIBUSB_SUCCESS)
	{
		applog(LOG_ERR, "[Clam] set dtr error %d", ret);
		return false;
	}
  return true;
}

static bool hard_reset(struct cgpu_info *cgpu)
{
	clear_dtr(cgpu);
	set_dtr(cgpu);
  return true;
}

static bool write_work(struct cgpu_info *cgpu, unsigned char *midstate, unsigned char *data)
{
	char *hex_midstate;
	char *hex_data;
	hex_midstate = bin2hex(midstate, 32);
	hex_data = bin2hex(data, 12);
	applog(LOG_DEBUG, "[Clam] send work : midstate[%s], data[%s]", hex_midstate, hex_data);
	free(hex_midstate);
	free(hex_data);

	//reverse bye order according to chip spec
	unsigned char tm[32], td[12];
	unsigned char buf[32+12];
	int i;
	for (i=0;i<32;i++)
		buf[i] = midstate[31-i];
	for (i=0;i<12;i++)
		buf[32 + i] = data[11-i];

	//clear_rts(cgpu);

	if (unlikely(!clam_write(cgpu, buf, sizeof(buf))))
	{
		applog(LOG_ERR, "[Clam] work write error");
		return false;
	}
	return true;
}

static bool write_register(struct cgpu_info *cgpu, uint8_t chip_id, uint8_t address, uint8_t value)
{
	applog(LOG_DEBUG, "[Clam] write register [%02x]/[%02x] : %02x", chip_id, address, value);
	set_rts(cgpu);
	//write chip id
	if (unlikely(!clam_write(cgpu, &chip_id, 1)))
	{
		clear_rts(cgpu);
		applog(LOG_ERR, "[Clam] write chip_id [%02x] failed", chip_id);
		return false;
	}

	address = (address << 1) | 0x01;	//write mode;

	if (unlikely(!clam_write(cgpu, &address, 1)))
	{
		clear_rts(cgpu);
		applog(LOG_ERR, "[Clam] write register address %02x of chip %02x failed", address, chip_id);
		return false;
	}
	if (unlikely(!clam_write(cgpu, &value, 1)))
	{
		clear_rts(cgpu);
		applog(LOG_ERR, "[Clam] write register value %02x to %02x/%02x failed", value, chip_id, address);
		return false;
	}
//	tcdrain(fd);
	clear_rts(cgpu);
	return true;
}


static bool request_register(struct cgpu_info *cgpu, uint8_t chip_id, uint8_t address)
{
	//register read access sequence
	//extracted this function from read_register to support chip detect operation

	//write chip id
	if (unlikely(!clam_write(cgpu, &chip_id, 1)))
	{
		applog(LOG_ERR, "[Clam] write chip_id [%02x] failed", chip_id);
		return false;
	}

	address <<= 1;

	if (unlikely(!clam_write(cgpu, &address, 1)))
	{
		applog(LOG_ERR, "[Clam] write register address %02x of chip %02x failed", address, chip_id);
		return false;
	}
	return true;
}

static bool read_register(struct cgpu_info *cgpu, uint8_t chip_id, uint8_t address, uint8_t *result)
{
	uint32_t data;

	applog(LOG_DEBUG, "read register [%02x]/[%02x]", chip_id, address);
	set_rts(cgpu);
	if (unlikely(!request_register(cgpu, chip_id, address)))
	{
		clear_rts(cgpu);
		return false;
	}
	clear_rts(cgpu);

	if (unlikely(!clam_read(cgpu, reg_ms_timeout, &data)))
	{
		applog(LOG_ERR, "read register value failed");
		return false;
	}
	applog(LOG_DEBUG, "register data [%08x]", data);
	*result = (data >> ((3 - (address & 0x3)) * 8)) & 0xff;
	return true;
}

static bool set_pll_simple(struct cgpu_info *cgpu, const int chip_id, int frequency)
{
	//default method to modify only M value
	uint8_t od = CLAM_PLL_DEFAULT_OD;
	uint8_t n = CLAM_PLL_DEFAULT_N;

	od = 2;
	n = 1;
	if (frequency > 360)
		od = 1;

	uint8_t m = (frequency << od) / (CLAM_PLL_DEFAULT_XIN / n);
	applog(LOG_DEBUG, "[Clam] set PLL M/N/OD value to %02x/%02x/%02x", m, n, od);
	if (unlikely(!write_register(cgpu, chip_id, CLAM_REG_PLL1, m)))
		return false;
	uint8_t pll2 = (n << 4) | ((od & 0x3) << 2);
	if (unlikely(!write_register(cgpu, chip_id, CLAM_REG_PLL2, pll2)))
		return false;
	//PLL should be reset after registers have been set
	if (unlikely(!write_register(cgpu, chip_id, CLAM_REG_PLL2, pll2 | 0x01)))
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

	uint32_t nonce;
	if (unlikely(!clam_read(cgpu, test_ms_timeout, &nonce)))
	{
		applog(LOG_ERR, "[Clam] read test nonce failed");
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

static bool set_core_range(struct cgpu_info *cgpu, uint8_t chip_id, int core, uint16_t low, uint16_t high)
{
	applog(LOG_DEBUG, "[%02x]-[%02x] scan range:%04x0000 - %04xffff", chip_id, core, low, high);
	if (!write_register(cgpu, chip_id, CLAM_REG_CORE0_RANGE_LOW + core * 4, low & 0xff) ||
		!write_register(cgpu, chip_id, CLAM_REG_CORE0_RANGE_LOW + core * 4 + 1, low >> 8) ||
		!write_register(cgpu, chip_id, CLAM_REG_CORE0_RANGE_LOW + core * 4 + 2, high & 0xff) ||
		!write_register(cgpu, chip_id, CLAM_REG_CORE0_RANGE_LOW + core * 4 + 3, high >> 8))
	{
		applog(LOG_ERR, "[%02x]-[%02x] set scan range failed", chip_id, core);
		return false;
	}

	if (unlikely(!write_register(cgpu, chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_RANGE_INITIAL)))
	{
		return false;
	}
	return true;
}

static bool set_core_mask(struct cgpu_info *cgpu, uint8_t chip_id, uint8_t core_mask)
{
	applog(LOG_DEBUG, "[%02x] set core mask %02x", chip_id, core_mask);
	return write_register(cgpu, chip_id, CLAM_REG_CORE_MASK, core_mask);
}

static bool assign_cores(struct cgpu_info *cgpu, struct clam_info *info)
{
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
				if (unlikely(!set_core_range(cgpu, i, j, range, range + range_width - 1)))
					return false;

				range += range_width;
			}
		}
	}
	return true;
}

static bool reset_all(struct cgpu_info *cgpu)
{
	/*
	struct timeval tv_now, tv_start;
	cgtime(&tv_start);
*/	
	if (unlikely(!write_register(cgpu, CLAM_CHIP_ID_ALL, CLAM_REG_GENERAL_CONTROL, CLAM_GC_RESET)))
	{
		applog(LOG_ERR, "[Clam] reset all failed");
		return false;
	}
	struct clam_info *info = cgpu->device_data;

	info->has_queued_work = false;
/*	cgtime(&tv_now);
	int passed = us_tdiff(&tv_now, &tv_start);
	applog(LOG_ERR, "passed:%d", passed);
*/
	/*
	if (unlikely(!write_register(cgpu, CLAM_CHIP_ID_ALL, CLAM_REG_GENERAL_CONTROL, CLAM_GC_RANGE_INITIAL)))
	{
		applog(LOG_ERR, "[Clam] init range all failed");
		return false;
	}

	info->last_nonce = 0;
*/
	//re-set bypass
//	int i;
//	for (i=0;i<CLAM_MAX_CHIP_COUNT;i++)
//	{
//		if (info->chip_bypass[i])
//		{
//			applog(LOG_ERR, "[Clam] bypass chip 0x%02x", i);
//			if (unlikely(!write_register(info->fd, i, CLAM_REG_GENERAL_CONTROL, CLAM_GC_CHIP_BYPASS)))
//				return false;
//		}
//	}

	return true;
}

static bool detect_cores(struct cgpu_info *cgpu, struct clam_info *info)
{
	//assume the max number of chips, reset all chips and read chip_id, wait response to detect chips
	int i;

	//loop all the chips as independent and reset all

	if (unlikely(!write_register(cgpu, CLAM_CHIP_ID_ALL, CLAM_REG_GENERAL_CONTROL,
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
	for (i = opt_clam_chip_start; i < opt_clam_chip_end;i++)
	{
		uint8_t chip_id;
		if (read_register(cgpu, i, CLAM_REG_CHIP_ID, &chip_id))
		{
			chip_ids[info->chip_count++] = chip_id;
			applog(LOG_NOTICE, "[Clam] Chip 0x%02x found!", chip_id);
		}
	}

	applog(LOG_NOTICE, "[Clam] %d chips detected", info->chip_count);
	if (info->chip_count == 0)
		return false;

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

		if (unlikely(!write_register(cgpu, chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_LOOP_DOWN_S|CLAM_GC_LOOP_UP_S|CLAM_GC_LOOP_UP_M))||
			unlikely(!read_register(cgpu, chip_id, CLAM_REG_CHIP_ID, &result))||
			unlikely(chip_id != result)||
			unlikely(!write_register(cgpu, chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_LOOP_DOWN_S|CLAM_GC_LOOP_UP_S|CLAM_GC_LOOP_UP_M|CLAM_GC_LOOP_DOWN_M)))	//restore loops
		{
			applog(LOG_ERR, "[Clam] M line test failed. chip 0x%02x -> 0x%02x", chip_id, next_chip_id);
			return false;
		}

		if (unlikely(!write_register(cgpu, next_chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_LOOP_UP_M|CLAM_GC_LOOP_DOWN_M|CLAM_GC_LOOP_DOWN_S))||
			unlikely(!read_register(cgpu, next_chip_id, CLAM_REG_CHIP_ID, &result))||
			unlikely(next_chip_id != result)||
			unlikely(!write_register(cgpu, next_chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_LOOP_DOWN_S|CLAM_GC_LOOP_UP_S|CLAM_GC_LOOP_UP_M|CLAM_GC_LOOP_DOWN_M)))	//restore loops
		{
			applog(LOG_ERR, "[Clam] S line test failed. chip 0x%02x -> 0x%02x", chip_id, next_chip_id);
			return false;
		}

	}
	applog(LOG_NOTICE, "[Clam] Line test passed.");

	if (!unlikely(write_register(cgpu, CLAM_CHIP_ID_ALL, CLAM_REG_GENERAL_CONTROL, CLAM_GC_RESET)))
		return false;

	//detect functional cores
	//first disable all chips' cores
	if (unlikely(!set_core_mask(cgpu, CLAM_CHIP_ID_ALL, 0)))
		return false;

	//then enable every single core to try golden nonce
	for (i=0; i< info->chip_count; i++)
	{
		uint8_t chip_id = chip_ids[i];

		uint8_t mask = 1;
		int j;
		for (j=0;j<opt_clam_core_limit;j++)
		{
			//send test work to try the hash core
			//core must be enabled BEFORE core range being set
			if (unlikely(!write_register(cgpu, chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_RESET))||	//reset first
				unlikely(!set_core_mask(cgpu, chip_id, mask))||			//enable the core only
				unlikely(!set_core_range(cgpu, chip_id, j, 0x0000, 0xffff))||	//set the full range
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
			mask <<=1;

		}

		//disable cores
		if (unlikely(!set_core_mask(cgpu, chip_id, 0)))
			return false;
	}

	applog(LOG_NOTICE, "[Clam] %d functional cores found.", info->core_count);
/*	applog(LOG_NOTICE, "[Clam] \t0\t1\t2\t3");
	for (i=0; i< info->chip_count; i++)
	{
		int j;
		for (j=0;j<8,j++)
		{
			char buf1[8 + 1];
			
			applog(LOG_NOTICE, "[Clam] %d\t");
		}
	}
*/

	//enable all functional cores
	for (i=0;i<info->chip_count;i++)
	{
		int chip_id = chip_ids[i];

		applog(LOG_DEBUG, "[Clam] chip 0x%02x map:0x%02x", chip_id, info->core_map[chip_id]);
		if (unlikely(!set_core_mask(cgpu, chip_id, info->core_map[chip_id])))
			return false;

		if (!info->core_map[chip_id])
		{
			//bypass
			if (unlikely(!write_register(cgpu, chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_CHIP_BYPASS)))
				return false;
			info->chip_bypass[i] = true;
		}
	}

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
		goto out;
		
	usb_buffer_enable(cgpu);
	usb_ftdi_set_latency(cgpu);

	int ret;
  if ((ret = libusb_control_transfer(cgpu->usbdev->handle, FTDI_TYPE_OUT, FTDI_REQUEST_BAUD, 0x1a, 0,// usb_interface(cgpu),
                              NULL, 0, 1000)) !=LIBUSB_SUCCESS)
	{
		applog(LOG_ERR, "[Clam] set baud error %d", ret);
	}
	hard_reset(cgpu);

	clear_rts(cgpu);

	uint8_t n;

	if (unlikely(!set_pll_simple(cgpu, CLAM_CHIP_ID_ALL, opt_clam_clock)))
		return false;

	if (unlikely(!detect_cores(cgpu, info)))
		goto failed;

	if (!info -> core_count)
	{
		applog(LOG_ERR, "[Clam] no functional core found");
		goto failed;
	}
	if (opt_clam_test)
		goto failed;

	//set ranges
	if (unlikely(!assign_cores(cgpu, info)))
		goto failed;

	reset_all(cgpu);
	
	flush_buffer(cgpu);

	return true;

	failed:;
	usb_uninit(cgpu);
	if (info)
		free(info);

	out:;
	if (cgpu)
		usb_free_cgpu(cgpu);
	return false;
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

	int ms_timeout = 4294967 / info->core_count / opt_clam_clock ;	//max timeout

			//clam debug
		if (info->work_count_total % 50 == 0)
		{
		//	applog(LOG_NOTICE, "[Clam Debug] ============timeout ratio: %.4f%%(total),  %.4f%%(20)", info->timeout_total * 100.0 / info->work_count_total, info->period_timeout * 100.0 / 50);
			info->period_timeout = 0;
		}

	//calc the actual timeout time
	struct timeval tv_now, tv_now2;
	cgtime(&tv_now);
	
	//applog(LOG_ERR, "[Clam Debug] timeout %d", ms_timeout);
	
	uint32_t nonce;
	if (unlikely(!clam_read(cgpu, ms_timeout, &nonce)))
	{
		applog(LOG_DEBUG, "[Clam] read nonce timeout or error, reset all");

		thr->work_restart = true;
		
		info->work_count_total++;
		info->timeout_total++;
		info->period_timeout++;

		return 0x100000000;
	}
	else
	{
//		applog(LOG_DEBUG, "[Clam] nonce found [%08x], for midstate[%08x]", nonce, *((uint32_t *)info->current_work->midstate));
		info->work_count_total++;
		
		//try submit
		int i;
		bool found = false;
		for (i = 0; i< info->array_top - 1; i++)
		{
			if (test_nonce(info->work_array[i], nonce))
			{
				if (i > 5)
					applog(LOG_ERR, "[Clam] Submit for work %d, %08x", i, nonce);
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
			applog(LOG_ERR, "[Clam] HW error, reset all, %08x, %d", nonce, info->work_count_total);
			thr->work_restart = true;
		}

		//estimate the hashes
		int range = 0x10000/info->core_count;
		int64_t hashes;
		hashes = (nonce >> 16) % range - (info->last_nonce >> 16) % range;
		hashes = (hashes + range) % range;
		hashes <<= 16;
		hashes = hashes + (nonce & 0xffff) - (info->last_nonce & 0xffff);
		hashes *= info->core_count;

		info->last_nonce = nonce;

		return hashes;
	}
}

static bool clam_queue_full(struct cgpu_info *cgpu)
{
	//send work to the device
	struct clam_info *info = cgpu->device_data;
	struct work *work = get_queued(cgpu);

	if (!unlikely(write_work(cgpu, work->midstate, work->data + 64)))
	{
		applog(LOG_ERR, "[Clam] send work error, discarding current work.");
		work_completed(cgpu, work);
		return false;
	}
	
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

	if (!info->has_queued_work)
	{
		info->has_queued_work = true;
		return false;
	}
	return true;
}

static void clam_flush_work(struct cgpu_info *cgpu)
{
	applog(LOG_DEBUG, "[Clam] flush work and reset all");
	reset_all(cgpu);
}

static void clam_thread_shutdown(struct thr_info *thr)
{
	struct cgpu_info *cgpu = thr->cgpu;
	clear_dtr(cgpu);
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

	.flush_work = clam_flush_work,
	.thread_shutdown = clam_thread_shutdown
};
