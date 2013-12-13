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
#include "fpgautils.h"
#include "uthash.h"

#include "driver-clam.h"

//#define RASPBERRYPI

#ifdef RASPBERRYPI

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)
#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */


// I/O access
volatile unsigned *gpio;


#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0


int get_3b (int g)
{
   return (gpio[g/10] >> ((g%10)*3)) & 7;
}

void set_3b (int g, int v)
{
   gpio[g/10] = (gpio[g/10] & ~(7 << ((g%10)*3)))  |
                        ((v & 7) << ((g%10) *3));
}

void gpio_set (int g)
{
   gpio[7 + (g/32)] = 1 << (g %32);
}

void gpio_clr (int g)
{
   gpio[10 + (g/32)] = 1 << (g %32);
}

int gpio_get (int g)
{
   return (gpio[0xd + (g/32)] >> (g % 32)) & 1;
}



enum gpio_funcs {GP_INP,  GP_OUT,  GP_ALT5, GP_ALT4,
                   GP_ALT0, GP_ALT1, GP_ALT2, GP_ALT3};


//
// Set up a memory region to access GPIO
//
volatile unsigned int *setup_io()
{
   int  mem_fd;
   char *gpio_mem, *gpio_map;


   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem \n");
      exit (-1);
   }

   /* mmap GPIO */

   // Allocate MAP block
   if ((gpio_mem = malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL) {
      printf("allocation error \n");
      exit (-1);
   }

   // Make sure pointer is on 4K boundary
   if ((unsigned long)gpio_mem % PAGE_SIZE)
     gpio_mem += PAGE_SIZE - ((unsigned long)gpio_mem % PAGE_SIZE);

   // Now map it
   gpio_map = mmap(
      (caddr_t)gpio_mem,
      BLOCK_SIZE,
      PROT_READ|PROT_WRITE,
      MAP_SHARED|MAP_FIXED,
      mem_fd,
      GPIO_BASE
   );

   if ((long)gpio_map < 0) {
      printf("mmap error %ld\n", (long)gpio_map);
      exit (-1);
   }

   close (mem_fd);

   // Always use volatile pointer!
   return (volatile unsigned *)gpio_map;
} // setup_io
#endif


struct device_drv clam_drv;

// used for functional core detection
// adopted from driver-icarus.c
// Block 171874 nonce = (0xa2870100) = 0x000187a2
const char *golden_midstate = "4a548fe471fa3a9a1371144556c3f64d2500b4826008fe4bbf7698c94eba7946";
const char *golden_data = "ce22a72f4f6726141a0b3287";
const uint32_t golden_nonce = 0x000187a2;

static struct timeval reg_read_timeout = {0, 100000};	//0.1s
static struct timeval test_work_timeout = {1, 0};	//0x187a2 should come out very soon

int opt_clam_clock = CLAM_DEFAULT_CLOCK;
int opt_clam_core_limit = CLAM_MAX_CORE_COUNT;
bool opt_clam_use_queue = true;

static bool clam_read(int fd, struct timeval *timeout, uint32_t *result)
{
	uint32_t buf;
	// Initialize file descriptor sets
	fd_set read_fds;
	FD_ZERO(&read_fds);
	FD_SET(fd, &read_fds);

	int c = 0;
	while (c < 4)
	{
		// Wait for input to become ready or until the time out; the first parameter is
		// 1 more than the largest file descriptor in any of the sets
		int rc;

		rc = select(fd + 1, &read_fds, NULL, NULL, timeout);

		if (rc == 1)
		{
			rc = read(fd, ((void *)&buf) + c, 4 - c);
			if (rc == -1)
			{
				applog(LOG_ERR, "read clam error");
				return false;
			}
			c += rc;
		}
		else if (rc == 0)
		{
			//timeout
			applog(LOG_DEBUG, "read timeout ");
			return false;
		}
		else
		{
			//timeout
			applog(LOG_ERR, "read error : %d ", errno);
			return false;
		}
	}

	//change endian
	*result = be32toh(buf);
	return true;
}

static bool clam_write(int fd, const void *data, int size)
{
	int i;
	for (i=0;i<size;i++)
	{
		if (write(fd, data+i, 1)!=1)
			return false;
//		if ((i + 1) % 8 == 0)
			tcdrain(fd);	//used for FPGA verification, can be safely deleted when ASIC is out
	}
	tcdrain(fd);
	return true;
}

static void set_rts(int fd)
{
	tcdrain(fd);
#ifdef RASPBERRYPI
	gpio_set(17);
#else
	int bits;
	ioctl(fd, TIOCMGET, &bits);
	bits &= ~TIOCM_RTS;
	ioctl(fd, TIOCMSET, &bits);
#endif
}

static void clear_rts(int fd)
{
	tcdrain(fd);		//buffer must be empty before rts being cleared
#ifdef RASPBERRYPI
	gpio_clr(17);
#else
	int bits;
	ioctl(fd, TIOCMGET, &bits);
	bits |= TIOCM_RTS;
	ioctl(fd, TIOCMSET, &bits);
#endif
}

static bool write_work(int fd, unsigned char *midstate, unsigned char *data)
{
	char *hex_midstate;
	char *hex_data;
	hex_midstate = bin2hex(midstate, 32);
	hex_data = bin2hex(data, 12);
	applog(LOG_NOTICE, "[Clam] send work : midstate[%s], data[%s]", hex_midstate, hex_data);
	free(hex_midstate);
	free(hex_data);

	//reverse bye order according to chip spec
	unsigned char tm[32], td[12];
	int i;
	for (i=0;i<32;i++)
		tm[i] = midstate[31-i];
	for (i=0;i<12;i++)
		td[i] = data[11-i];
	midstate = tm;
	data = td;

	clear_rts(fd);

	if (unlikely(!clam_write(fd, midstate, 32)))
	{
		applog(LOG_ERR, "[Clam] midstate write error");
		return false;
	}
	if (unlikely(!clam_write(fd, data, 12)))
	{
		applog(LOG_ERR, "data write error");
		return false;
	}
	return true;
}

static bool write_register(int fd, uint8_t chip_id, uint8_t address, uint8_t value)
{
	applog(LOG_DEBUG, "[Clam] write register [%02x]/[%02x] : %02x", chip_id, address, value);
	set_rts(fd);
	//write chip id
	if (unlikely(!clam_write(fd, &chip_id, 1)))
	{
		clear_rts(fd);
		applog(LOG_ERR, "[Clam] write chip_id [%02x] failed", chip_id);
		return false;
	}

	address = (address << 1) | 0x01;	//write mode;

	if (unlikely(!clam_write(fd, &address, 1)))
	{
		clear_rts(fd);
		applog(LOG_ERR, "[Clam] write register address %02x of chip %02x failed", address, chip_id);
		return false;
	}
	if (unlikely(!clam_write(fd, &value, 1)))
	{
		clear_rts(fd);
		applog(LOG_ERR, "[Clam] write register value %02x to %02x/%02x failed", value, chip_id, address);
		return false;
	}
	clear_rts(fd);
	return true;
}

static bool request_register(int fd, uint8_t chip_id, uint8_t address)
{
	//register read access sequence
	//extracted this function from read_register to support chip detect operation

	//write chip id
	if (unlikely(!clam_write(fd, &chip_id, 1)))
	{
		applog(LOG_ERR, "[Clam] write chip_id [%02x] failed", chip_id);
		return false;
	}

	address <<= 1;

	if (unlikely(!clam_write(fd, &address, 1)))
	{
		applog(LOG_ERR, "[Clam] write register address %02x of chip %02x failed", address, chip_id);
		return false;
	}
	return true;
}

static bool read_register(int fd, uint8_t chip_id, uint8_t address, uint8_t *result)
{
	uint32_t data;
	struct timeval timeout;

	memcpy(&timeout, &reg_read_timeout, sizeof(timeout));

	applog(LOG_DEBUG, "read register [%02x]/[%02x]", chip_id, address);
	set_rts(fd);
	if (unlikely(!request_register(fd, chip_id, address)))
	{
		clear_rts(fd);
		return false;
	}
	clear_rts(fd);

	if (unlikely(!clam_read(fd, &timeout, &data)))
	{
		applog(LOG_ERR, "read register value failed");
		return false;
	}
	*result = (data >> ((3 - (address & 0x3)) * 8)) & 0xff;
	return true;
}

static bool set_pll_simple(const int fd, const int chip_id, int frequency)
{
	//default method to modify only M value
	uint8_t m = (frequency << CLAM_PLL_DEFAULT_OD) / (CLAM_PLL_DEFAULT_XIN / CLAM_PLL_DEFAULT_N);
	uint8_t n = CLAM_PLL_DEFAULT_N;
	uint8_t od = CLAM_PLL_DEFAULT_OD;
	applog(LOG_DEBUG, "[Clam] set PLL M/N/OD value to %02x/%02x/%02x", m, n, od);
	if (unlikely(!write_register(fd, chip_id, CLAM_REG_PLL1, m)))
		return false;
	uint8_t pll2 = (n << 4) | ((od & 0x3) << 2);
	if (unlikely(!write_register(fd, chip_id, CLAM_REG_PLL2, pll2)))
		return false;
	//PLL should be reset after registers have been set
	if (unlikely(!write_register(fd, chip_id, CLAM_REG_PLL2, pll2 | 0x01)))
		return false;
	return true;

}

static bool send_test_work(int fd)
{
	unsigned char midstate[32];
	unsigned char data[12];
	hex2bin(midstate, golden_midstate, sizeof(midstate));
	hex2bin(data, golden_data, sizeof(data));

	if (unlikely(!write_work(fd, midstate, data)))
	{
		applog(LOG_ERR, "[Clam] write test work failed");
		return false;
	}

	uint32_t nonce;
	struct timeval timeout;
	memcpy(&timeout, &test_work_timeout, sizeof(timeout));
	if (unlikely(!clam_read(fd, &timeout, &nonce)))
	{
		applog(LOG_ERR, "[Clam] read test nonce failed");
		return false;
	}
	if (nonce != golden_nonce)
	{
		applog(LOG_ERR, "[Clam] returned nonce [%08x] do not mathch the golden nonce [%08x]", nonce, golden_nonce);
		return false;
	}
	else
	{
		applog(LOG_NOTICE, "[Clam] test work OK");
	}

	return true;
}

static bool set_core_range(int fd, uint8_t chip_id, int core, uint16_t low, uint16_t high)
{
	applog(LOG_DEBUG, "[%02x]-[%02x] scan range:%04x0000 - %04xffff", chip_id, core, low, high);
	if (!write_register(fd, chip_id, CLAM_REG_CORE0_RANGE_LOW + core * 4, low & 0xff) ||
		!write_register(fd, chip_id, CLAM_REG_CORE0_RANGE_LOW + core * 4 + 1, low >> 8) ||
		!write_register(fd, chip_id, CLAM_REG_CORE0_RANGE_LOW + core * 4 + 2, high & 0xff) ||
		!write_register(fd, chip_id, CLAM_REG_CORE0_RANGE_LOW + core * 4 + 3, high >> 8))
	{
		applog(LOG_ERR, "[%02x]-[%02x] set scan range failed", chip_id, core);
		return false;
	}

	if (unlikely(!write_register(fd, chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_RANGE_INITIAL)))
	{
		return false;
	}
	return true;
}

static bool set_core_mask(int fd, uint8_t chip_id, uint8_t core_mask)
{
	applog(LOG_DEBUG, "[%02x] set core mask %02x", chip_id, core_mask);
	return write_register(fd, chip_id, CLAM_REG_CORE_MASK, core_mask);
}

static bool assign_cores(struct clam_info *info)
{
	//set ranges
	int fd = info->fd;
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
				if (unlikely(!set_core_range(fd, i, j, range, range + range_width - 1)))
					return false;

				range += range_width;
			}
		}
	}
	return true;
}

static bool reset_all(struct clam_info *info)
{
	if (unlikely(!write_register(info->fd, CLAM_CHIP_ID_ALL, CLAM_REG_GENERAL_CONTROL, CLAM_GC_RESET)))
	{
		applog(LOG_ERR, "[Clam] reset all failed");
		return false;
	}
	if (unlikely(!write_register(info->fd, CLAM_CHIP_ID_ALL, CLAM_REG_GENERAL_CONTROL, CLAM_GC_RANGE_INITIAL)))
	{
		applog(LOG_ERR, "[Clam] init range all failed");
		return false;
	}

	info->last_nonce = 0;

	//re-set bypass
	int i;
	for (i=0;i<CLAM_MAX_CHIP_COUNT;i++)
	{
		if (info->chip_bypass[i])
		{
			applog(LOG_ERR, "[Clam] bypass chip 0x%02x", i);
			if (unlikely(!write_register(info->fd, i, CLAM_REG_GENERAL_CONTROL, CLAM_GC_CHIP_BYPASS)))
				return false;
		}
	}

	return true;
}

static bool detect_cores(struct clam_info *info)
{
	//assume the max number of chips, reset all chips and read chip_id, wait response to detect chips
	int i;
	int fd = info->fd;

	//loop all the chips as independent and reset all

	if (unlikely(!write_register(fd, CLAM_CHIP_ID_ALL, CLAM_REG_GENERAL_CONTROL,
			CLAM_GC_LOOP_DOWN_M|
			CLAM_GC_LOOP_DOWN_S|
			CLAM_GC_LOOP_UP_M|
			CLAM_GC_LOOP_UP_S|
			CLAM_GC_RESET)))		//all loop bits
	{
		applog(LOG_ERR, "[Clam] reset all for detection failed");
		return false;
	}

	for (i = 0;i < CLAM_MAX_CHIP_COUNT; i++)
	{
		//send read chip_id reg command
		set_rts(fd);
		if (unlikely(!request_register(fd, i, CLAM_REG_CHIP_ID)))
		{
			applog(LOG_ERR, "[Clam] send read chip_id command failed [%02x]", i);
			clear_rts(fd);
			return false;
		}
		clear_rts(fd);
	}

	struct timeval tv_timeout = { 0, 100000 };//0.1s
	//read all returned data until timeout
	uint8_t chip_ids[CLAM_MAX_CHIP_COUNT];
	while(42)
	{
		uint32_t result;
		if (!clam_read(fd, &tv_timeout, &result))
			break;
		uint8_t chip_id = (result >> ((3 - (CLAM_REG_CHIP_ID & 0x3)) * 8)) & 0xff;
		chip_ids[info->chip_count++] = chip_id;
		applog(LOG_NOTICE, "[Clam] Chip 0x%02x found!", chip_id);
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

		if (unlikely(!write_register(fd, chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_LOOP_DOWN_S|CLAM_GC_LOOP_UP_S|CLAM_GC_LOOP_UP_M))||
			unlikely(!read_register(fd, chip_id, CLAM_REG_CHIP_ID, &result))||
			unlikely(chip_id != result)||
			unlikely(!write_register(fd, chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_LOOP_DOWN_S|CLAM_GC_LOOP_UP_S|CLAM_GC_LOOP_UP_M|CLAM_GC_LOOP_DOWN_M)))	//restore loops
		{
			applog(LOG_ERR, "[Clam] M line test failed. chip 0x%02x -> 0x%02x", chip_id, next_chip_id);
			return false;
		}

		if (unlikely(!write_register(fd, next_chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_LOOP_UP_M|CLAM_GC_LOOP_DOWN_M|CLAM_GC_LOOP_DOWN_S))||
			unlikely(!read_register(fd, next_chip_id, CLAM_REG_CHIP_ID, &result))||
			unlikely(next_chip_id != result)||
			unlikely(!write_register(fd, next_chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_LOOP_DOWN_S|CLAM_GC_LOOP_UP_S|CLAM_GC_LOOP_UP_M|CLAM_GC_LOOP_DOWN_M)))	//restore loops
		{
			applog(LOG_ERR, "[Clam] S line test failed. chip 0x%02x -> 0x%02x", chip_id, next_chip_id);
			return false;
		}

	}
	applog(LOG_NOTICE, "[Clam] Line test passed.");

	if (!unlikely(write_register(fd, CLAM_CHIP_ID_ALL, CLAM_REG_GENERAL_CONTROL, CLAM_GC_RESET)))
		return false;

	//detect functional cores
	//first disable all chips' cores
	if (unlikely(!set_core_mask(fd, CLAM_CHIP_ID_ALL, 0)))
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
			if (unlikely(!write_register(fd, chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_RESET))||	//reset first
				unlikely(!set_core_mask(fd, chip_id, mask))||			//enable the core only
				unlikely(!set_core_range(fd, chip_id, j, 0x0000, 0xffff))||	//set the full range
				unlikely(!send_test_work(fd)))							//send test golden work
			{
				applog(LOG_ERR, "[Clam] chip [%02x] core [%02x] test failed, ignored", chip_id, j);
			}
			else
			{
				//funtional core found
				applog(LOG_NOTICE, "[Clam] Funtional core found:[%02x]/[%02x]", chip_id, j);
				info->core_count++;
				info->core_map[chip_id] |= mask;
			}
			mask <<=1;

		}

		//disable cores
		if (unlikely(!set_core_mask(fd, chip_id, 0)))
			return false;
	}

	applog(LOG_NOTICE, "[Clam] %d functional cores found.", info->core_count);

	//enable all functional cores
	for (i=0;i<info->chip_count;i++)
	{
		int chip_id = chip_ids[i];

		applog(LOG_DEBUG, "[Clam] chip 0x%02x map:0x%02x", chip_id, info->core_map[chip_id]);
		if (unlikely(!set_core_mask(fd, chip_id, info->core_map[chip_id])))
			return false;

		if (!info->core_map[chip_id])
		{
			//bypass
			if (unlikely(!write_register(fd, chip_id, CLAM_REG_GENERAL_CONTROL, CLAM_GC_CHIP_BYPASS)))
				return false;
			info->chip_bypass[i] = true;
		}
	}

	return true;
}

static bool clam_detect_one(const char *devpath)
{
#ifdef RASPBERRYPI
	gpio = setup_io();
	set_3b(17, GP_OUT);
#endif

	applog(LOG_DEBUG, "[Clam] detecting device on serial %s", devpath);

	struct clam_info *info = calloc(1, sizeof(*info));

	//open serial for detect
	int fd = serial_open(devpath, 115200, 0, true);
	if (unlikely(fd == -1)) {
		applog(LOG_ERR, "[Clam] failed to open %s", devpath);
		goto failed;
	}

	info->fd = fd;
	clear_rts(fd);

	if (opt_clam_clock != CLAM_DEFAULT_CLOCK)
	{
		if (unlikely(!set_pll_simple(fd, CLAM_CHIP_ID_ALL, opt_clam_clock)))
			return false;
	}

	if (unlikely(!detect_cores(info)))
		goto failed;

	if (!info -> core_count)
	{
		applog(LOG_ERR, "[Clam] no functional core found");
		goto failed;
	}

	//set ranges
	if (unlikely(!assign_cores(info)))
		goto failed;

	//detect completed
	struct cgpu_info *cgpu = calloc(1, sizeof(*cgpu));

	cgpu->drv = &clam_drv;
	cgpu->device_path = strdup(devpath);
	cgpu->device_data = info;
	cgpu->threads = 1;
	add_cgpu(cgpu);

	return true;

	failed:;
	close(fd);
	if (info)
		free(info);

	return false;
}

static void clam_detect(void)
{
	serial_detect(&clam_drv, clam_detect_one);
}

static int64_t clam_scanwork(struct thr_info *thr)
{
	struct clam_info *info = thr->cgpu->device_data;
	struct timeval tv_timeout;

	int64_t us_timeout = 0xffffffff / info->core_count / opt_clam_clock ;	//max timeout

	//calc the actual timeout time
	struct timeval tv_now;
	cgtime(&tv_now);
	int64_t passed = (int64_t)us_tdiff(&tv_now, &info->tv_work_start);

	us_timeout -= passed;

	us_to_timeval(&tv_timeout, us_timeout);

	uint32_t nonce;
	if (unlikely(!clam_read(info->fd, &tv_timeout, &nonce)))
	{
		applog(LOG_NOTICE, "[Clam] read nonce timeout or error, reset all");

		thr->work_restart = true;

		return 0x100000000;
	}
	else
	{
		applog(LOG_NOTICE, "[Clam] nonce found [%08x], for midstate[%08x]", nonce, *((uint32_t *)info->current_work->midstate));
		if (!submit_nonce(thr, info->current_work, nonce))
		{
			applog(LOG_ERR, "[Clam] HW error, reset all");
			thr->work_restart = true;
		}

		work_completed(thr->cgpu, info->current_work);
		info->current_work = info->queued_work;
		info->queued_work = NULL;
		if (info->current_work != NULL)
		{
			//queued work
			cgtime(&(info->tv_work_start));
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

	if (!unlikely(write_work(info->fd, work->midstate, work->data + 64)))
	{
		applog(LOG_ERR, "[Clam] send work error, discarding current work.");
		work_completed(cgpu, work);
		return false;
	}

	if (opt_clam_use_queue)
	{
		if (!info->current_work)
		{
			info->current_work = work;
			cgtime(&(info->tv_work_start));
			return false;
		}
		else
		{
			info->queued_work = work;
			return true;
		}
	}
	else
	{
		info->current_work = work;
		return true;
	}
}

static void clam_flush_work(struct cgpu_info *cgpu)
{
	applog(LOG_NOTICE, "[Clam] flush work and reset all");
	//clean all the work on device
	struct clam_info *info = cgpu->device_data;

	reset_all(info);
	tcflush(info->fd, TCIOFLUSH);

	info->last_nonce = 0;

	if (info->queued_work)
	{
		work_completed(cgpu, info->queued_work);
		info->queued_work = NULL;
	}
	if (info->current_work)
	{
		work_completed(cgpu, info->current_work);
		info->current_work = NULL;
	}
}

static void clam_thread_shutdown(struct thr_info *thr)
{
	struct clam_info *info = thr->cgpu->device_data;
	reset_all(info);
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

char *set_clam_noqueue(void)
{
	opt_clam_use_queue = false;
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
