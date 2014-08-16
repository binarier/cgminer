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
int opt_clam_chip_start = 0;
int opt_clam_chip_end = CLAM_MAX_CHIP_COUNT;
bool opt_clam_test = false;

static void clam_detect(void)
{
	//detect completed
	struct cgpu_info *cgpu = calloc(1, sizeof(*cgpu));
	static struct clam_info ci ={0};
	cgpu->drv = &clam_drv;
	cgpu->device_path = "/dev/test";
	cgpu->device_data = &ci;
	cgpu->threads = 1;
	add_cgpu(cgpu);
}

static struct work *current_work = NULL;

static int64_t clam_scanwork(struct thr_info *thr)
{
	struct clam_info *info = thr->cgpu->device_data;

	work_completed(thr->cgpu, current_work);
	current_work = NULL;
	return 1l;
}

static bool clam_queue_full(struct cgpu_info *cgpu)
{
	current_work = get_queued(cgpu);

	return true;
}

static void clam_flush_work(struct cgpu_info *cgpu)
{
}

static void clam_thread_shutdown(struct thr_info *thr)
{
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
