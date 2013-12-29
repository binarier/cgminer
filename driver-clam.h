/*
 * Copyright 2013 Con Kolivas <kernel@kolivas.org>
 * Copyright 2013 Li Chenjun <binarier@clambtc.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#ifndef CLAM_H
#define CLAM_H

#ifdef USE_CLAM

#define CLAM_MAX_CHIP_COUNT 64
#define CLAM_MAX_CORE_COUNT 8

#define CLAM_PLL_DEFAULT_M		0xa0
#define CLAM_PLL_DEFAULT_N		0x3
#define CLAM_PLL_DEFAULT_OD		0x2
#define CLAM_PLL_DEFAULT_XIN	24		//24MHz crystal oscillator
#define CLAM_DEFAULT_CLOCK		((CLAM_PLL_DEFAULT_XIN * CLAM_PLL_DEFAULT_M / CLAM_PLL_DEFAULT_N) >> CLAM_PLL_DEFAULT_OD)
#define CLAM_MAX_CLOCK			750
//#define CLAM_MIN_CLOCK			250
#define CLAM_MIN_CLOCK			50		//FIXME FPGA verification only

#define CLAM_CHIP_ID_ALL				0xff		//to all chips

#define CLAM_REG_GENERAL_CONTROL		0x00
#define CLAM_REG_PLL1					0x01		//M register
#define CLAM_REG_PLL2					0x02		//N register and OD and reset
#define CLAM_REG_CORE_MASK				0x14
#define CLAM_REG_CHIP_ID				0x15
//range register addresses
#define CLAM_REG_CORE0_RANGE_LOW		0x20

//REG_GENERAL_CONTROL
#define CLAM_GC_RESET			0x01		//software reset - 1: reset, this bit will be self cleared
#define CLAM_GC_RANGE_INITIAL	0x02
#define CLAM_GC_CHIP_BYPASS		0x04
#define CLAM_GC_LOOP_UP_M		0x10		//Up loop for UP_MREQ and UP_MACK
#define CLAM_GC_LOOP_UP_S		0x20		//Up loop for UP_SREQ and UP_SACK
#define CLAM_GC_LOOP_DOWN_M		0x40		//Down loop for DN_MREQ and DN_MACK
#define CLAM_GC_LOOP_DOWN_S		0x80		//Down loop for DN_SREQ and DN_SACK

#define WORK_ARRAY_SIZE 20

#ifdef WIN32
#error "Windows is not supported by Clam driver yet."
#endif

struct clam_info
{
	int chip_count;
	int core_count;
	unsigned char core_map[CLAM_MAX_CHIP_COUNT];
	bool chip_bypass[CLAM_MAX_CHIP_COUNT];
	int fd;
	uint32_t last_nonce;
	struct work *work_array[WORK_ARRAY_SIZE];
	bool has_queued_work;
	int array_top;

	//track last work finish time
	struct timeval tv_work_start;
};

char *set_clam_clock(char *arg);
extern int opt_clam_core_limit;
extern int opt_clam_chip_start;
extern int opt_clam_chip_end;

#endif /* USE_CLAM */
#endif	/* CLAM_H */
