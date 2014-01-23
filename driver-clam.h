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

#define WORK_ARRAY_SIZE 80
#define CONTROLLER_QUEUE_TARGET_SIZE 20

#define CLAM_TYPE_IN (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN)
#define CLAM_TYPE_OUT (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT)

// out
#define CLAM_REQUEST_RESET_CONTROLLER				0x00
#define CLAM_REQUEST_RESET_CHANNEL					0x01	//index - channel_id
#define CLAM_REQUEST_PLL							0x02	//value[7:0] - m, value[15:8] - n, value[23:16] - od, index - channel_id
#define CLAM_REQUEST_CLOCK							0x03	//value - frequency in MHz, index - channel_id
#define CLAM_REQUEST_IDENTIFY						0x04
#define CLAM_REQUEST_FLUSH_WORK						0x05
#define CLAM_REQUEST_FLUSH_RESULT					0x06

//in
#define CLAM_REQUEST_VERSION						0x10
#define CLAM_REQUEST_CHANNELS						0x11
#define CLAM_REQUEST_CORES							0x12	//index for channel_id
#define CLAM_REQUEST_WORK_QUEUE						0x13

#define CLAM_MAX_CHANNELS 40

struct channel_info
{
	int chip_count;
	int core_count;
	uint8_t core_map[CLAM_MAX_CHIP_COUNT];
	uint32_t last_nonce;
	int cont_timeout;
	int cont_hw;
};

struct clam_info
{
	struct work *work_array[WORK_ARRAY_SIZE];
	int array_top;

	uint32_t channel_count;
	struct channel_info channels[CLAM_MAX_CHANNELS];
	
	int controller_queue_size;
};

#define CLAM_RESULT_TYPE_NONCE		0x00
#define CLAM_RESULT_TYPE_TIMEOUT	0xff
struct clam_result
{
	uint8_t channel_id;
	uint8_t type;
	uint16_t reserved;
	uint32_t result;
};

char *set_clam_clock(char *arg);
extern int opt_clam_core_limit;
extern int opt_clam_chip_start;
extern int opt_clam_chip_end;
extern bool opt_clam_test_only;



#endif /* USE_CLAM */
#endif	/* CLAM_H */
