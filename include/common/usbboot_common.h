/*
 * Copyright (C) 2012, Texas Instruments, Inc.
 * Texas Instruments, <www.ti.com>
 *
 * Copyright (C) 2012 The Android Open Source Project
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _USBBOOT_COMMON_H_
#define _USBBOOT_COMMON_H_

#include <aboot/types.h>
#include <version.h>

#define CEIL(a, b) (((a) / (b)) + ((a % b) > 0 ? 1 : 0))

struct partition {
	const char *name;
	u32 size_kb;
};

/* Use these functions to override the
 * default configuration for the processor */
struct proc_specific_functions {
	u32 (*proc_get_api_base)(void);
	char* (*proc_get_serial_num)(void);
	char* (*proc_get_type)(void);
	char* (*proc_get_revision)(void);
	char* (*proc_get_version)(void);
	int (*proc_get_proc_id)(void);
};

/* Use these functions to override the
 * default configuration for the processor */
struct board_specific_functions {
	void (*board_scale_vcores)(void);
	struct partition *(*board_get_part_tbl)(void);
	void (*board_prcm_init)(void);
	void (*board_gpmc_init)(void);
	void (*board_late_init)(void);
	void (*board_mux_init)(void);
	void (*board_ddr_init)(struct proc_specific_functions *proc_ops);
	int (*board_storage_init)(u8 device);
	int (*board_user_fastboot_request)(void);
	u8 (*board_get_flash_slot)(void);
};

struct bootloader_ops {
	struct board_specific_functions *board_ops;
	struct proc_specific_functions *proc_ops;
};

void* init_board_funcs(void);
void* init_processor_id_funcs(void);

unsigned long crc32(unsigned long crc, const unsigned char *buf,
						unsigned int len);

int get_downloadsize_from_string(int count, char *string);

#endif
