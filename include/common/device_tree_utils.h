/*
* Copyright (C) 2013 Texas Instruments, Inc.
* All rights reserved.
*
* Author: Ruslan Bilovol <ruslan.bilovol@ti.com>
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

#ifndef __DEVICE_TREE_UTILS_H__
#define __DEVICE_TREE_UTILS_H__

#include <usbboot_common.h>

/*
 * FDT structures and constants as per
 * "Power.org™ Standard for Embedded Power Architecture™
 * Platform Requirements (ePAPR)" Version 1.1 – 08 April 2011
 */
#define FDT_MAGIC		0xd00dfeed
#define FDT_BEGIN_NODE		0x00000001
#define FDT_END_NODE		0x00000002
#define FDT_PROP		0x00000003
#define FDT_NOP			0x00000004
#define FDT_END			0x00000009

struct fdt_header {
	u32 magic;
	u32 totalsize;
	u32 off_dt_struct;
	u32 off_dt_strings;
	u32 off_mem_rsvmap;
	u32 version;
	u32 last_comp_version;
	u32 boot_cpuid_phys;
	u32 size_dt_strings;
	u32 size_dt_struct;
};

struct fdt_reserve_entry {
	u64 address;
	u64 size;
};

struct fdt_property {
	u32 len;
	u32 nameoff;
};

u32 dt_find_and_replace(void *dt_blob, char *property_path, u32 value);

#endif /* __DEVICE_TREE_UTILS_H__ */
