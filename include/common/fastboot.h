/*
 * Copyright (c) 2010, The Android Open Source Project.
 *
  * Copyright (C) 2012, Texas Instruments, Inc.
 * Texas Instruments, <www.ti.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Neither the name of The Android Open Source Project nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
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
 *
 */

#ifndef _FASTBOOT_H_
#define _FASTBOOT_H_
#include <common/usbboot_common.h>
#include <common/sparse_format.h>

/* Write the file as a series of variable/value pairs
   using the setenv and saveenv commands */
#define FASTBOOT_PTENTRY_FLAGS_WRITE_ENV	0x00000400
/* To support the Android-style naming of flash */
#define MAX_PTN 16

/* Android-style flash naming */
typedef struct fastboot_ptentry fastboot_ptentry;


/* flash partitions are defined in terms of blocks
** (flash erase units)
*/
struct fastboot_ptentry {
	/* The logical name for this partition, null terminated */
	char name[16];
	/* The start wrt the nand part, must be multiple of nand block size */
	unsigned int start;
	/* The length of the partition, must be multiple of nand block size */
	u64 length;
	/* Controls the details of how operations are done on the partition
	See the FASTBOOT_PTENTRY_FLAGS_*'s defined below */
	unsigned int flags;
};

struct fastboot_data {
	struct board_specific_functions *board_ops;
	struct proc_specific_functions *proc_ops;
	struct storage_specific_functions *storage_ops;
	struct fastboot_ptentry *e;

	struct fastboot_ptentry ptable[MAX_PTN];

	char *dsize;
	u32 getsize;
	u32 sector;
	sparse_header_t *sparse_header;
};

#if defined CONFIG_FASTBOOT

void do_fastboot(struct bootloader_ops *board_funcs);
char *get_serial_number(void);
void fastboot_flash_reset_ptn(void);
void fastboot_flash_add_ptn(fastboot_ptentry *ptn, int count);
unsigned int fastboot_flash_get_ptn_count(void);
fastboot_ptentry *fastboot_flash_find_ptn(const char *name);
char *get_ptn_size(struct fastboot_data *fb_data, char *buf, const char *ptn) ;

#else

static inline void do_fastboot(struct bootloader_ops *board_funcs) { return; };
static inline char *get_serial_number(void) { return 0; };
static inline void fastboot_flash_reset_ptn(void) { return; };
static inline void fastboot_flash_add_ptn(fastboot_ptentry *ptn, int count) { return; };
static inline unsigned int fastboot_flash_get_ptn_count(void) { return 0; };
static inline fastboot_ptentry *fastboot_flash_find_ptn(const char *name) { return NULL; };

static inline char *get_ptn_size(struct fastboot_data *fb_data, char *buf,
					const char *ptn) { return 0; };

#endif /* CONFIG_FASTBOOT */

int do_gpt_format(struct fastboot_data *fb_data);
int load_ptbl(struct storage_specific_functions *fb_data, u8 silent);


#endif /* FASTBOOT_H */
