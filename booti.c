/*
 * Copyright (C) 2011 The Android Open Source Project
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

#include <aboot/aboot.h>
#include <aboot/bootimg.h>
#include <aboot/common.h>
#include <libc/string.h>
#include <common/omap_rom.h>
#include <common/fastboot.h>
#include <common/boot_settings.h>
#include "config.h"
#include "version.h"

/* Section for Android bootimage format support
 * Refer:
 * http://android.git.kernel.org/?p=platform/system/core.git;
 * a=blob;f=mkbootimg/bootimg.h
 */

#define ALIGN(n, pagesz)	((n + (pagesz - 1)) & (~(pagesz - 1)))

#ifndef EXTENDED_CMDLINE
#define EXTENDED_CMDLINE	""
#endif

struct mmc mmc;
struct mmc_devicedata *dd;

static u32 setup_atag(boot_img_hdr *hdr, u32 *atag)
{
	u32 *atag_start = atag;
	char *p;
	char *cmdline = (char *)hdr->cmdline;

	*(atag++) = 5;
	*(atag++) = _CORE;
	*(atag++) = 0;
	*(atag++) = 0;
	*(atag++) = 0;

	*(atag++) = 4;
	*(atag++) = _INITRD;
	*(atag++) = hdr->ramdisk_addr;
	*(atag++) = hdr->ramdisk_size;

	*(atag++) = 4;
	*(atag++) = _MEM;
	*(atag++) = 0x80000000; /* memory start */
	*(atag++) = 0x80000000; /* memory size */

	if (!cmdline)
		goto _none;

	for (p = cmdline; *p == ' '; p++)
		;

	if (*p == '\0')
		goto _none;

	u32 size = strlen(p); /* size in bytes */
	*(atag++) = 2 + size/4; /* size text + size of size + size of tag */
	*(atag++) = _CMDLINE;
	strcpy((char *)atag, p);
	atag += size/4;	/* size in u32 */
_none:
	*(atag++) = 0;
	*(atag) = _NONE;

	return atag - atag_start;
}

void boot_settings(boot_img_hdr *hdr, u32 atag)
{
	char temp_cmdline[512] = EXTENDED_CMDLINE;
	char serial_str[64];
	int serial_len;
	u32 atag_size, boot_len;
	char aboot_version_string[64];
	char boot_str[64];

	serial_len = sprintf(serial_str, " androidboot.serialno=%s",
		get_serial_number());

	strcpy((char *)hdr->cmdline, temp_cmdline);
	if (sizeof(hdr->cmdline) >= (serial_len +
		strlen((const char *)hdr->cmdline) + 1))
		strcat((char *)hdr->cmdline, serial_str);

	strcpy(aboot_version_string, ABOOT_VERSION);
	boot_len = sprintf(boot_str, " androidboot.bootloader=%s",
		aboot_version_string);

	if (sizeof(hdr->cmdline) >= (boot_len +
			strlen((const char *)hdr->cmdline) + 1))
		strcat((char *)hdr->cmdline, boot_str);

	atag_size = setup_atag(hdr, (u32 *)atag);

	return;
}

void bootimg_print_image_hdr(boot_img_hdr *hdr)
{
#ifdef DEBUG
	int i;
	printf("printing bootimg header ...\n");
	printf("   Image magic:   %s\n", hdr->magic);

	printf("   kernel_size:   0x%x\n", hdr->kernel_size);
	printf("   kernel_addr:   0x%x\n", hdr->kernel_addr);

	printf("   rdisk_size:   0x%x\n", hdr->ramdisk_size);
	printf("   rdisk_addr:   0x%x\n", hdr->ramdisk_addr);

	printf("   second_size:   0x%x\n", hdr->second_size);
	printf("   second_addr:   0x%x\n", hdr->second_addr);

	printf("   tags_addr:   0x%x\n", hdr->tags_addr);
	printf("   page_size:   0x%x\n", hdr->page_size);

	printf("   name:      %s\n", hdr->name);
	printf("   cmdline:   %s%x\n", hdr->cmdline);

	for (i = 0; i < 8; i++)
		printf("   id[%d]:   0x%x\n", i, hdr->id[i]);
#endif
	return;
}

int do_booti(char *info)
{
	boot_img_hdr *hdr;
	u32 addr; u32 sector1; u32 sector2;
	char *ptn = "boot";
	int boot_from_mmc = 0;
	int sector_count = 0; int num_sectors = 0;
	int ret = 0;

	if (!(strcmp(info, "mmc")))
		boot_from_mmc = 1;

	addr = CONFIG_ADDR_DOWNLOAD;

	hdr = (boot_img_hdr *) addr;

	if (boot_from_mmc) {

		struct fastboot_ptentry *pte;

		/* Init the MMC and load the partition table */
		ret = board_mmc_init();
		if (ret != 0) {
			printf("board_mmc_init() failed\n");
			goto fail;
		}

		pte = fastboot_flash_find_ptn(ptn);
		if (!pte) {
			printf("booti: cannot find '%s' partition\n", ptn);
			do_fastboot();
		}

		ret = mmc_read(&mmc, pte->start, sizeof(boot_img_hdr),
							(void *) hdr);
		if (ret != 0) {
			printf("booti: failed to read bootimg header\n");
			goto fail;
		} else
			bootimg_print_image_hdr(hdr);

		ret = memcmp(hdr->magic, "ANDROID!", 8);
		if (ret != 0) {
			printf("booti: bad boot image magic\n");
			goto fail;
		}

		sector1 = pte->start + (hdr->page_size / 512);

		sector2 = sector1 +
			ALIGN(hdr->kernel_size, hdr->page_size) / 512;

		num_sectors = CEIL(hdr->kernel_size, 512);
		if (num_sectors > (hdr->kernel_size / 512)) {
			/* do nothing */
		} else
			num_sectors = (hdr->kernel_size / 512);

		printf("Reading kernel from start sector %d and reading %d "
			"number of sectors %d\n", sector1, num_sectors);

		for (sector_count = 0; sector_count < num_sectors;
							sector_count++) {
			ret = mmc_read(&mmc, sector1+sector_count, 1,
			(void *) hdr->kernel_addr + (sector_count*512));
			if (ret != 0) {
				printf("mmc read failed\n");
				goto fail;
			}
		}
#ifdef DEBUG
		printf("Done reading kernel from mmc\n");
#endif
		num_sectors = CEIL(hdr->ramdisk_size, 512);
		if (num_sectors > (hdr->ramdisk_size / 512)) {
			/* do nothing */
		} else
			num_sectors = (hdr->ramdisk_size / 512);

		printf("Reading ramdisk from start sector %d and reading %d "
			"number of sectors %d\n", sector2, num_sectors);

		for (sector_count = 0; sector_count < num_sectors;
							sector_count++) {
			ret = mmc_read(&mmc, sector2+sector_count, 1,
			(void *) hdr->ramdisk_addr + (sector_count*512));
			if (ret != 0) {
				printf("mmc read failed\n");
				goto fail;
			}
		}
#ifdef DEBUG
		printf("Done reading ramdisk from mmc\n");
#endif
	} else {
		u32 kaddr, raddr;

		printf("user wants to boot an image downloaded using "
							"fastboot\n");

		ret = memcmp(hdr->magic, "ANDROID!", 8);
		if (ret != 0) {
			printf("booti: bad boot image magic\n");
			goto fail;
		}

		bootimg_print_image_hdr(hdr);

		kaddr = addr + hdr->page_size;

		raddr = kaddr + ALIGN(hdr->kernel_size, hdr->page_size);

		memmove((void *) hdr->kernel_addr, (void *)kaddr,
							hdr->kernel_size);
		memmove((void *) hdr->ramdisk_addr, (void *)raddr,
							hdr->ramdisk_size);
	}

	printf("kernel   @ %08x (%d)\n", hdr->kernel_addr, hdr->kernel_size);
	printf("ramdisk  @ %08x (%d)\n", hdr->ramdisk_addr, hdr->ramdisk_size);

#if defined CONFIG_OMAP4_ANDROID_CMD_LINE || \
	defined CONFIG_OMAP5_ANDROID_CMD_LINE
	boot_settings(&hdr[0], ATAGS_ARGS);
#endif

	void (*theKernel)(int zero, int arch, uint params);
	theKernel = (void (*)(int, int, uint))(hdr->kernel_addr);

	printf("\nbooting kernel...\n");
	theKernel(0, cfg_machine_type, ATAGS_ARGS);

fail:
	printf("do_booti failed, stay here\n");
	while (1)
		;
}
