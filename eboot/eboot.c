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

#include <aboot.h>
#include <common.h>
#include <io.h>

#include <common_proc.h>
#include <fastboot.h>
#include <omap_rom.h>
#include <usbboot_common.h>
#include <alloc.h>
#include <user_params.h>

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

#ifdef TWO_STAGE_OMAPBOOT
static int do_sboot(struct bootloader_ops *boot_ops, int bootdevice)
{
	int ret = 0;
	struct fastboot_ptentry *pte;
	void (*Sboot)(u32 bootops_addr, int bootdevice, void *addr);

	u32 bootops_addr = (u32) boot_ops;

	int sector_sz = boot_ops->storage_ops->get_sector_size();

	int num_sectors = SECOND_STAGE_OBJECT_SIZE/sector_sz;

	u32 addr = CONFIG_ADDR_DOWNLOAD;

	/* look for sboot in bootloader ptn, load and jump */
	ret = load_ptbl(boot_ops->storage_ops, 1);
	if (ret != 0) {
		printf("unable to load the partition table\n");
		return ret;
	}

	pte = fastboot_flash_find_ptn("bootloader");
	if (!pte) {
		printf("eboot: cannot find '%s' partition\n");
		return -1;
	}

	ret = boot_ops->storage_ops->read(pte->start, num_sectors,
								(void *) addr);
	if (ret != 0) {
		printf("mmc read failed\n");
		return ret;
	}

	Sboot = (void (*)(u32, int, void *))(addr);
	Sboot((u32) bootops_addr, (int) bootdevice, (void *) addr);

	return -1;
}
#endif

void eboot(unsigned *info)
{
	int ret = 0;
	unsigned bootdevice = -1;
	struct usb usb;
	struct bootloader_ops *boot_ops;

	if (info)
		bootdevice = info[2] & 0xFF;
	else
		goto fail;

	boot_ops = boot_common(bootdevice, &usb);
	if (!boot_ops)
		goto fail;

	if (info)
		bootdevice = info[2] & 0xFF;
	else
		goto fail;

#ifdef TWO_STAGE_OMAPBOOT
	ret = do_sboot(boot_ops, bootdevice);
	if (ret != 0)
		goto fail;
#else
	if (boot_ops->board_ops->board_user_fastboot_request)
		if (boot_ops->board_ops->board_user_fastboot_request())
			goto fastboot;

	do_booti(boot_ops, "storage", NULL, &usb);

fastboot:
	ret = usb_open(&usb);
	if (ret != 0) {
		printf("\nusb_open failed\n");
		goto fail;
	}
	usb_init(&usb);
	do_fastboot(boot_ops, &usb);

#endif

fail:
	printf("boot failed\n");
	while (1)
		;
}
