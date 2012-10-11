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

static unsigned MSG = 0xaabbccdd;

#ifdef TWO_STAGE_OMAPBOOT

static u32 load_from_usb(u32 addr, unsigned *_len, struct usb *usb)
{
	unsigned len, n, param = 0;
	enable_irqs();

	usb_queue_read(usb, &param, 4);
	usb_write(usb, &MSG, 4);
	n = usb_wait_read(usb);
	if (n)
		return 0;

	if (usb_read(usb, &len, 4))
		return 0;

	usb_write(usb, &MSG, 4);

	if (usb_read(usb, (void *) addr, len))
		return 0;

	*_len = len;

#if DO_MEMORY_TEST_DURING_FIRST_STAGE_IN_IBOOT
	if ((param == USER_RQ_MEMTEST) || (param == USER_RQ_UMEMTEST)) {
		memtest((void *)0x82000000, 8*1024*1024);
		memtest((void *)0xA0208000, 8*1024*1024);
	}
#endif
	return param;
}

static int do_sboot(struct bootloader_ops *boot_ops, int bootdevice)
{
	int ret = 0;
	unsigned len;

	void (*Sboot)(u32 bootops_addr, int bootdevice, void *addr);
	u32 bootops_addr = (u32) boot_ops;

	u32 addr = CONFIG_ADDR_SBOOT;

	ret = load_from_usb(addr, &len, &boot_ops->usb);
	if (ret == 0)
		return -1;

	Sboot = (void (*)(u32, int, void *))(addr);
	Sboot((u32) bootops_addr, (int) (bootdevice), (void *) addr);

	return -1;
}
#endif

void iboot(unsigned *info)
{
	struct bootloader_ops *boot_ops;
	unsigned bootdevice = -1;

	if (info)
		bootdevice = info[2] & 0xFF;
	else
		goto fail;

	boot_ops = boot_common(bootdevice);
	if (!boot_ops)
		goto fail;

#ifndef TWO_STAGE_OMAPBOOT
	usb_write(&boot_ops->usb, &MSG, 4);
	do_fastboot(boot_ops);
#else
	do_sboot(boot_ops, bootdevice);
#endif

fail:
	printf("Boot failed\n");
	while (1)
		;
}
