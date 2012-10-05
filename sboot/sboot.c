/*
* Copyright (C) 2012, Texas Instruments, Inc.
* Texas Instruments, <www.ti.com>
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
#include <usbboot_common.h>
#include <alloc.h>

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

u32 public_rom_base;
static struct bootloader_ops *boot_ops;

int usb_open(struct usb *usb)
{
	boot_ops->usb_ops->omap_usb_open(usb);
	return 1;
}

int usb_read(struct usb *usb, void *data, unsigned len)
{
	boot_ops->usb_ops->omap_usb_read(usb, data, len);
	return 1;
}

int usb_write(struct usb *usb, void *data, unsigned len)
{
	boot_ops->usb_ops->omap_usb_write(usb, data, len);
	return 1;
}

void usb_init(struct usb *usb)
{
	boot_ops->usb_ops->omap_usb_init(usb);
}

void usb_close(struct usb *usb)
{
	boot_ops->usb_ops->omap_usb_close(usb);
}

#ifdef BOARD_PROCESS_CMDLINE
static void process_cmdline(struct bootloader_ops *boot_ops,
				int bootdevice)
{
	/* Insert here command prompt processing */
	printf("Omapboot> ");
	ldelay(0x10000000);
	printf(". ");
	ldelay(0x10000000);
	printf(". ");
	ldelay(0x10000000);
	printf(".\n");
}
#endif

void sboot(u32 bootops_addr, int bootdevice)
{
	char buf[DEV_STR_LENGTH];
	boot_ops = (struct bootloader_ops *)bootops_addr;

	init_memory_alloc();

	dev_to_devstr(bootdevice, buf);
	printf("Second Stage Boot: boot device: %s\n", buf);
	if (bootdevice != DEVICE_USB) {
		if (boot_ops->board_ops->board_user_fastboot_request) {
			if (boot_ops->board_ops->board_user_fastboot_request()) {
				usb_init(&boot_ops->usb);
				do_fastboot(boot_ops);
			}
		}
	}

#ifdef BOARD_PROCESS_CMDLINE
	do_cmdline(boot_ops, bootdevice);
#else
	if (bootdevice == DEVICE_USB) {
		do_fastboot(boot_ops);
	} else
		do_booti(boot_ops, "storage", NULL);
#endif
	printf("boot failed\n");
	while (1)
		;
}
