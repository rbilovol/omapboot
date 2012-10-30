/*
 * Copyright (C) 2010 The Android Open Source Project
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

#include <mux.h>
#include <hw.h>

#include <omap_rom.h>
#include <usbboot_common.h>

#define WITH_FLASH_BOOT		0

static unsigned MSG = 0xaabbccdd;

unsigned call_trusted(unsigned appid, unsigned procid, unsigned flag, void *args);

int verify(void *data, unsigned len, void *signature, unsigned rights) {
	struct {
		unsigned count;
		void *data;
		unsigned len;
		void *signature;
		unsigned rights;
	} args;
	args.count = 4;
	args.data = data;
	args.len = len;
	args.signature = signature;
	args.rights = rights;
	return call_trusted(12, 0, 0, &args);
}

#if WITH_FLASH_BOOT
static int load_from_mmc(struct storage_specific_functions *storage_ops,
							unsigned *len)
{
	int ret = 0;
	/* FIX ME: Why are we hardcoding? */
	ret = storage_ops->read(512, 512, (void *) CONFIG_ADDR_DOWNLOAD);
	*len = 256 * 1024;
	return ret;
}
#endif

static int load_from_usb(struct usb_specific_functions *usb_ops, unsigned *_len)
{
	unsigned len, n;
	enable_irqs();

	usb_ops->usb_queue_read(usb_ops->usb, &len, 4);
	usb_ops->usb_write(usb_ops->usb, &MSG, 4);
	n = usb_ops->usb_wait_read(usb_ops->usb);
	if (n)
		return -1;

	if (usb_ops->usb_read(usb_ops->usb, (void *)CONFIG_ADDR_DOWNLOAD, len))
		return -1;

	usb_ops->usb_close(usb_ops->usb);

	disable_irqs();
	*_len = len;
	return 0;
}

void aboot(unsigned *info)
{
	unsigned n, len;
	unsigned bootdevice = -1;
	struct bootloader_ops *boot_ops;

	if (info)
		bootdevice = info[2] & 0xFF;
	else
		goto fail;

	boot_ops = boot_common(bootdevice);
	if (!boot_ops)
		goto fail;


#if !WITH_FLASH_BOOT
	n = load_from_usb(boot_ops->usb_ops, &len);
#else
	unsigned bootdevice;

	if (info) {
		bootdevice = info[2] & 0xFF;
	} else {
		bootdevice = 0x45;
	}

	switch (bootdevice) {
	case 0x45: /* USB */
		serial_puts("boot device: USB\n\n");
		n = load_from_usb(boot_ops->usb_ops->usb, &len);
		break;
	case 0x05:
	case 0x06:
		serial_puts("boot device: MMC\n\n");
		n = load_from_mmc(boot_ops->storage_ops, &len);
		break;
	default:
		serial_puts("boot device: unknown\n");
		for (;;) ;
	}
#endif

	if (n) {
		serial_puts("*** IO ERROR ***\n");
	} else {
		if (boot_ops->proc_ops->proc_get_type() ==
			(char *)OMAP_TYPE_SEC) {
			void *data = (void *) (CONFIG_ADDR_DOWNLOAD);
			void *sign = (void *) (CONFIG_ADDR_DOWNLOAD +
								len - 280);
			if ((len < 281) || (len > (32 * 1024 * 1024)))
				goto fail_verify;

			len -= 280;

			n = verify(data, len, sign, 2);
			if (n != 0)
				goto fail_verify;

		fail_verify:
			serial_puts("*** SIGNATURE VERIFICATION FAILED ***\n");
			for (;;) ;
		}

		do_booti(boot_ops, "ram", NULL);
		serial_puts("*** BOOT FAILED ***\n");
	}

fail:
	for (;;) ;
}
