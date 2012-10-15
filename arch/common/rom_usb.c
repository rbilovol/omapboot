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
#include <omap_rom.h>
#include <string.h>

#if defined(CONFIG_IS_OMAP5)
static struct usb_ioconf ioconf_read;
static struct usb_ioconf ioconf_write;
volatile struct usb_trb trbout;
#endif

int usb_open(struct usb *usb, int init)
{
	struct per_handle *boot;
	u16 options = 1;
	int n;

	/*clear global usb structure*/
	memset(usb, 0, sizeof(*usb));

	if (init) {
		usb->dread.config_object = NULL;
		usb->dread.options = &options;
		usb->dread.device_type = DEVICE_USB;

		usb->dwrite.config_object = NULL;
		usb->dwrite.options = &options;
		usb->dwrite.device_type = DEVICE_USB;

#if defined(CONFIG_IS_OMAP4)
		usb->dread.xfer_mode = 1;
		usb->dwrite.xfer_mode = 1;
#endif

		n = rom_get_device_data((void *)&usb->dread.device_data);
		if (n)
			return n;

		usb->dwrite.device_data = usb->dread.device_data;
	} else {
		/* get peripheral device descriptor
		that was used during rom usb boot */
		n = rom_get_per_device(&boot);
		if (n)
			return n;

		memcpy(&usb->dread, boot, sizeof(struct per_handle));
		memcpy(&usb->dwrite, boot, sizeof(struct per_handle));
	}


	/* get rom usb driver */
	n = rom_get_per_driver(&usb->io, boot->device_type);
	if (n)
		return n;

	if (init)
		usb_init(usb);

	return 0;
}


static struct usb *local_read_usb;
static void rom_read_callback(struct per_handle *rh)
{

	local_read_usb->dread.status = rh->status;
	return;
}

void usb_queue_read(struct usb *usb, void *data, unsigned len)
{
	int n;

#if defined(CONFIG_IS_OMAP5)
	memset((void *)&trbout, 0, sizeof(trbout));

	ioconf_read.mode         = 0;
	ioconf_read.conf_timeout = 0;
	ioconf_read.trb_pool     = (struct usb_trb *) &trbout;
	usb->dread.config_object = &ioconf_read;

	trbout.ptrlo    = (u32)data;
	trbout.ptrhi    = 0;
	trbout.bufsiz   = ((len >> 9) + ((len & 0x1FF) ? 1 : 0)) << 9;
	trbout.hwo      = HAL_USB_TRB_HWO_HW_OWNED;
	trbout.chn      = HAL_USB_TRB_CHN_NO_CHAIN;
	trbout.lst      = HAL_USB_TRB_LST_LAST;
	trbout.csp      = HAL_USB_TRB_CSP_NO_CONT_SHORT_PACKET;
	trbout.trbctl   = HAL_USB_TRB_TRBCTL_TYPE_NORMAL;
#endif

	usb->dread.data = data;
	usb->dread.length = len;
	usb->dread.status = -1;
	usb->dread.device_type = DEVICE_USB;
#if defined(CONFIG_IS_OMAP4)
	usb->dread.xfer_mode = 1;
#endif
	usb->dread.callback = rom_read_callback;
	local_read_usb = usb;

	n = usb->io->read(&usb->dread);
	if (n)
		usb->dread.status = n;
}

int usb_wait_read(struct usb *usb)
{
	for (;;) {
		if (usb->dread.status == -1)
			continue;
		if (usb->dread.status == STATUS_WAITING)
			continue;
		return usb->dread.status;
	}
}

static struct usb *local_write_usb;
static void rom_write_callback(struct per_handle *rh)
{
	local_write_usb->dwrite.status = rh->status;
	return;
}

void usb_queue_write(struct usb *usb, void *data, unsigned len)
{
	int n;

#if defined(CONFIG_IS_OMAP5)
	ioconf_write.mode         = 0;
	ioconf_write.conf_timeout = 0;
	ioconf_write.trb_pool     = 0;
	usb->dwrite.config_object = &ioconf_write;
#endif
	usb->dwrite.data = data;
	usb->dwrite.length = len;
	usb->dwrite.status = -1;
#if defined(CONFIG_IS_OMAP4)
	usb->dwrite.xfer_mode = 1;
#endif
	usb->dwrite.device_type = DEVICE_USB;
	usb->dwrite.callback = rom_write_callback;
	local_write_usb = usb;
	n = usb->io->write(&usb->dwrite);
	if (n)
		usb->dwrite.status = n;
}

int usb_wait_write(struct usb *usb)
{
	for (;;) {
		if (usb->dwrite.status == -1)
			continue;
		if (usb->dwrite.status == STATUS_WAITING)
			continue;
		return usb->dwrite.status;
	}
}

#define USB_MAX_IO 65536
int usb_read(struct usb *usb, void *data, unsigned len)
{
	unsigned xfer;
	unsigned char *x = data;
	int n;

	while (len > 0) {
		xfer = (len > USB_MAX_IO) ? USB_MAX_IO : len;
		usb_queue_read(usb, x, xfer);
		n = usb_wait_read(usb);
		if (n)
			return n;
		x += xfer;
		len -= xfer;
	}
	return 0;
}

int usb_write(struct usb *usb, void *data, unsigned len)
{
	usb_queue_write(usb, data, len);
	return usb_wait_write(usb);
}

void usb_init(struct usb *usb)
{
	usb->io->init(&usb->dread);
}

void usb_close(struct usb *usb)
{
	usb->io->close(&usb->dread);
}
