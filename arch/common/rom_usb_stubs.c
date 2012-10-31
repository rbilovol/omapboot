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
#include <io.h>
#include <omap_rom.h>
#include <string.h>

void usb_queue_read(struct usb *usb, void *data, unsigned len)
{
	printf("ERROR! this %s should never be called\n", __func__);
}

int usb_wait_read(struct usb *usb)
{
	printf("ERROR! this %s should never be called\n", __func__);
	return 0;
}

int usb_read(struct usb *usb, void *data, unsigned len)
{
	printf("ERROR! this %s should never be called\n", __func__);
	return 0;
}

void usb_queue_write(struct usb *usb, void *data, unsigned len)
{
	printf("ERROR! this %s should never be called\n", __func__);
}

int usb_wait_write(struct usb *usb)
{
	printf("ERROR! this %s should never be called\n", __func__);
	return 0;
}

int usb_write(struct usb *usb, void *data, unsigned len)
{
	printf("ERROR! this %s should never be called\n", __func__);
	return 0;
}

void usb_init(struct usb *usb)
{
	printf("ERROR! this %s should never be called\n", __func__);
}

void usb_close(struct usb *usb)
{
	printf("ERROR! this %s should never be called\n", __func__);
}

int usb_open(struct usb *usb, int init,
				struct proc_specific_functions *proc_ops)
{
	printf("ERROR! this %s should never be called\n", __func__);
	return 0;
}
