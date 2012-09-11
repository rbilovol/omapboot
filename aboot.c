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

#define WITH_MEMORY_TEST	0
#define WITH_FLASH_BOOT		0

#if WITH_MEMORY_TEST
void memtest(void *x, unsigned count) {
	unsigned *w = x;
	unsigned n;
	count /= 4;

	printf("memtest write - %d\n",count);
	for (n = 0; n < count; n++) {
		unsigned chk = 0xa5a5a5a5 ^ n;
		w[n] = chk;
	}
	printf("memtest read\n");
	for (n = 0; n < count; n++) {
		unsigned chk = 0xa5a5a5a5 ^ n;
		if (w[n] != chk) {
			printf("ERROR @ %x (%x != %x)\n", 
				(unsigned) (w+n), w[n], chk);
			return;
		}
	}
	printf("OK!\n");
}
#endif

static unsigned MSG = 0xaabbccdd;

struct usb usb;

unsigned cfg_machine_type = CONFIG_BOARD_MACH_TYPE;

u32 public_rom_base;

__attribute__((__section__(".mram")))
static struct bootloader_ops boot_operations;
struct bootloader_ops *boot_ops = &boot_operations;

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

static int load_from_usb(unsigned *_len)
{
	unsigned len, n;
	enable_irqs();

	if (usb_open(&usb))
		return -1;

	usb_queue_read(&usb, &len, 4);
	usb_write(&usb, &MSG, 4);
	n = usb_wait_read(&usb);
	if (n)
		return -1;

	if (usb_read(&usb, (void*) CONFIG_ADDR_DOWNLOAD, len))
		return -1;

	usb_close(&usb);

	disable_irqs();
	*_len = len;
	return 0;
}

void aboot(unsigned *info)
{
	unsigned n, len;
	int ret = 0;

	boot_ops->board_ops = init_board_funcs();
	boot_ops->proc_ops = init_processor_id_funcs();

	if (boot_ops->proc_ops->proc_get_api_base)
		public_rom_base = boot_ops->proc_ops->proc_get_api_base();

	if (boot_ops->board_ops->board_mux_init)
		boot_ops->board_ops->board_mux_init();

	ldelay(100);

	if (boot_ops->board_ops->board_scale_vcores)
		boot_ops->board_ops->board_scale_vcores();

	if(boot_ops->board_ops->board_prcm_init)
		boot_ops->board_ops->board_prcm_init();

	if (boot_ops->board_ops->board_ddr_init)
		boot_ops->board_ops->board_ddr_init(boot_ops->proc_ops);

	if (boot_ops->board_ops->board_gpmc_init)
		boot_ops->board_ops->board_gpmc_init();

	if (boot_ops->board_ops->board_late_init)
		boot_ops->board_ops->board_late_init();

	serial_init();

	printf("%s\n", ABOOT_VERSION);
	printf("Build Info: "__DATE__ " - " __TIME__ "\n");

	if (!boot_ops->board_ops->board_get_flash_slot)
		goto fail;

	boot_ops->storage_ops =
		init_rom_mmc_funcs(boot_ops->board_ops->board_get_flash_slot());
	if (!boot_ops->storage_ops) {
		printf("Unable to init rom mmc functions\n");
		goto fail;
	}

	if (boot_ops->board_ops->board_storage_init)
		ret = boot_ops->board_ops->board_storage_init
			(boot_ops->board_ops->board_get_flash_slot(),
							boot_ops->storage_ops);
		if (ret != 0) {
			printf("Storage driver init failed\n");
			goto fail;
		}

	/* printf("MSV=%08x\n",*((unsigned*) 0x4A00213C)); */

#if WITH_MEMORY_TEST
	memtest(0x82000000, 8*1024*1024);
	memtest(0xA0208000, 8*1024*1024);
#endif

#if !WITH_FLASH_BOOT
	n = load_from_usb(&len);
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
		n = load_from_usb(&len);
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

		do_booti("ram", NULL);
		serial_puts("*** BOOT FAILED ***\n");
	}

fail:
	for (;;) ;
}
