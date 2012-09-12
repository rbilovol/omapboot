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

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

struct usb usb;
u32 public_rom_base;

__attribute__((__section__(".mram")))
static struct bootloader_ops boot_operations;

void eboot(unsigned *info)
{
	int ret = 0;
	unsigned bootdevice = -1;
	char buf[DEV_STR_LENGTH];

	struct bootloader_ops *boot_ops = &boot_operations;

	boot_ops->board_ops = init_board_funcs();
	boot_ops->proc_ops = init_processor_id_funcs();

	if (boot_ops->proc_ops->proc_check_lpddr2_temp)
		boot_ops->proc_ops->proc_check_lpddr2_temp();

	if (boot_ops->proc_ops->proc_get_api_base)
		public_rom_base = boot_ops->proc_ops->proc_get_api_base();

	watchdog_disable();

	if (boot_ops->board_ops->board_mux_init)
		boot_ops->board_ops->board_mux_init();

	if (boot_ops->board_ops->board_ddr_init)
		boot_ops->board_ops->board_ddr_init(boot_ops->proc_ops);

	if (boot_ops->board_ops->board_signal_integrity_reg_init)
		boot_ops->board_ops->board_signal_integrity_reg_init
							(boot_ops->proc_ops);

	ldelay(100);

	if (boot_ops->board_ops->board_scale_vcores)
		boot_ops->board_ops->board_scale_vcores();

	if(boot_ops->board_ops->board_prcm_init)
		boot_ops->board_ops->board_prcm_init();

	init_memory_alloc();

	if (boot_ops->board_ops->board_gpmc_init)
		boot_ops->board_ops->board_gpmc_init();

	if (boot_ops->board_ops->board_late_init)
		boot_ops->board_ops->board_late_init();

	enable_irqs();

	serial_init();

	printf("%s\n", ABOOT_VERSION);
	printf("Build Info: "__DATE__ " - " __TIME__ "\n");

	if (boot_ops->board_ops->board_pmic_enable)
		boot_ops->board_ops->board_pmic_enable();

	if (boot_ops->board_ops->board_configure_pwm_mode)
		boot_ops->board_ops->board_configure_pwm_mode();

	ret = usb_open(&usb);
	if (ret != 0) {
		printf("\nusb_open failed\n");
		goto fail;
	}

	if (!boot_ops->board_ops->board_get_flash_slot ||
				!boot_ops->board_ops->board_set_flash_slot)
		goto fail;

	boot_ops->storage_ops =
		init_rom_mmc_funcs(boot_ops->proc_ops->proc_get_proc_id(),
				boot_ops->board_ops->board_get_flash_slot());
	if (!boot_ops->storage_ops) {
		printf("Unable to init rom mmc functions\n");
		goto fail;
	}

	if (info)
		bootdevice = info[2] & 0xFF;
	else
		goto fail;

	if (boot_ops->board_ops->board_set_flash_slot)
		ret = boot_ops->board_ops->board_set_flash_slot
					(bootdevice, boot_ops->storage_ops);
		if (ret != 0) {
			DBG("Storage driver init failed\n");
			goto fail;
		}

	dev_to_devstr(bootdevice, buf);
	printf("sram: boot device: %s\n", buf);

	if (boot_ops->board_ops->board_user_fastboot_request)
		if (boot_ops->board_ops->board_user_fastboot_request())
			goto fastboot;

	do_booti(boot_ops, "storage", NULL);

fastboot:
	usb_init(&usb);
	do_fastboot(boot_ops);

fail:
	printf("boot failed\n");
	while (1)
		;
}
