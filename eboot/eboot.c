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

#include <aboot/aboot.h>
#include <aboot/io.h>
#include <common/omap_rom.h>
#include <common/fastboot.h>

#if defined CONFIG_IS_OMAP4
#include <omap4/mux.h>
#include <omap4/hw.h>
#elif defined CONFIG_IS_OMAP5
#include <omap5/mux.h>
#include <omap5/hw.h>
#endif

#include "config.h"

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

struct usb usb;

unsigned cfg_machine_type = CONFIG_BOARD_MACH_TYPE;

u32 public_rom_base;

#if defined CONFIG_PANDA
u8 device = DEVICE_SDCARD;
#elif defined CONFIG_BLAZE || defined CONFIG_OMAP5EVM || \
				defined CONFIG_BLAZE_TABLET
u8 device = DEVICE_EMMC;
#endif

void eboot(unsigned *info)
{
	int ret = 0;
	unsigned bootdevice = -1;

	if (get_omap_rev() >= OMAP_5430_ES1_DOT_0)
		public_rom_base = PUBLIC_API_BASE_5430;
	else if (get_omap_rev() >= OMAP_4460_ES1_DOT_1)
		public_rom_base = PUBLIC_API_BASE_4460;
	else
		public_rom_base = PUBLIC_API_BASE_4430;

	watchdog_disable();

	board_mux_init();
	sdelay(100);

	scale_vcores();
	prcm_init();
	board_ddr_init();
	gpmc_init();
	board_late_init();

	serial_init();

	serial_puts("\n[ eboot second-stage loader ]\n\n");

	#if defined CONFIG_IS_OMAP4
	hal_i2c i2c_id = HAL_I2C1;

	ret = i2c_init(i2c_id);
	if (ret != 0) {
		printf("\nFailed to init I2C-%d\n", i2c_id);
		goto fail;
	} else
		printf("\nInitialized I2C-%d\n", i2c_id);

	ret = pmic_enable();
	if (ret != 0) {
		printf("could not enable the pmic\n");
		goto fail;
	} else {
		printf("Configure the pbias\n");
		pbias_config();
	}
	#endif

	enable_irqs();

	ret = usb_open(&usb);
	if (ret != 0) {
		printf("\nusb_open failed\n");
		goto fail;
	}

	if (info)
		bootdevice = info[2] & 0xFF;

	switch (bootdevice) {
	case 0x05:
		serial_puts("boot device: MMC1\n\n");
		do_booti("mmc");
		break;
	case 0x06:
	case 0x07:
		serial_puts("boot device: MMC2\n\n");
		do_booti("mmc");
		break;
	default:
		serial_puts("boot device: unknown\n");
	}

	serial_puts("\nstay in SRAM and enter FASTBOOT mode\n");
	do_fastboot();

fail:
	while (1)
		;
}
