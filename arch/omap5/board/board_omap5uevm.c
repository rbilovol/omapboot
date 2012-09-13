/*
* Copyright (C) 2012 Texas Instruments, Inc.
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
#include <omap_rom.h>
#include <usbboot_common.h>

#include <hw.h>
#include <mux.h>
#include <smartio.h>

#define FASTBOOT_BUTTON_GPIO	83

static struct partition partitions[] = {
	{ "-", 128 },
	{ "bootloader", 256 },
	{ "environment", 256 },
	/* "misc" partition is required for recovery */
	{ "misc", 128 },
	{ "-", 384 },
	{ "efs", 16384 },
	{ "crypto", 16 },
	{ "recovery", 8*1024 },
	{ "boot", 8*1024 },
	{ "system", 512*1024 },
	{ "cache", 256*1024 },
	{ "userdata", 0},
	{ NULL, 0 },
};

static u8 device = DEVICE_EMMC;

static struct partition * omap5uevm_get_partition(void)
{
	return partitions;
}

static void omap5uevm_signal_int_reg_init
				(struct proc_specific_functions *proc_ops)
{
	/* configure smart io */
	configure_smartio(NULL);

#ifdef CONFIG_USE_CH_RAM_CONFIG
	if (proc_ops->proc_get_proc_id) {
		if (proc_ops->proc_get_proc_id() > OMAP_5432_ES1_DOT_0)
			return;
	}
#endif

	/* configure ddr io */
	omap5_ddrio_init(NULL);
}


static void omap5uevm_mux_init(void)
{
	/* core padconf essential */
	setup_core(CONTROL_PADCONF_EMMC_CLK, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_EMMC_CMD, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_EMMC_DATA0, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_EMMC_DATA1, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_EMMC_DATA2, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_EMMC_DATA3, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_EMMC_DATA4, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_EMMC_DATA5, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_EMMC_DATA6, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_EMMC_DATA7, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_SDCARD_CLK, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_SDCARD_CMD, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_SDCARD_DATA0, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_SDCARD_DATA1, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_SDCARD_DATA2, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_SDCARD_DATA3, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_UART3_RX_IRRX, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_UART3_TX_IRTX, (M0));

	/* wakeup padconf essential */
	setup_wakeup(CONTROL_WAKEUP_SR_PMIC_SCL, (PTU | IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_SR_PMIC_SDA, (PTU | IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_SYS_32K, (IEN | M0));

	/* push button (GPIO 83) for fastboot mode */
	setup_core(CONTROL_PADCONF_HSI2_ACDATA, (IEN | M6));
}

/* Use CH (configuration header) to do the settings */
static void omap5uevm_late_init(void)
{
	/* enable uart3 console */
	writel(2, 0x4A009550);
}

static void omap5uevm_scale_cores(void)
{
	/* Use default OMAP voltage */
	scale_vcores();
}

static int omap5uevm_check_fastboot(void)
{
	if (!gpio_read(FASTBOOT_BUTTON_GPIO)) {
		/* small debounce to make sure the button is really pressed */
		ldelay(200000);
		if (!gpio_read(FASTBOOT_BUTTON_GPIO)) {
			printf("Button press detected: go to fastboot mode\n");
			return 1;
		}
	}

	return 0;
}

static u8 omap5uevm_get_flash_slot(void)
{
	return device;
}

static void omap5uevm_prcm_init(void)
{
	prcm_init();
}

static int omap5uevm_read_sw_revision(void)
{
	int ret = 0;

	ret = palmas_read_sw_revision();
	if (ret != 0)
		printf("unable to read palmas sw revision\n");

	return ret;
}

static int omap5uevm_configure_pwm_mode(void)
{
	int ret = 0;

	ret = palmas_configure_pwm_mode();
	if (ret != 0)
		printf("unable to configure PWM mode\n");

	return ret;
}

static int omap5uevm_storage_init(u8 dev,
				struct storage_specific_functions *storage_ops)
{
	int ret = 0;

	ret = storage_ops->init(dev);
	if (ret)
		printf("Unable to init storage device\n");

	return ret;
}

static int omap5uevm_set_flash_slot(u8 dev,
				struct storage_specific_functions *storage_ops)
{
	int ret = 0;
	char buf[DEV_STR_LENGTH];
	u8 prev_dev = device;

	switch (dev) {
	case DEVICE_SDCARD:
	case DEVICE_EMMC:
		device = dev;
		ret = omap5uevm_storage_init(dev, storage_ops);
		if (ret != 0) {
			dev_to_devstr(dev, buf);
			printf("Unable to set flash slot: %s\n", buf);
			device = prev_dev;
		}

		break;
	default:
		printf("Unable to set flash slot: %d\n", dev);
		ret = -1;
	}

	return ret;
}
static u32 omap5uevm_get_board_rev(void)
{
	return 0xff;
}

static struct board_specific_functions omap5uevm_funcs = {
	.board_get_flash_slot = omap5uevm_get_flash_slot,
	.board_set_flash_slot = omap5uevm_set_flash_slot,
	.board_signal_integrity_reg_init = omap5uevm_signal_int_reg_init,
	.board_mux_init = omap5uevm_mux_init,
	.board_user_fastboot_request = omap5uevm_check_fastboot,
	.board_late_init = omap5uevm_late_init,
	.board_get_part_tbl = omap5uevm_get_partition,
	.board_prcm_init = omap5uevm_prcm_init,
	.board_scale_vcores = omap5uevm_scale_cores,
	.board_storage_init = omap5uevm_storage_init,
	.board_read_sw_revision = omap5uevm_read_sw_revision,
	.board_configure_pwm_mode = omap5uevm_configure_pwm_mode,
	.board_get_board_rev = omap5uevm_get_board_rev,
};

void* init_board_funcs(void)
{
	return &omap5uevm_funcs;
}
