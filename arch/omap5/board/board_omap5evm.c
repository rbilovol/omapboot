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

#include <config.h>

#include <aboot/aboot.h>
#include <aboot/io.h>

#include <common/common_proc.h>
#include <common/omap_rom.h>
#include <common/usbboot_common.h>

#include <omap5/hw.h>
#include <omap5/mux.h>

static struct partition partitions[] = {
	{ "-", 128 },
	{ "xloader", 256 },
	{ "bootloader", 256 },
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
	{ 0, 0 },
};

static struct partition * omap5evm_get_partition(void)
{
	return partitions;
}

static void omap5evm_mux_init(void)
{
	/* core padconf essential */
	setup_core_padconf(CP(EMMC_CLK), (PTU | IEN | M0));
	setup_core_padconf(CP(EMMC_CMD), (PTU | IEN | M0));
	setup_core_padconf(CP(EMMC_DATA0), (PTU | IEN | M0));
	setup_core_padconf(CP(EMMC_DATA1), (PTU | IEN | M0));
	setup_core_padconf(CP(EMMC_DATA2), (PTU | IEN | M0));
	setup_core_padconf(CP(EMMC_DATA3), (PTU | IEN | M0));
	setup_core_padconf(CP(EMMC_DATA4), (PTU | IEN | M0));
	setup_core_padconf(CP(EMMC_DATA5), (PTU | IEN | M0));
	setup_core_padconf(CP(EMMC_DATA6), (PTU | IEN | M0));
	setup_core_padconf(CP(EMMC_DATA7), (PTU | IEN | M0));
	setup_core_padconf(CP(SDCARD_CLK), (PTU | IEN | M0));
	setup_core_padconf(CP(SDCARD_CMD), (PTU | IEN | M0));
	setup_core_padconf(CP(SDCARD_DATA0), (PTU | IEN | M0));
	setup_core_padconf(CP(SDCARD_DATA1), (PTU | IEN | M0));
	setup_core_padconf(CP(SDCARD_DATA2), (PTU | IEN | M0));
	setup_core_padconf(CP(SDCARD_DATA3), (PTU | IEN | M0));
	setup_core_padconf(CP(UART3_RX_IRRX), (PTU | IEN | M0));
	setup_core_padconf(CP(UART3_TX_IRTX), (M0));

	/* wakeup padconf essential */
	setup_wakeup_padconf(WK(SR_PMIC_SCL), (PTU | IEN | M0));
	setup_wakeup_padconf(WK(SR_PMIC_SDA), (PTU | IEN | M0));
	setup_wakeup_padconf(WK(SYS_32K), (IEN | M0));

	/* core padconf non-essential */
	setup_core_padconf(CP(C2C_DATAIN0), (IEN | M0));
	setup_core_padconf(CP(C2C_DATAIN1), (IEN | M0));
	setup_core_padconf(CP(C2C_DATAIN2), (IEN | M0));
	setup_core_padconf(CP(C2C_DATAIN3), (IEN | M0));
	setup_core_padconf(CP(C2C_DATAIN4), (IEN | M0));
	setup_core_padconf(CP(C2C_DATAIN5), (IEN | M0));
	setup_core_padconf(CP(C2C_DATAIN6), (IEN | M0));
	setup_core_padconf(CP(C2C_DATAIN7), (IEN | M0));
	setup_core_padconf(CP(C2C_CLKIN1), (IEN | M0));
	setup_core_padconf(CP(C2C_CLKIN0), (IEN | M0));
	setup_core_padconf(CP(C2C_CLKOUT0), (M0));
	setup_core_padconf(CP(C2C_CLKOUT1), (M0));
	setup_core_padconf(CP(C2C_DATAOUT0), (M0));
	setup_core_padconf(CP(C2C_DATAOUT1), (M0));
	setup_core_padconf(CP(C2C_DATAOUT2), (M0));
	setup_core_padconf(CP(C2C_DATAOUT3), (M0));
	setup_core_padconf(CP(C2C_DATAOUT4), (M0));
	setup_core_padconf(CP(C2C_DATAOUT5), (M0));
	setup_core_padconf(CP(C2C_DATAOUT6), (M0));
	setup_core_padconf(CP(C2C_DATAOUT7), (M0));
	setup_core_padconf(CP(C2C_DATA8), (IEN | M0));
	setup_core_padconf(CP(C2C_DATA9), (IEN | M0));
	setup_core_padconf(CP(C2C_DATA10), (IEN | M0));
	setup_core_padconf(CP(C2C_DATA11), (IEN | M0));
	setup_core_padconf(CP(C2C_DATA12), (IEN | M0));
	setup_core_padconf(CP(C2C_DATA13), (IEN | M0));
	setup_core_padconf(CP(C2C_DATA14), (IEN | M0));
	setup_core_padconf(CP(C2C_DATA15), (IEN | M0));
	setup_core_padconf(CP(LLIB_WAKEREQOUT), (PTU | IEN | M6));
	setup_core_padconf(CP(LLIA_WAKEREQOUT), (M1));
	setup_core_padconf(CP(HSI1_ACREADY), (PTD | M6));
	setup_core_padconf(CP(HSI1_CAREADY), (PTD | M6));
	setup_core_padconf(CP(HSI1_ACWAKE), (PTD | IEN | M6));
	setup_core_padconf(CP(HSI1_CAWAKE), (PTU | IEN | M6));
	setup_core_padconf(CP(HSI1_ACFLAG), (PTD | IEN | M6));
	setup_core_padconf(CP(HSI1_ACDATA), (PTD | M6));
	setup_core_padconf(CP(HSI1_CAFLAG), (M6));
	setup_core_padconf(CP(HSI1_CADATA), (M6));
	setup_core_padconf(CP(UART1_TX), (M0));
	setup_core_padconf(CP(UART1_CTS), (PTU | IEN | M0));
	setup_core_padconf(CP(UART1_RX), (PTU | IEN | M0));
	setup_core_padconf(CP(UART1_RTS), (M0));
	setup_core_padconf(CP(HSI2_CAREADY), (IEN | M0));
	setup_core_padconf(CP(HSI2_ACREADY), (OFF_EN | M0));
	setup_core_padconf(CP(HSI2_CAWAKE), (IEN | PTD | M0));
	setup_core_padconf(CP(HSI2_ACWAKE), (M0));
	setup_core_padconf(CP(HSI2_CAFLAG), (IEN | PTD | M0));
	setup_core_padconf(CP(HSI2_CADATA), (IEN | PTD | M0));
	setup_core_padconf(CP(HSI2_ACDATA), (M0));
	setup_core_padconf(CP(HSI2_ACFLAG), (M0));
	setup_core_padconf(CP(UART2_RTS), (IEN | M1));
	setup_core_padconf(CP(UART2_CTS), (IEN | M1));
	setup_core_padconf(CP(UART2_RX), (IEN | M1));
	setup_core_padconf(CP(UART2_TX), (IEN | M1));
	setup_core_padconf(CP(USBB1_HSIC_STROBE), (PTU | IEN | M0));
	setup_core_padconf(CP(USBB1_HSIC_DATA), (PTU | IEN | M0));
	setup_core_padconf(CP(USBB2_HSIC_STROBE), (PTU | IEN | M0));
	setup_core_padconf(CP(USBB2_HSIC_DATA), (PTU | IEN | M0));
	setup_core_padconf(CP(TIMER10_PWM_EVT), (IEN | M0));
	setup_core_padconf(CP(DSIPORTA_TE0), (IEN | M0));
	setup_core_padconf(CP(DSIPORTA_LANE0X), (IEN | M0));
	setup_core_padconf(CP(DSIPORTA_LANE0Y), (IEN | M0));
	setup_core_padconf(CP(DSIPORTA_LANE1X), (IEN | M0));
	setup_core_padconf(CP(DSIPORTA_LANE1Y), (IEN | M0));
	setup_core_padconf(CP(DSIPORTA_LANE2X), (IEN | M0));
	setup_core_padconf(CP(DSIPORTA_LANE2Y), (IEN | M0));
	setup_core_padconf(CP(DSIPORTA_LANE3X), (IEN | M0));
	setup_core_padconf(CP(DSIPORTA_LANE3Y), (IEN | M0));
	setup_core_padconf(CP(DSIPORTA_LANE4X), (IEN | M0));
	setup_core_padconf(CP(DSIPORTA_LANE4Y), (IEN | M0));
	setup_core_padconf(CP(TIMER9_PWM_EVT), (IEN | M0));
	setup_core_padconf(CP(DSIPORTC_TE0), (IEN | M0));
	setup_core_padconf(CP(DSIPORTC_LANE0X), (IEN | M0));
	setup_core_padconf(CP(DSIPORTC_LANE0Y), (IEN | M0));
	setup_core_padconf(CP(DSIPORTC_LANE1X), (IEN | M0));
	setup_core_padconf(CP(DSIPORTC_LANE1Y), (IEN | M0));
	setup_core_padconf(CP(DSIPORTC_LANE2X), (IEN | M0));
	setup_core_padconf(CP(DSIPORTC_LANE2Y), (IEN | M0));
	setup_core_padconf(CP(DSIPORTC_LANE3X), (IEN | M0));
	setup_core_padconf(CP(DSIPORTC_LANE3Y), (IEN | M0));
	setup_core_padconf(CP(DSIPORTC_LANE4X), (IEN | M0));
	setup_core_padconf(CP(DSIPORTC_LANE4Y), (IEN | M0));
	setup_core_padconf(CP(RFBI_HSYNC0), (M4));
	setup_core_padconf(CP(RFBI_TE_VSYNC0), (PTD | M6));
	setup_core_padconf(CP(RFBI_RE), (M4));
	setup_core_padconf(CP(RFBI_A0), (PTD | IEN | M6));
	setup_core_padconf(CP(RFBI_DATA8), (M4));
	setup_core_padconf(CP(RFBI_DATA9), (PTD | M6));
	setup_core_padconf(CP(RFBI_DATA10), (PTD | M6));
	setup_core_padconf(CP(RFBI_DATA11), (PTD | M6));
	setup_core_padconf(CP(RFBI_DATA12), (PTD | M6));
	setup_core_padconf(CP(RFBI_DATA13), (PTU | IEN | M6));
	setup_core_padconf(CP(RFBI_DATA14), (M4));
	setup_core_padconf(CP(RFBI_DATA15), (M4));
	setup_core_padconf(CP(GPIO6_182), (M6));
	setup_core_padconf(CP(GPIO6_183), (PTD | M6));
	setup_core_padconf(CP(GPIO6_184), (M4));
	setup_core_padconf(CP(GPIO6_185), (PTD | IEN | M6));
	setup_core_padconf(CP(GPIO6_186), (PTD | M6));
	setup_core_padconf(CP(GPIO6_187), (PTU | IEN | M4));
	setup_core_padconf(CP(RFBI_DATA0), (PTD | M6));
	setup_core_padconf(CP(RFBI_DATA1), (PTD | M6));
	setup_core_padconf(CP(RFBI_DATA2), (PTD | M6));
	setup_core_padconf(CP(RFBI_DATA3), (PTD | IEN | M6));
	setup_core_padconf(CP(RFBI_DATA4), (IEN | M6));
	setup_core_padconf(CP(RFBI_DATA5), (IEN | M6));
	setup_core_padconf(CP(RFBI_DATA6), (PTD | M6));
	setup_core_padconf(CP(RFBI_DATA7), (PTD | M6));
	setup_core_padconf(CP(RFBI_CS0), (PTD | IEN | M6));
	setup_core_padconf(CP(RFBI_WE), (PTD | M6));
	setup_core_padconf(CP(MCSPI2_CS0), (M0));
	setup_core_padconf(CP(MCSPI2_CLK), (IEN | M0));
	setup_core_padconf(CP(MCSPI2_SIMO), (IEN | M0));
	setup_core_padconf(CP(MCSPI2_SOMI), (PTU | IEN | M0));
	setup_core_padconf(CP(I2C4_SCL), (IEN | M0));
	setup_core_padconf(CP(I2C4_SDA), (IEN | M0));
	setup_core_padconf(CP(HDMI_CEC), (IEN | M0));
	setup_core_padconf(CP(HDMI_HPD), (PTD | IEN | M0));
	setup_core_padconf(CP(HDMI_DDC_SCL), (IEN | M0));
	setup_core_padconf(CP(HDMI_DDC_SDA), (IEN | M0));
	setup_core_padconf(CP(CSIPORTA_LANE0X), (IEN | M0));
	setup_core_padconf(CP(CSIPORTA_LANE0Y), (IEN | M0));
	setup_core_padconf(CP(CSIPORTA_LANE1Y), (IEN | M0));
	setup_core_padconf(CP(CSIPORTA_LANE1X), (IEN | M0));
	setup_core_padconf(CP(CSIPORTA_LANE2Y), (IEN | M0));
	setup_core_padconf(CP(CSIPORTA_LANE2X), (IEN | M0));
	setup_core_padconf(CP(CSIPORTA_LANE3X), (IEN | M0));
	setup_core_padconf(CP(CSIPORTA_LANE3Y), (IEN | M0));
	setup_core_padconf(CP(CSIPORTA_LANE4X), (IEN | M0));
	setup_core_padconf(CP(CSIPORTA_LANE4Y), (IEN | M0));
	setup_core_padconf(CP(CSIPORTB_LANE0X), (IEN | M0));
	setup_core_padconf(CP(CSIPORTB_LANE0Y), (IEN | M0));
	setup_core_padconf(CP(CSIPORTB_LANE1Y), (IEN | M0));
	setup_core_padconf(CP(CSIPORTB_LANE1X), (IEN | M0));
	setup_core_padconf(CP(CSIPORTB_LANE2Y), (IEN | M0));
	setup_core_padconf(CP(CSIPORTB_LANE2X), (IEN | M0));
	setup_core_padconf(CP(CSIPORTC_LANE0Y), (IEN | M0));
	setup_core_padconf(CP(CSIPORTC_LANE0X), (IEN | M0));
	setup_core_padconf(CP(CSIPORTC_LANE1Y), (IEN | M0));
	setup_core_padconf(CP(CSIPORTC_LANE1X), (IEN | M0));
	setup_core_padconf(CP(CAM_SHUTTER), (M0));
	setup_core_padconf(CP(CAM_STROBE), (M0));
	setup_core_padconf(CP(CAM_GLOBALRESET), (IEN | M0));
	setup_core_padconf(CP(TIMER11_PWM_EVT), (PTD | M6));
	setup_core_padconf(CP(TIMER5_PWM_EVT), (PTD | M6));
	setup_core_padconf(CP(TIMER6_PWM_EVT), (PTD | M6));
	setup_core_padconf(CP(TIMER8_PWM_EVT), (PTU | M6));
	setup_core_padconf(CP(I2C3_SCL), (IEN | M0));
	setup_core_padconf(CP(I2C3_SDA), (IEN | M0));
	setup_core_padconf(CP(GPIO8_233), (IEN | M2));
	setup_core_padconf(CP(ABE_CLKS), (IEN | M0));
	setup_core_padconf(CP(ABEDMIC_DIN1), (IEN | M0));
	setup_core_padconf(CP(ABEDMIC_DIN2), (IEN | M0));
	setup_core_padconf(CP(ABEDMIC_DIN3), (IEN | M0));
	setup_core_padconf(CP(ABEDMIC_CLK1), (M0));
	setup_core_padconf(CP(ABEDMIC_CLK2), (IEN | M1));
	setup_core_padconf(CP(ABEDMIC_CLK3), (M1));
	setup_core_padconf(CP(ABESLIMBUS1_CLOCK), (IEN | M1));
	setup_core_padconf(CP(ABESLIMBUS1_DATA), (IEN | M1));
	setup_core_padconf(CP(ABEMCBSP2_DR), (IEN | M0));
	setup_core_padconf(CP(ABEMCBSP2_DX), (M0));
	setup_core_padconf(CP(ABEMCBSP2_FSX), (IEN | M0));
	setup_core_padconf(CP(ABEMCBSP2_CLKX), (IEN | M0));
	setup_core_padconf(CP(ABEMCPDM_UL_DATA),
			(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0));
	setup_core_padconf(CP(ABEMCPDM_DL_DATA),
			(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0));
	setup_core_padconf(CP(ABEMCPDM_FRAME),
			(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));
	setup_core_padconf(CP(ABEMCPDM_LB_CLK),
			(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0));
	setup_core_padconf(CP(WLSDIO_CLK), (PTU | IEN | M0));
	setup_core_padconf(CP(WLSDIO_CMD), (PTU | IEN | M0));
	setup_core_padconf(CP(WLSDIO_DATA0), (PTU | IEN | M0));
	setup_core_padconf(CP(WLSDIO_DATA1), (PTU | IEN | M0));
	setup_core_padconf(CP(WLSDIO_DATA2), (PTU | IEN | M0));
	setup_core_padconf(CP(WLSDIO_DATA3), (PTU | IEN | M0));
	setup_core_padconf(CP(UART5_RX), (PTU | IEN | M0));
	setup_core_padconf(CP(UART5_TX), (M0));
	setup_core_padconf(CP(UART5_CTS), (PTU | IEN | M0));
	setup_core_padconf(CP(UART5_RTS), (M0));
	setup_core_padconf(CP(I2C2_SCL), (IEN | M0));
	setup_core_padconf(CP(I2C2_SDA), (IEN | M0));
	setup_core_padconf(CP(MCSPI1_CLK), (M6));
	setup_core_padconf(CP(MCSPI1_SOMI), (IEN | M6));
	setup_core_padconf(CP(MCSPI1_SIMO), (PTD | M6));
	setup_core_padconf(CP(MCSPI1_CS0), (PTD | M6));
	setup_core_padconf(CP(MCSPI1_CS1), (PTD | IEN | M6));
	setup_core_padconf(CP(I2C5_SCL), (IEN | M0));
	setup_core_padconf(CP(I2C5_SDA), (IEN | M0));
	setup_core_padconf(CP(PERSLIMBUS2_CLOCK), (PTD | M6));
	setup_core_padconf(CP(PERSLIMBUS2_DATA), (PTD | IEN | M6));
	setup_core_padconf(CP(UART6_TX), (PTU | IEN | M6));
	setup_core_padconf(CP(UART6_RX), (PTU | IEN | M6));
	setup_core_padconf(CP(UART6_CTS), (PTU | IEN | M6));
	setup_core_padconf(CP(UART6_RTS), (PTU | M0));
	setup_core_padconf(CP(UART3_CTS_RCTX), (PTU | IEN | M6));
	setup_core_padconf(CP(UART3_RTS_IRSD), (PTU | IEN | M1));
	setup_core_padconf(CP(USBB3_HSIC_STROBE), (PTU | IEN | M0));
	setup_core_padconf(CP(USBB3_HSIC_DATA), (PTU | IEN | M0));
	setup_core_padconf(CP(USBD0_HS_DP), (IEN | M0));
	setup_core_padconf(CP(USBD0_HS_DM), (IEN | M0));
	setup_core_padconf(CP(USBD0_SS_RX), (IEN | M0));
	setup_core_padconf(CP(I2C1_PMIC_SCL), (PTU | IEN | M0));
	setup_core_padconf(CP(I2C1_PMIC_SDA), (PTU | IEN | M0));

	/* wakeup padconf non-essential */
	setup_wakeup_padconf(WK(LLIA_WAKEREQIN), (M7));
	setup_wakeup_padconf(WK(LLIB_WAKEREQIN), (M7));
	setup_wakeup_padconf(WK(DRM_EMU0), (PTU | IEN | M0));
	setup_wakeup_padconf(WK(DRM_EMU1), (PTU | IEN | M0));
	setup_wakeup_padconf(WK(JTAG_NTRST), (IEN | M0));
	setup_wakeup_padconf(WK(JTAG_TCK), (IEN | M0));
	setup_wakeup_padconf(WK(JTAG_RTCK), (M0));
	setup_wakeup_padconf(WK(JTAG_TMSC), (IEN | M0));
	setup_wakeup_padconf(WK(JTAG_TDI), (IEN | M0));
	setup_wakeup_padconf(WK(JTAG_TDO), (M0));
	setup_wakeup_padconf(WK(FREF_CLK_IOREQ), (IEN | M0));
	setup_wakeup_padconf(WK(FREF_CLK0_OUT), (M0));
	setup_wakeup_padconf(WK(FREF_CLK1_OUT), (M0));
	setup_wakeup_padconf(WK(FREF_CLK2_OUT), (M0));
	setup_wakeup_padconf(WK(FREF_CLK2_REQ), (PTU | IEN | M6));
	setup_wakeup_padconf(WK(FREF_CLK1_REQ), (PTD | IEN | M6));
	setup_wakeup_padconf(WK(SYS_NRESPWRON), (IEN | M0));
	setup_wakeup_padconf(WK(SYS_NRESWARM), (PTU | IEN | M0));
	setup_wakeup_padconf(WK(SYS_PWR_REQ), (M0));
	setup_wakeup_padconf(WK(SYS_NIRQ1), (PTU | IEN | M0));
	setup_wakeup_padconf(WK(SYS_NIRQ2), (PTU | IEN | M0));
	setup_wakeup_padconf(WK(SYS_BOOT0), (IEN | M0));
	setup_wakeup_padconf(WK(SYS_BOOT1), (IEN | M0));
	setup_wakeup_padconf(WK(SYS_BOOT2), (IEN | M0));
	setup_wakeup_padconf(WK(SYS_BOOT3), (IEN | M0));
	setup_wakeup_padconf(WK(SYS_BOOT4), (IEN | M0));
	setup_wakeup_padconf(WK(SYS_BOOT5), (IEN | M0));
}

static void omap5evm_late_init(void)
{
	/* enable uart3 console */
	writel(2, 0x4A009550);
}

static int omap5evm_check_fastboot(void)
{
	u32 temp;
	/* set the clock for the keypad */
	sr32(CM_WKUPAON_KBD_CLKCTRL, 0, 2, 0x02);
	/* any key pressed ? */
	temp = readl(KBD_STATEMACHINE);
	if (temp == 0)
		return 0;
	sdelay(200000);
	temp = readl(KBD_FULLCODE31_0);
	if ((temp & USER_FASTBOOT_RQ) == USER_FASTBOOT_RQ) {
		printf("Keypress detected: going to fastboot mode\n");
		return 1;
	}
	return 0;
}

static u8 omap5evm_get_flash_slot(void)
{
	return DEVICE_EMMC;
}

static void omap5evm_scale_cores(void)
{
	/* Use default OMAP voltage */
	scale_vcores();
}

static void omap5evm_gpmc_init(void)
{
	/* Use default OMAP gpmc init function */
	gpmc_init();
}

static void omap5evm_prcm_init(void)
{
	/* Use default OMAP gpmc init function */
	prcm_init();
}

static struct storage_specific_functions *omap5evm_storage_init(void)
{
	int ret;
	struct storage_specific_functions *storage_ops;
	storage_ops = init_rom_mmc_funcs(omap5evm_get_flash_slot());
	if (!storage_ops) {
		printf("Unable to get rom mmc functions\n");
		return NULL;
	}
	ret = storage_ops->init();
	if (ret) {
		printf("Unable to init storage device\n");
		return NULL;
	}
	return storage_ops;
}

static struct board_specific_functions omap5evm_funcs = {
	.board_get_flash_slot = omap5evm_get_flash_slot,
	.board_mux_init = omap5evm_mux_init,
	.board_user_fastboot_request = omap5evm_check_fastboot,
	.board_late_init = omap5evm_late_init,
	.board_get_part_tbl = omap5evm_get_partition,
	.board_scale_vcores = omap5evm_scale_cores,
	.board_gpmc_init = omap5evm_gpmc_init,
	.board_prcm_init = omap5evm_prcm_init,
	.board_storage_init = omap5evm_storage_init,
};

void* init_board_funcs(void)
{
	return &omap5evm_funcs;
}
