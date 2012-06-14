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

#include <config.h>

#include <aboot/aboot.h>
#include <aboot/io.h>

#include <common/common_proc.h>
#include <common/omap_rom.h>
#include <common/usbboot_common.h>

#include <omap5/hw.h>
#include <omap5/mux.h>
#include <omap5/smartio.h>

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
	{ NULL, 0 },
};

static u8 device = DEVICE_EMMC;

static struct partition * omap5evm_get_partition(void)
{
	return partitions;
}

static void omap5evm_ddr_init(struct proc_specific_functions *proc_ops)
{
#ifdef CONFIG_USE_CH_RAM_CONFIG
	if (proc_ops->proc_get_proc_id) {
		if (proc_ops->proc_get_proc_id() > OMAP_5430_ES1_DOT_0)
			return;
	}
#endif

	omap5_ddr_init();
}

static void omap5evm_mux_init(void)
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

	/* core padconf non-essential */
	setup_core(CONTROL_PADCONF_C2C_DATAIN0, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_DATAIN1, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_DATAIN2, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_DATAIN3, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_DATAIN4, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_DATAIN5, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_DATAIN6, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_DATAIN7, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_CLKIN1, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_CLKIN0, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_CLKOUT0, (M0));
	setup_core(CONTROL_PADCONF_C2C_CLKOUT1, (M0));
	setup_core(CONTROL_PADCONF_C2C_DATAOUT0, (M0));
	setup_core(CONTROL_PADCONF_C2C_DATAOUT1, (M0));
	setup_core(CONTROL_PADCONF_C2C_DATAOUT2, (M0));
	setup_core(CONTROL_PADCONF_C2C_DATAOUT3, (M0));
	setup_core(CONTROL_PADCONF_C2C_DATAOUT4, (M0));
	setup_core(CONTROL_PADCONF_C2C_DATAOUT5, (M0));
	setup_core(CONTROL_PADCONF_C2C_DATAOUT6, (M0));
	setup_core(CONTROL_PADCONF_C2C_DATAOUT7, (M0));
	setup_core(CONTROL_PADCONF_C2C_DATA8, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_DATA9, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_DATA10, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_DATA11, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_DATA12, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_DATA13, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_DATA14, (IEN | M0));
	setup_core(CONTROL_PADCONF_C2C_DATA15, (IEN | M0));
	setup_core(CONTROL_PADCONF_LLIB_WAKEREQOUT, (PTU | IEN | M6));
	setup_core(CONTROL_PADCONF_LLIA_WAKEREQOUT, (M1));
	setup_core(CONTROL_PADCONF_HSI1_ACREADY, (PTD | M6));
	setup_core(CONTROL_PADCONF_HSI1_CAREADY, (PTD | M6));
	setup_core(CONTROL_PADCONF_HSI1_ACWAKE, (PTD | IEN | M6));
	setup_core(CONTROL_PADCONF_HSI1_CAWAKE, (PTU | IEN | M6));
	setup_core(CONTROL_PADCONF_HSI1_ACFLAG, (PTD | IEN | M6));
	setup_core(CONTROL_PADCONF_HSI1_ACDATA, (PTD | M6));
	setup_core(CONTROL_PADCONF_HSI1_CAFLAG, (M6));
	setup_core(CONTROL_PADCONF_HSI1_CADATA, (M6));
	setup_core(CONTROL_PADCONF_UART1_TX, (M0));
	setup_core(CONTROL_PADCONF_UART1_CTS, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_UART1_RX, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_UART1_RTS, (M0));
	setup_core(CONTROL_PADCONF_HSI2_CAREADY, (IEN | M0));
	setup_core(CONTROL_PADCONF_HSI2_ACREADY, (OFF_EN | M0));
	setup_core(CONTROL_PADCONF_HSI2_CAWAKE, (IEN | PTD | M0));
	setup_core(CONTROL_PADCONF_HSI2_ACWAKE, (M0));
	setup_core(CONTROL_PADCONF_HSI2_CAFLAG, (IEN | PTD | M0));
	setup_core(CONTROL_PADCONF_HSI2_CADATA, (IEN | PTD | M0));
	setup_core(CONTROL_PADCONF_HSI2_ACDATA, (M0));
	setup_core(CONTROL_PADCONF_HSI2_ACFLAG, (M0));
	setup_core(CONTROL_PADCONF_UART2_RTS, (IEN | M1));
	setup_core(CONTROL_PADCONF_UART2_CTS, (IEN | M1));
	setup_core(CONTROL_PADCONF_UART2_RX, (IEN | M1));
	setup_core(CONTROL_PADCONF_UART2_TX, (IEN | M1));
	setup_core(CONTROL_PADCONF_USBB1_HSIC_STROBE, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_USBB1_HSIC_DATA, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_USBB2_HSIC_STROBE, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_USBB2_HSIC_DATA, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_TIMER10_PWM_EVT, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTA_TE0, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTA_LANE0X, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTA_LANE0Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTA_LANE1X, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTA_LANE1Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTA_LANE2X, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTA_LANE2Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTA_LANE3X, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTA_LANE3Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTA_LANE4X, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTA_LANE4Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_TIMER9_PWM_EVT, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTC_TE0, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTC_LANE0X, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTC_LANE0Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTC_LANE1X, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTC_LANE1Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTC_LANE2X, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTC_LANE2Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTC_LANE3X, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTC_LANE3Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTC_LANE4X, (IEN | M0));
	setup_core(CONTROL_PADCONF_DSIPORTC_LANE4Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_RFBI_HSYNC0, (M4));
	setup_core(CONTROL_PADCONF_RFBI_TE_VSYNC0, (PTD | M6));
	setup_core(CONTROL_PADCONF_RFBI_RE, (M4));
	setup_core(CONTROL_PADCONF_RFBI_A0, (PTD | IEN | M6));
	setup_core(CONTROL_PADCONF_RFBI_DATA8, (M4));
	setup_core(CONTROL_PADCONF_RFBI_DATA9, (PTD | M6));
	setup_core(CONTROL_PADCONF_RFBI_DATA10, (PTD | M6));
	setup_core(CONTROL_PADCONF_RFBI_DATA11, (PTD | M6));
	setup_core(CONTROL_PADCONF_RFBI_DATA12, (PTD | M6));
	setup_core(CONTROL_PADCONF_RFBI_DATA13, (PTU | IEN | M6));
	setup_core(CONTROL_PADCONF_RFBI_DATA14, (M4));
	setup_core(CONTROL_PADCONF_RFBI_DATA15, (M4));
	setup_core(CONTROL_PADCONF_GPIO6_182, (M6));
	setup_core(CONTROL_PADCONF_GPIO6_183, (PTD | M6));
	setup_core(CONTROL_PADCONF_GPIO6_184, (M4));
	setup_core(CONTROL_PADCONF_GPIO6_185, (PTD | IEN | M6));
	setup_core(CONTROL_PADCONF_GPIO6_186, (PTD | M6));
	setup_core(CONTROL_PADCONF_GPIO6_187, (PTU | IEN | M4));
	setup_core(CONTROL_PADCONF_RFBI_DATA0, (PTD | M6));
	setup_core(CONTROL_PADCONF_RFBI_DATA1, (PTD | M6));
	setup_core(CONTROL_PADCONF_RFBI_DATA2, (PTD | M6));
	setup_core(CONTROL_PADCONF_RFBI_DATA3, (PTD | IEN | M6));
	setup_core(CONTROL_PADCONF_RFBI_DATA4, (IEN | M6));
	setup_core(CONTROL_PADCONF_RFBI_DATA5, (IEN | M6));
	setup_core(CONTROL_PADCONF_RFBI_DATA6, (PTD | M6));
	setup_core(CONTROL_PADCONF_RFBI_DATA7, (PTD | M6));
	setup_core(CONTROL_PADCONF_RFBI_CS0, (PTD | IEN | M6));
	setup_core(CONTROL_PADCONF_RFBI_WE, (PTD | M6));
	setup_core(CONTROL_PADCONF_MCSPI2_CS0, (M0));
	setup_core(CONTROL_PADCONF_MCSPI2_CLK, (IEN | M0));
	setup_core(CONTROL_PADCONF_MCSPI2_SIMO, (IEN | M0));
	setup_core(CONTROL_PADCONF_MCSPI2_SOMI, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_I2C4_SCL, (IEN | M0));
	setup_core(CONTROL_PADCONF_I2C4_SDA, (IEN | M0));
	setup_core(CONTROL_PADCONF_HDMI_CEC, (IEN | M0));
	setup_core(CONTROL_PADCONF_HDMI_HPD, (PTD | IEN | M0));
	setup_core(CONTROL_PADCONF_HDMI_DDC_SCL, (IEN | M0));
	setup_core(CONTROL_PADCONF_HDMI_DDC_SDA, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTA_LANE0X, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTA_LANE0Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTA_LANE1Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTA_LANE1X, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTA_LANE2Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTA_LANE2X, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTA_LANE3X, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTA_LANE3Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTA_LANE4X, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTA_LANE4Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTB_LANE0X, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTB_LANE0Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTB_LANE1Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTB_LANE1X, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTB_LANE2Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTB_LANE2X, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTC_LANE0Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTC_LANE0X, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTC_LANE1Y, (IEN | M0));
	setup_core(CONTROL_PADCONF_CSIPORTC_LANE1X, (IEN | M0));
	setup_core(CONTROL_PADCONF_CAM_SHUTTER, (M0));
	setup_core(CONTROL_PADCONF_CAM_STROBE, (M0));
	setup_core(CONTROL_PADCONF_CAM_GLOBALRESET, (IEN | M0));
	setup_core(CONTROL_PADCONF_TIMER11_PWM_EVT, (PTD | M6));
	setup_core(CONTROL_PADCONF_TIMER5_PWM_EVT, (PTD | M6));
	setup_core(CONTROL_PADCONF_TIMER6_PWM_EVT, (PTD | M6));
	setup_core(CONTROL_PADCONF_TIMER8_PWM_EVT, (PTU | M6));
	setup_core(CONTROL_PADCONF_I2C3_SCL, (IEN | M0));
	setup_core(CONTROL_PADCONF_I2C3_SDA, (IEN | M0));
	setup_core(CONTROL_PADCONF_GPIO8_233, (IEN | M2));
	setup_core(CONTROL_PADCONF_ABE_CLKS, (IEN | M0));
	setup_core(CONTROL_PADCONF_ABEDMIC_DIN1, (IEN | M0));
	setup_core(CONTROL_PADCONF_ABEDMIC_DIN2, (IEN | M0));
	setup_core(CONTROL_PADCONF_ABEDMIC_DIN3, (IEN | M0));
	setup_core(CONTROL_PADCONF_ABEDMIC_CLK1, (M0));
	setup_core(CONTROL_PADCONF_ABEDMIC_CLK2, (IEN | M1));
	setup_core(CONTROL_PADCONF_ABEDMIC_CLK3, (M1));
	setup_core(CONTROL_PADCONF_ABESLIMBUS1_CLOCK, (IEN | M1));
	setup_core(CONTROL_PADCONF_ABESLIMBUS1_DATA, (IEN | M1));
	setup_core(CONTROL_PADCONF_ABEMCBSP2_DR, (IEN | M0));
	setup_core(CONTROL_PADCONF_ABEMCBSP2_DX, (M0));
	setup_core(CONTROL_PADCONF_ABEMCBSP2_FSX, (IEN | M0));
	setup_core(CONTROL_PADCONF_ABEMCBSP2_CLKX, (IEN | M0));
	setup_core(CONTROL_PADCONF_ABEMCPDM_UL_DATA,
			(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0));
	setup_core(CONTROL_PADCONF_ABEMCPDM_DL_DATA,
			(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0));
	setup_core(CONTROL_PADCONF_ABEMCPDM_FRAME,
			(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));
	setup_core(CONTROL_PADCONF_ABEMCPDM_LB_CLK,
			(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0));
	setup_core(CONTROL_PADCONF_WLSDIO_CLK, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_WLSDIO_CMD, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_WLSDIO_DATA0, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_WLSDIO_DATA1, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_WLSDIO_DATA2, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_WLSDIO_DATA3, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_UART5_RX, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_UART5_TX, (M0));
	setup_core(CONTROL_PADCONF_UART5_CTS, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_UART5_RTS, (M0));
	setup_core(CONTROL_PADCONF_I2C2_SCL, (IEN | M0));
	setup_core(CONTROL_PADCONF_I2C2_SDA, (IEN | M0));
	setup_core(CONTROL_PADCONF_MCSPI1_CLK, (M6));
	setup_core(CONTROL_PADCONF_MCSPI1_SOMI, (IEN | M6));
	setup_core(CONTROL_PADCONF_MCSPI1_SIMO, (PTD | M6));
	setup_core(CONTROL_PADCONF_MCSPI1_CS0, (PTD | M6));
	setup_core(CONTROL_PADCONF_MCSPI1_CS1, (PTD | IEN | M6));
	setup_core(CONTROL_PADCONF_I2C5_SCL, (IEN | M0));
	setup_core(CONTROL_PADCONF_I2C5_SDA, (IEN | M0));
	setup_core(CONTROL_PADCONF_PERSLIMBUS2_CLOCK, (PTD | M6));
	setup_core(CONTROL_PADCONF_PERSLIMBUS2_DATA, (PTD | IEN | M6));
	setup_core(CONTROL_PADCONF_UART6_TX, (PTU | IEN | M6));
	setup_core(CONTROL_PADCONF_UART6_RX, (PTU | IEN | M6));
	setup_core(CONTROL_PADCONF_UART6_CTS, (PTU | IEN | M6));
	setup_core(CONTROL_PADCONF_UART6_RTS, (PTU | M0));
	setup_core(CONTROL_PADCONF_UART3_CTS_RCTX, (PTU | IEN | M6));
	setup_core(CONTROL_PADCONF_UART3_RTS_IRSD, (PTU | IEN | M1));
	setup_core(CONTROL_PADCONF_USBB3_HSIC_STROBE, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_USBB3_HSIC_DATA, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_USBD0_HS_DP, (IEN | M0));
	setup_core(CONTROL_PADCONF_USBD0_HS_DM, (IEN | M0));
	setup_core(CONTROL_PADCONF_USBD0_SS_RX, (IEN | M0));
	setup_core(CONTROL_PADCONF_I2C1_PMIC_SCL, (PTU | IEN | M0));
	setup_core(CONTROL_PADCONF_I2C1_PMIC_SDA, (PTU | IEN | M0));

	/* wakeup padconf non-essential */
	setup_wakeup(CONTROL_WAKEUP_LLIA_WAKEREQIN, (M7));
	setup_wakeup(CONTROL_WAKEUP_LLIB_WAKEREQIN, (M7));
	setup_wakeup(CONTROL_WAKEUP_DRM_EMU0, (PTU | IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_DRM_EMU1, (PTU | IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_JTAG_NTRST, (IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_JTAG_TCK, (IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_JTAG_RTCK, (M0));
	setup_wakeup(CONTROL_WAKEUP_JTAG_TMSC, (IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_JTAG_TDI, (IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_JTAG_TDO, (M0));
	setup_wakeup(CONTROL_WAKEUP_FREF_CLK_IOREQ, (IEN | PTD | M0));
	setup_wakeup(CONTROL_WAKEUP_FREF_CLK0_OUT, (M0));
	setup_wakeup(CONTROL_WAKEUP_FREF_CLK1_OUT, (M0));
	setup_wakeup(CONTROL_WAKEUP_FREF_CLK2_OUT, (M0));
	setup_wakeup(CONTROL_WAKEUP_FREF_CLK2_REQ, (PTU | IEN | M6));
	setup_wakeup(CONTROL_WAKEUP_FREF_CLK1_REQ, (PTD | IEN | M6));
	setup_wakeup(CONTROL_WAKEUP_SYS_NRESPWRON, (IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_SYS_NRESWARM, (PTU | IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_SYS_PWR_REQ, (M0));
	setup_wakeup(CONTROL_WAKEUP_SYS_NIRQ1, (PTU | IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_SYS_NIRQ2, (PTU | IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_SYS_BOOT0, (IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_SYS_BOOT1, (IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_SYS_BOOT2, (IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_SYS_BOOT3, (IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_SYS_BOOT4, (IEN | M0));
	setup_wakeup(CONTROL_WAKEUP_SYS_BOOT5, (IEN | M0));

}

static void omap5evm_smartio_init(void)
{

	/* configure smart io */
	configure_smartio(NULL);
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
	set_modify(CM_WKUPAON_KBD_CLKCTRL, 0x00000003, 0x02);
	/* any key pressed ? */
	temp = readl(KBD_STATEMACHINE);
	if (temp == 0)
		return 0;
	ldelay(200000);
	temp = readl(KBD_FULLCODE31_0);
	if ((temp & USER_FASTBOOT_RQ) == USER_FASTBOOT_RQ) {
		printf("Keypress detected: going to fastboot mode\n");
		return 1;
	}
	return 0;
}

static u8 omap5evm_get_flash_slot(void)
{
	return device;
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

static int omap5evm_read_sw_revision(void)
{
	int ret = 0;

	ret = palmas_read_sw_revision();
	if (ret != 0)
		printf("unable to read palmas sw revision\n");

	return ret;
}

static int omap5evm_configure_pwm_mode(void)
{
	int ret = 0;

	ret = palmas_configure_pwm_mode();
	if (ret != 0)
		printf("unable to configure PWM mode\n");

	return ret;
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

static int omap5evm_set_flash_slot(u8 dev)
{
	int ret = 0;
	u8 prev_dev = device;
	switch (dev) {
	case DEVICE_SDCARD:
	case DEVICE_EMMC:
		device = dev;
		if (!omap5evm_storage_init()) {
			printf("Unable to set flash slot: %d\n", dev);
			ret = -1;
			device = prev_dev;
		} else
			break;
	default:
		printf("Unable to set flash slot: %d\n", dev);
		ret = -1;
	}
	return ret;
}

static struct board_specific_functions omap5evm_funcs = {
	.board_get_flash_slot = omap5evm_get_flash_slot,
	.board_set_flash_slot = omap5evm_set_flash_slot,
	.board_ddr_init = omap5evm_ddr_init,
	.board_mux_init = omap5evm_mux_init,
	.board_smartio_init = omap5evm_smartio_init,
	.board_user_fastboot_request = omap5evm_check_fastboot,
	.board_late_init = omap5evm_late_init,
	.board_get_part_tbl = omap5evm_get_partition,
	.board_scale_vcores = omap5evm_scale_cores,
	.board_gpmc_init = omap5evm_gpmc_init,
	.board_prcm_init = omap5evm_prcm_init,
	.board_storage_init = omap5evm_storage_init,
	.board_read_sw_revision = omap5evm_read_sw_revision,
	.board_configure_pwm_mode = omap5evm_configure_pwm_mode,
};

void* init_board_funcs(void)
{
	return &omap5evm_funcs;
}
