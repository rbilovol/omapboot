/*
 * Copyright (C) 2011 The Android Open Source Project
 * All rights reserved.
 *
 * Copyright(c) 2012 Texas Instruments.
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

#include <mux.h>
#include <hw.h>

static struct partition partitions[] = {
	{ "-", 128 },
	{ "xloader", 128 },
	{ "bootloader", 256 },
	{ "-", 512 },
	{ "recovery", 8*1024 },
	{ "boot", 8*1024 },
	{ "system", 512*1024 },
	{ "cache", 256*1024 },
	{ "userdata", 0},
	{ NULL, 0 },
};

static struct partition * panda_get_partition(void)
{
	return partitions;
}

static void panda_mux_init(void)
{
	setup_core(CONTROL_PADCONF_GPMC_AD0, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* sdmmc2_dat0 */
	setup_core(CONTROL_PADCONF_GPMC_AD1, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* sdmmc2_dat1 */
	setup_core(CONTROL_PADCONF_GPMC_AD2, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* sdmmc2_dat2 */
	setup_core(CONTROL_PADCONF_GPMC_AD3, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* sdmmc2_dat3 */
	setup_core(CONTROL_PADCONF_GPMC_AD4, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* sdmmc2_dat4 */
	setup_core(CONTROL_PADCONF_GPMC_AD5, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* sdmmc2_dat5 */
	setup_core(CONTROL_PADCONF_GPMC_AD6, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* sdmmc2_dat6 */
	setup_core(CONTROL_PADCONF_GPMC_AD7, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* sdmmc2_dat7 */
	setup_core(CONTROL_PADCONF_GPMC_AD8, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M3));  /* gpio_32 */
	setup_core(CONTROL_PADCONF_GPMC_AD9, (PTU | IEN | M3));  /* gpio_33 */
	setup_core(CONTROL_PADCONF_GPMC_AD10, (PTU | IEN | M3));  /* gpio_34 */
	setup_core(CONTROL_PADCONF_GPMC_AD11, (PTU | IEN | M3));  /* gpio_35 */
	setup_core(CONTROL_PADCONF_GPMC_AD12, (PTU | IEN | M3));  /* gpio_36 */
	setup_core(CONTROL_PADCONF_GPMC_AD13, (PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3));  /* gpio_37 */
	setup_core(CONTROL_PADCONF_GPMC_AD14, (PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3));  /* gpio_38 */
	setup_core(CONTROL_PADCONF_GPMC_AD15, (PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3));  /* gpio_39 */
	setup_core(CONTROL_PADCONF_GPMC_A16, (M3));  /* gpio_40 */
	setup_core(CONTROL_PADCONF_GPMC_A17, (PTD | M3));  /* gpio_41 */
	setup_core(CONTROL_PADCONF_GPMC_A18, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* kpd_row6 */
	setup_core(CONTROL_PADCONF_GPMC_A19, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* kpd_row7 */
	setup_core(CONTROL_PADCONF_GPMC_A20, (IEN | M3));  /* gpio_44 */
	setup_core(CONTROL_PADCONF_GPMC_A21, (M3));  /* gpio_45 */
	setup_core(CONTROL_PADCONF_GPMC_A22, (M3));  /* gpio_46 */
	setup_core(CONTROL_PADCONF_GPMC_A23, (OFF_EN | OFF_PD | OFF_IN | M1));  /* kpd_col7 */
	setup_core(CONTROL_PADCONF_GPMC_A24, (PTD | M3));  /* gpio_48 */
	setup_core(CONTROL_PADCONF_GPMC_A25, (PTD | M3));  /* gpio_49 */
	setup_core(CONTROL_PADCONF_GPMC_NCS0, (M3));  /* gpio_50 */
	setup_core(CONTROL_PADCONF_GPMC_NCS1, (IEN | M3));  /* gpio_51 */
	setup_core(CONTROL_PADCONF_GPMC_NCS2, (IEN | M3));  /* gpio_52 */
	setup_core(CONTROL_PADCONF_GPMC_NCS3, (IEN | M3));  /* gpio_53 */
	setup_core(CONTROL_PADCONF_GPMC_NWP, (M3));  /* gpio_54 */
	setup_core(CONTROL_PADCONF_GPMC_CLK, (PTD | M3));  /* gpio_55 */
	setup_core(CONTROL_PADCONF_GPMC_NADV_ALE, (M3));  /* gpio_56 */
	setup_core(CONTROL_PADCONF_GPMC_NOE, (PTU | IEN | OFF_EN | OFF_OUT_PTD | M1));  /* sdmmc2_clk */
	setup_core(CONTROL_PADCONF_GPMC_NWE, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* sdmmc2_cmd */
	setup_core(CONTROL_PADCONF_GPMC_NBE0_CLE, (M3));  /* gpio_59 */
	setup_core(CONTROL_PADCONF_GPMC_NBE1, (PTD | M3));  /* gpio_60 */
	setup_core(CONTROL_PADCONF_GPMC_WAIT0, (PTU | IEN | M3));  /* gpio_61 */
	setup_core(CONTROL_PADCONF_GPMC_WAIT1, (PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3));  /* gpio_62 */
	setup_core(CONTROL_PADCONF_C2C_DATA11, (PTD | M3));  /* gpio_100 */
	setup_core(CONTROL_PADCONF_C2C_DATA12, (PTU | IEN | M3));  /* gpio_101 */
	setup_core(CONTROL_PADCONF_C2C_DATA13, (PTD | M3));  /* gpio_102 */
	setup_core(CONTROL_PADCONF_C2C_DATA14, ( M1));  /* dsi2_te0 */
	setup_core(CONTROL_PADCONF_C2C_DATA15, (PTD | M3));  /* gpio_104 */
	setup_core(CONTROL_PADCONF_HDMI_HPD, (M0));  /* hdmi_hpd */
	setup_core(CONTROL_PADCONF_HDMI_CEC, (M0));  /* hdmi_cec */
	setup_core(CONTROL_PADCONF_HDMI_DDC_SCL, (PTU | M0));  /* hdmi_ddc_scl */
	setup_core(CONTROL_PADCONF_HDMI_DDC_SDA, (PTU | IEN | M0));  /* hdmi_ddc_sda */
	setup_core(CONTROL_PADCONF_CSI21_DX0, (IEN | M0));  /* csi21_dx0 */
	setup_core(CONTROL_PADCONF_CSI21_DY0, (IEN | M0));  /* csi21_dy0 */
	setup_core(CONTROL_PADCONF_CSI21_DX1, (IEN | M0));  /* csi21_dx1 */
	setup_core(CONTROL_PADCONF_CSI21_DY1, (IEN | M0));  /* csi21_dy1 */
	setup_core(CONTROL_PADCONF_CSI21_DX2, (IEN | M0));  /* csi21_dx2 */
	setup_core(CONTROL_PADCONF_CSI21_DY2, (IEN | M0));  /* csi21_dy2 */
	setup_core(CONTROL_PADCONF_CSI21_DX3, (PTD | M7));  /* csi21_dx3 */
	setup_core(CONTROL_PADCONF_CSI21_DY3, (PTD | M7));  /* csi21_dy3 */
	setup_core(CONTROL_PADCONF_CSI21_DX4, (PTD | OFF_EN | OFF_PD | OFF_IN | M7));  /* csi21_dx4 */
	setup_core(CONTROL_PADCONF_CSI21_DY4, (PTD | OFF_EN | OFF_PD | OFF_IN | M7));  /* csi21_dy4 */
	setup_core(CONTROL_PADCONF_CSI22_DX0, (IEN | M0));  /* csi22_dx0 */
	setup_core(CONTROL_PADCONF_CSI22_DY0, (IEN | M0));  /* csi22_dy0 */
	setup_core(CONTROL_PADCONF_CSI22_DX1, (IEN | M0));  /* csi22_dx1 */
	setup_core(CONTROL_PADCONF_CSI22_DY1, (IEN | M0));  /* csi22_dy1 */
	setup_core(CONTROL_PADCONF_CAM_SHUTTER, (OFF_EN | OFF_PD | OFF_OUT_PTD | M0));  /* cam_shutter */
	setup_core(CONTROL_PADCONF_CAM_STROBE, (OFF_EN | OFF_PD | OFF_OUT_PTD | M0));  /* cam_strobe */
	setup_core(CONTROL_PADCONF_CAM_GLOBALRESET, (PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3));  /* gpio_83 */
	setup_core(CONTROL_PADCONF_USBB1_ULPITLL_CLK, (PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M4));  /* usbb1_ulpiphy_clk */
	setup_core(CONTROL_PADCONF_USBB1_ULPITLL_STP, (OFF_EN | OFF_OUT_PTD | M4));  /* usbb1_ulpiphy_stp */
	setup_core(CONTROL_PADCONF_USBB1_ULPITLL_DIR, (IEN | OFF_EN | OFF_PD | OFF_IN | M4));  /* usbb1_ulpiphy_dir */
	setup_core(CONTROL_PADCONF_USBB1_ULPITLL_NXT, (IEN | OFF_EN | OFF_PD | OFF_IN | M4));  /* usbb1_ulpiphy_nxt */
	setup_core(CONTROL_PADCONF_USBB1_ULPITLL_DAT0, (IEN | OFF_EN | OFF_PD | OFF_IN | M4));  /* usbb1_ulpiphy_dat0 */
	setup_core(CONTROL_PADCONF_USBB1_ULPITLL_DAT1, (IEN | OFF_EN | OFF_PD | OFF_IN | M4));  /* usbb1_ulpiphy_dat1 */
	setup_core(CONTROL_PADCONF_USBB1_ULPITLL_DAT2, (IEN | OFF_EN | OFF_PD | OFF_IN | M4));  /* usbb1_ulpiphy_dat2 */
	setup_core(CONTROL_PADCONF_USBB1_ULPITLL_DAT3, (IEN | OFF_EN | OFF_PD | OFF_IN | M4));  /* usbb1_ulpiphy_dat3 */
	setup_core(CONTROL_PADCONF_USBB1_ULPITLL_DAT4, (IEN | OFF_EN | OFF_PD | OFF_IN | M4));  /* usbb1_ulpiphy_dat4 */
	setup_core(CONTROL_PADCONF_USBB1_ULPITLL_DAT5, (IEN | OFF_EN | OFF_PD | OFF_IN | M4));  /* usbb1_ulpiphy_dat5 */
	setup_core(CONTROL_PADCONF_USBB1_ULPITLL_DAT6, (IEN | OFF_EN | OFF_PD | OFF_IN | M4));  /* usbb1_ulpiphy_dat6 */
	setup_core(CONTROL_PADCONF_USBB1_ULPITLL_DAT7, (IEN | OFF_EN | OFF_PD | OFF_IN | M4));  /* usbb1_ulpiphy_dat7 */
	setup_core(CONTROL_PADCONF_USBB1_HSIC_DATA, (IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* usbb1_hsic_data */
	setup_core(CONTROL_PADCONF_USBB1_HSIC_STROBE, (IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* usbb1_hsic_strobe */
	setup_core(CONTROL_PADCONF_USBC1_ICUSB_DP, (IEN | M0));  /* usbc1_icusb_dp */
	setup_core(CONTROL_PADCONF_USBC1_ICUSB_DM, (IEN | M0));  /* usbc1_icusb_dm */
	setup_core(CONTROL_PADCONF_SDMMC1_CLK, (PTU | OFF_EN | OFF_OUT_PTD | M0));  /* sdmmc1_clk */
	setup_core(CONTROL_PADCONF_SDMMC1_CMD, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* sdmmc1_cmd */
	setup_core(CONTROL_PADCONF_SDMMC1_DAT0, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* sdmmc1_dat0 */
	setup_core(CONTROL_PADCONF_SDMMC1_DAT1, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* sdmmc1_dat1 */
	setup_core(CONTROL_PADCONF_SDMMC1_DAT2, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* sdmmc1_dat2 */
	setup_core(CONTROL_PADCONF_SDMMC1_DAT3, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* sdmmc1_dat3 */
	setup_core(CONTROL_PADCONF_SDMMC1_DAT4, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* sdmmc1_dat4 */
	setup_core(CONTROL_PADCONF_SDMMC1_DAT5, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* sdmmc1_dat5 */
	setup_core(CONTROL_PADCONF_SDMMC1_DAT6, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* sdmmc1_dat6 */
	setup_core(CONTROL_PADCONF_SDMMC1_DAT7, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* sdmmc1_dat7 */
	setup_core(CONTROL_PADCONF_ABE_MCBSP2_CLKX, (IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* abe_mcbsp2_clkx */
	setup_core(CONTROL_PADCONF_ABE_MCBSP2_DR, (IEN | OFF_EN | OFF_OUT_PTD | M0));  /* abe_mcbsp2_dr */
	setup_core(CONTROL_PADCONF_ABE_MCBSP2_DX, (OFF_EN | OFF_OUT_PTD | M0));  /* abe_mcbsp2_dx */
	setup_core(CONTROL_PADCONF_ABE_MCBSP2_FSX, (IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* abe_mcbsp2_fsx */
	setup_core(CONTROL_PADCONF_ABE_MCBSP1_CLKX, (IEN | M1));  /* abe_slimbus1_clock */
	setup_core(CONTROL_PADCONF_ABE_MCBSP1_DR, (IEN | M1));  /* abe_slimbus1_data */
	setup_core(CONTROL_PADCONF_ABE_MCBSP1_DX, (OFF_EN | OFF_OUT_PTD | M0));  /* abe_mcbsp1_dx */
	setup_core(CONTROL_PADCONF_ABE_MCBSP1_FSX, (IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* abe_mcbsp1_fsx */
	setup_core(CONTROL_PADCONF_ABE_PDM_UL_DATA, (PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* abe_pdm_ul_data */
	setup_core(CONTROL_PADCONF_ABE_PDM_DL_DATA, (PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* abe_pdm_dl_data */
	setup_core(CONTROL_PADCONF_ABE_PDM_FRAME, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* abe_pdm_frame */
	setup_core(CONTROL_PADCONF_ABE_PDM_LB_CLK, (PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* abe_pdm_lb_clk */
	setup_core(CONTROL_PADCONF_ABE_CLKS, (PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* abe_clks */
	setup_core(CONTROL_PADCONF_ABE_DMIC_CLK1, (M0));  /* abe_dmic_clk1 */
	setup_core(CONTROL_PADCONF_ABE_DMIC_DIN1, (IEN | M0));  /* abe_dmic_din1 */
	setup_core(CONTROL_PADCONF_ABE_DMIC_DIN2,   (PTU | IEN | M3));
	setup_core(CONTROL_PADCONF_ABE_DMIC_DIN3, (IEN | M0));  /* abe_dmic_din3 */
	setup_core(CONTROL_PADCONF_UART2_CTS, (PTU | IEN | M0));  /* uart2_cts */
	setup_core(CONTROL_PADCONF_UART2_RTS, (M0));  /* uart2_rts */
	setup_core(CONTROL_PADCONF_UART2_RX, (PTU | IEN | M0));  /* uart2_rx */
	setup_core(CONTROL_PADCONF_UART2_TX, (M0));  /* uart2_tx */
	setup_core(CONTROL_PADCONF_HDQ_SIO, (M3));  /* gpio_127 */
	setup_core(CONTROL_PADCONF_I2C1_SCL, (PTU | IEN | M0));  /* i2c1_scl */
	setup_core(CONTROL_PADCONF_I2C1_SDA, (PTU | IEN | M0));  /* i2c1_sda */
	setup_core(CONTROL_PADCONF_I2C2_SCL, (PTU | IEN | M0));  /* i2c2_scl */
	setup_core(CONTROL_PADCONF_I2C2_SDA, (PTU | IEN | M0));  /* i2c2_sda */
	setup_core(CONTROL_PADCONF_I2C3_SCL, (PTU | IEN | M0));  /* i2c3_scl */
	setup_core(CONTROL_PADCONF_I2C3_SDA, (PTU | IEN | M0));  /* i2c3_sda */
	setup_core(CONTROL_PADCONF_I2C4_SCL, (PTU | IEN | M0));  /* i2c4_scl */
	setup_core(CONTROL_PADCONF_I2C4_SDA, (PTU | IEN | M0));  /* i2c4_sda */
	setup_core(CONTROL_PADCONF_MCSPI1_CLK, (IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* mcspi1_clk */
	setup_core(CONTROL_PADCONF_MCSPI1_SOMI, (IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* mcspi1_somi */
	setup_core(CONTROL_PADCONF_MCSPI1_SIMO, (IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* mcspi1_simo */
	setup_core(CONTROL_PADCONF_MCSPI1_CS0, (PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* mcspi1_cs0 */
	setup_core(CONTROL_PADCONF_MCSPI1_CS1, (PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M3));  /* mcspi1_cs1 */
	setup_core(CONTROL_PADCONF_MCSPI1_CS2, (PTU | OFF_EN | OFF_OUT_PTU | M3));  /* gpio_139 */
	setup_core(CONTROL_PADCONF_MCSPI1_CS3, (PTU | IEN | M3));  /* gpio_140 */
	setup_core(CONTROL_PADCONF_UART3_CTS_RCTX, (PTU | IEN | M0));  /* uart3_tx */
	setup_core(CONTROL_PADCONF_UART3_RTS_SD, (M0));  /* uart3_rts_sd */
	setup_core(CONTROL_PADCONF_UART3_RX_IRRX, (IEN | M0));  /* uart3_rx */
	setup_core(CONTROL_PADCONF_UART3_TX_IRTX, (M0));  /* uart3_tx */
	setup_core(CONTROL_PADCONF_SDMMC5_CLK, (PTU | IEN | OFF_EN | OFF_OUT_PTD | M0));  /* sdmmc5_clk */
	setup_core(CONTROL_PADCONF_SDMMC5_CMD, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* sdmmc5_cmd */
	setup_core(CONTROL_PADCONF_SDMMC5_DAT0, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* sdmmc5_dat0 */
	setup_core(CONTROL_PADCONF_SDMMC5_DAT1, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* sdmmc5_dat1 */
	setup_core(CONTROL_PADCONF_SDMMC5_DAT2, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* sdmmc5_dat2 */
	setup_core(CONTROL_PADCONF_SDMMC5_DAT3, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* sdmmc5_dat3 */
	setup_core(CONTROL_PADCONF_MCSPI4_CLK, (IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* mcspi4_clk */
	setup_core(CONTROL_PADCONF_MCSPI4_SIMO, (IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* mcspi4_simo */
	setup_core(CONTROL_PADCONF_MCSPI4_SOMI, (IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* mcspi4_somi */
	setup_core(CONTROL_PADCONF_MCSPI4_CS0, (PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* mcspi4_cs0 */
	setup_core(CONTROL_PADCONF_UART4_RX, (IEN | M0));  /* uart4_rx */
	setup_core(CONTROL_PADCONF_UART4_TX, (M0));  /* uart4_tx */
	setup_core(CONTROL_PADCONF_USBB2_ULPITLL_CLK, (IEN | M3));  /* gpio_157 */
	setup_core(CONTROL_PADCONF_USBB2_ULPITLL_STP, (IEN | M5));  /* dispc2_data23 */
	setup_core(CONTROL_PADCONF_USBB2_ULPITLL_DIR, (IEN | M5));  /* dispc2_data22 */
	setup_core(CONTROL_PADCONF_USBB2_ULPITLL_NXT, (IEN | M5));  /* dispc2_data21 */
	setup_core(CONTROL_PADCONF_USBB2_ULPITLL_DAT0, (IEN | M5));  /* dispc2_data20 */
	setup_core(CONTROL_PADCONF_USBB2_ULPITLL_DAT1, (IEN | M5));  /* dispc2_data19 */
	setup_core(CONTROL_PADCONF_USBB2_ULPITLL_DAT2, (IEN | M5));  /* dispc2_data18 */
	setup_core(CONTROL_PADCONF_USBB2_ULPITLL_DAT3, (IEN | M5));  /* dispc2_data15 */
	setup_core(CONTROL_PADCONF_USBB2_ULPITLL_DAT4, (IEN | M5));  /* dispc2_data14 */
	setup_core(CONTROL_PADCONF_USBB2_ULPITLL_DAT5, (IEN | M5));  /* dispc2_data13 */
	setup_core(CONTROL_PADCONF_USBB2_ULPITLL_DAT6, (IEN | M5));  /* dispc2_data12 */
	setup_core(CONTROL_PADCONF_USBB2_ULPITLL_DAT7, (IEN | M5));  /* dispc2_data11 */
	setup_core(CONTROL_PADCONF_USBB2_HSIC_DATA, (PTD | OFF_EN | OFF_OUT_PTU | M3));  /* gpio_169 */
	setup_core(CONTROL_PADCONF_USBB2_HSIC_STROBE, (PTD | OFF_EN | OFF_OUT_PTU | M3));  /* gpio_170 */
	setup_core(CONTROL_PADCONF_UNIPRO_TX0, (PTD | IEN | M3));  /* gpio_171 */
	setup_core(CONTROL_PADCONF_UNIPRO_TY0, (OFF_EN | OFF_PD | OFF_IN | M1));  /* kpd_col1 */
	setup_core(CONTROL_PADCONF_UNIPRO_TX1, (OFF_EN | OFF_PD | OFF_IN | M1));  /* kpd_col2 */
	setup_core(CONTROL_PADCONF_UNIPRO_TY1, (OFF_EN | OFF_PD | OFF_IN | M1));  /* kpd_col3 */
	setup_core(CONTROL_PADCONF_UNIPRO_TX2, (PTU | IEN | M3));  /* gpio_0 */
	setup_core(CONTROL_PADCONF_UNIPRO_TY2, (PTU | IEN | M3));  /* gpio_1 */
	setup_core(CONTROL_PADCONF_UNIPRO_RX0, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* kpd_row0 */
	setup_core(CONTROL_PADCONF_UNIPRO_RY0, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* kpd_row1 */
	setup_core(CONTROL_PADCONF_UNIPRO_RX1, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* kpd_row2 */
	setup_core(CONTROL_PADCONF_UNIPRO_RY1, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* kpd_row3 */
	setup_core(CONTROL_PADCONF_UNIPRO_RX2, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* kpd_row4 */
	setup_core(CONTROL_PADCONF_UNIPRO_RY2, (PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1));  /* kpd_row5 */
	setup_core(CONTROL_PADCONF_USBA0_OTG_CE, (PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M0));  /* usba0_otg_ce */
	setup_core(CONTROL_PADCONF_USBA0_OTG_DP, (IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* usba0_otg_dp */
	setup_core(CONTROL_PADCONF_USBA0_OTG_DM, (IEN | OFF_EN | OFF_PD | OFF_IN | M0));  /* usba0_otg_dm */
	setup_core(CONTROL_PADCONF_FREF_CLK1_OUT, (M0));  /* fref_clk1_out */
	setup_core(CONTROL_PADCONF_FREF_CLK2_OUT, (PTU | IEN | M3));  /* gpio_182 */
	setup_core(CONTROL_PADCONF_SYS_NIRQ1, (PTU | IEN | M0));  /* sys_nirq1 */
	setup_core(CONTROL_PADCONF_SYS_NIRQ2, (PTU | IEN | M0));  /* sys_nirq2 */
	setup_core(CONTROL_PADCONF_SYS_BOOT0, (PTU | IEN | M3));  /* gpio_184 */
	setup_core(CONTROL_PADCONF_SYS_BOOT1, (M3));  /* gpio_185 */
	setup_core(CONTROL_PADCONF_SYS_BOOT2, (PTD | IEN | M3));  /* gpio_186 */
	setup_core(CONTROL_PADCONF_SYS_BOOT3, (M3));  /* gpio_187 */
	setup_core(CONTROL_PADCONF_SYS_BOOT4, (M3));  /* gpio_188 */
	setup_core(CONTROL_PADCONF_SYS_BOOT5, (PTD | IEN | M3));  /* gpio_189 */
	setup_core(CONTROL_PADCONF_DPM_EMU0, (IEN | M0));  /* dpm_emu0 */
	setup_core(CONTROL_PADCONF_DPM_EMU1, (IEN | M0));  /* dpm_emu1 */
	setup_core(CONTROL_PADCONF_DPM_EMU2, (IEN | M0));  /* dpm_emu2 */
	setup_core(CONTROL_PADCONF_DPM_EMU3, (IEN | M5));  /* dispc2_data10 */
	setup_core(CONTROL_PADCONF_DPM_EMU4, (IEN | M5));  /* dispc2_data9 */
	setup_core(CONTROL_PADCONF_DPM_EMU5, (IEN | M5));  /* dispc2_data16 */
	setup_core(CONTROL_PADCONF_DPM_EMU6, (IEN | M5));  /* dispc2_data17 */
	setup_core(CONTROL_PADCONF_DPM_EMU7, (IEN | M5));  /* dispc2_hsync */
	setup_core(CONTROL_PADCONF_DPM_EMU8, (IEN | M5));  /* dispc2_pclk */
	setup_core(CONTROL_PADCONF_DPM_EMU9, (IEN | M5));  /* dispc2_vsync */
	setup_core(CONTROL_PADCONF_DPM_EMU10, (IEN | M5));  /* dispc2_de */
	setup_core(CONTROL_PADCONF_DPM_EMU11, (IEN | M5));  /* dispc2_data8 */
	setup_core(CONTROL_PADCONF_DPM_EMU12, (IEN | M5));  /* dispc2_data7 */
	setup_core(CONTROL_PADCONF_DPM_EMU13, (IEN | M5));  /* dispc2_data6 */
	setup_core(CONTROL_PADCONF_DPM_EMU14, (IEN | M5));  /* dispc2_data5 */
	setup_core(CONTROL_PADCONF_DPM_EMU15, (IEN | M5));  /* dispc2_data4 */
	setup_core(CONTROL_PADCONF_DPM_EMU16, (M3));  /* gpio_27 */
	setup_core(CONTROL_PADCONF_DPM_EMU17, (IEN | M5));  /* dispc2_data2 */
	setup_core(CONTROL_PADCONF_DPM_EMU18, (IEN | M5));  /* dispc2_data1 */
	setup_core(CONTROL_PADCONF_DPM_EMU19, (IEN | M5));  /* dispc2_data0 */
	setup_wakeup(CONTROL_WKUP_PAD0_SIM_IO, (IEN | M0));  /* sim_io */
	setup_wakeup(CONTROL_WKUP_PAD1_SIM_CLK, (M0));  /* sim_clk */
	setup_wakeup(CONTROL_WKUP_PAD0_SIM_RESET, (M0));  /* sim_reset */
	setup_wakeup(CONTROL_WKUP_PAD1_SIM_CD, (PTU | IEN | M0));  /* sim_cd */
	setup_wakeup(CONTROL_WKUP_PAD0_SIM_PWRCTRL, (M0));  /* sim_pwrctrl */
	setup_wakeup(CONTROL_WKUP_PAD1_SR_SCL, (PTU | IEN | M0));  /* sr_scl */
	setup_wakeup(CONTROL_WKUP_PAD0_SR_SDA, (PTU | IEN | M0));  /* sr_sda */
	setup_wakeup(CONTROL_WKUP_PAD1_FREF_XTAL_IN, (M0));  /* # */
	setup_wakeup(CONTROL_WKUP_PAD0_FREF_SLICER_IN, (M0));  /* fref_slicer_in */
	setup_wakeup(CONTROL_WKUP_PAD1_FREF_CLK_IOREQ, (M0));  /* fref_clk_ioreq */
	setup_wakeup(CONTROL_WKUP_PAD0_FREF_CLK0_OUT, (M2));  /* sys_drm_msecure */
	setup_wakeup(CONTROL_WKUP_PAD1_FREF_CLK3_REQ, (M3));  /* gpio_wk30 */
	setup_wakeup(CONTROL_WKUP_PAD0_FREF_CLK3_OUT, (M0));  /* fref_clk3_out */
	setup_wakeup(CONTROL_WKUP_PAD1_FREF_CLK4_REQ, (PTU | IEN | M0));  /* # */
	setup_wakeup(CONTROL_WKUP_PAD0_FREF_CLK4_OUT, (M0));  /* # */
	setup_wakeup(CONTROL_WKUP_PAD1_SYS_32K, (IEN | M0));  /* sys_32k */
	setup_wakeup(CONTROL_WKUP_PAD0_SYS_NRESPWRON, (M0));  /* sys_nrespwron */
	setup_wakeup(CONTROL_WKUP_PAD1_SYS_NRESWARM, (M0));  /* sys_nreswarm */
	setup_wakeup(CONTROL_WKUP_PAD0_SYS_PWR_REQ, (PTU | M0));  /* sys_pwr_req */
	setup_wakeup(CONTROL_WKUP_PAD1_SYS_PWRON_RESET, (M3));  /* gpio_wk29 */
	setup_wakeup(CONTROL_WKUP_PAD0_SYS_BOOT6, (IEN | M3));  /* gpio_wk9 */
	setup_wakeup(CONTROL_WKUP_PAD1_SYS_BOOT7, (IEN | M3));  /* gpio_wk10 */
	setup_wakeup(CONTROL_WKUP_PAD1_FREF_CLK3_REQ, (M3)); /* gpio_wk30 */
	setup_wakeup(CONTROL_WKUP_PAD1_FREF_CLK4_REQ, (M3)); /* gpio_wk7 */
	setup_wakeup(CONTROL_WKUP_PAD0_FREF_CLK4_OUT, (M3)); /* gpio_wk8 */
}


static struct ddr_regs elpida2G_400_mhz_2cs = {
	/* tRRD changed from 10ns to 12.5ns because of the tFAW requirement*/
	.tim1		= 0x10eb0662,
	.tim2		= 0x20370dd2,
	.tim3		= 0x00b1c33f,
	.phy_ctrl_1	= 0x849FF408,
	.ref_ctrl	= 0x00000618,
	.config_init	= 0x80000eb9,
	.config_final	= 0x80001ab9,
	.zq_config	= 0xD00b3215,
	.mr1		= 0x83,
	.mr2		= 0x4
};

static struct ddr_regs elpida4G_400_mhz_1cs = {
	.tim1		= 0x10eb0662,
	.tim2		= 0x20370dd2,
	.tim3		= 0x00b1c33f,
	.phy_ctrl_1	= 0x449FF408,
	.ref_ctrl	= 0x00000618,
	.config_init	= 0x80800eb2,
	.config_final	= 0x80801ab2,
	.zq_config	= 0xd00b3215,
	.mr1		= 0x83,
	.mr2		= 0x4
};

static void panda_ddr_init(struct proc_specific_functions *proc_ops)
{
	const struct ddr_regs *ddr_regs = 0;
	int omap_rev = OMAP_REV_INVALID;
	/* 1GB, 128B interleaved */
	writel(0x80640300, DMM_BASE + DMM_LISA_MAP_0);
	writel(0x00000000, DMM_BASE + DMM_LISA_MAP_2);
	writel(0xFF020100, DMM_BASE + DMM_LISA_MAP_3);

	omap_rev = proc_ops->proc_get_proc_id();
	switch (omap_rev) {
	case OMAP_4430_ES2_DOT_2:
	case OMAP_4430_ES2_DOT_3:
		ddr_regs = &elpida2G_400_mhz_2cs;
		break;
	case OMAP_4460_ES1_DOT_0:
	case OMAP_4460_ES1_DOT_1:
		writel(0x80640300, MA_BASE + DMM_LISA_MAP_0);
		elpida2G_400_mhz_2cs.phy_ctrl_1	= 0x449FF408;
		ddr_regs = &elpida2G_400_mhz_2cs;
		break;
	case OMAP_4470_ES1_DOT_0:
		writel(0x80640300, MA_BASE + DMM_LISA_MAP_3);
		ddr_regs = &elpida4G_400_mhz_1cs;
		break;
	case OMAP_REV_INVALID:
	default:
		printf("%s: unsupported OMAP4 revision %d",
					__func__, omap_rev);
	}

	omap4_ddr_init(ddr_regs, ddr_regs);
}

static int panda_check_fastboot(void)
{
	return 0;
}

static u8 panda_get_flash_slot()
{
	return DEVICE_SDCARD;
}


static int panda_storage_init(u8 dev,
				struct storage_specific_functions *storage_ops)
{
	int ret;

	ret = storage_ops->init(dev);
	if (ret)
		printf("Unable to init storage device\n");

	return ret;
}

struct storage_specific_functions *panda_set_flash_slot(u8 dev,
				struct proc_specific_functions *proc_ops,
				struct storage_specific_functions *storage_ops)
{
	int ret = 0;
	char buf[DEV_STR_LENGTH];

	if ((dev == DEVICE_SDCARD) || (!storage_ops))
		storage_ops = init_rom_mmc_funcs
					(proc_ops->proc_get_proc_id(), dev);
	else
		return NULL;

	if (storage_ops != NULL) {
		ret = panda_storage_init(dev, storage_ops);
		if (ret != 0) {
			dev_to_devstr(dev, buf);
			printf("Unable to set flash slot: %s\n", buf);
		}
	}

	return storage_ops;
}

static void panda_scale_cores(void)
{
	/* Use default OMAP voltage */
	scale_vcores();
}

static void panda_gpmc_init(void)
{
	/* Use default OMAP gpmc init function */
	gpmc_init();
}

static void panda_prcm_init(struct proc_specific_functions *proc_ops)
{
	/* Use default OMAP gpmc init function */
	prcm_init(proc_ops);
}

int panda_usb_len_request(struct usb_specific_functions *usb_ops,
				void *data, unsigned len)
{
	return usb_ops->usb_read(usb_ops->usb, data, len);
}

static struct board_specific_functions panda_funcs = {
	.board_get_flash_slot = panda_get_flash_slot,
	.board_set_flash_slot = panda_set_flash_slot,
	.board_mux_init = panda_mux_init,
	.board_ddr_init = panda_ddr_init,
	.board_user_fastboot_request = panda_check_fastboot,
	.board_get_part_tbl = panda_get_partition,
	.board_scale_vcores = panda_scale_cores,
	.board_gpmc_init = panda_gpmc_init,
	.board_prcm_init = panda_prcm_init,
	.board_storage_init = panda_storage_init,
	.board_fastboot_size_request = panda_usb_len_request
};

void* init_board_funcs(void)
{
	return &panda_funcs;
}
