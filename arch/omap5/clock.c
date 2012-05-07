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
#include <omap5/hw.h>
#include <omap5/clock.h>
#include <config.h>

typedef struct dpll_param dpll_param;

/* OPP NOM */
struct dpll_param core_dpll_params[2] = {
	{665, 11,  2,  5,  8,  4, 62,  5,   5,  7},	/* 19.2 MHz */
	{665, 23,  2,  5,  8,  4, 62,  5,  5,   7}	/* 38.4 MHz */
};

/* OPP NOM */
struct dpll_param usb_dpll_params[2] = {
	{400, 7, 2, -1, -1, -1, -1, -1, -1, -1},	/* 19.2 MHz */
	{400, 15, 2, -1, -1, -1, -1, -1, -1, -1}	/* 38.4 MHz */
};

/* OPP NOM */
struct dpll_param iva_dpll_params[2] = {
	{1881, 30, -1, -1,  5,  6,  -1, -1, -1, -1},	/* 19.2 MHz */
	{1972, 64, -1, -1,  5,  6,  -1, -1, -1, -1}	/* 38.4 MHz */
};

/* OPP NOM */
struct dpll_param abe_dpll_params = {
	750, 0, 1, 1, -1, -1, -1, -1, -1, -1
};

static void setup_clocks(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(omap5_clocks); i++) {
		switch (omap5_clocks[i].ctrl) {
		case WRITEL:
			writel(omap5_clocks[i].value, omap5_clocks[i].ad);
			break;
		case MODIFY:
		case MODIFY_WAIT:
			set_modify(omap5_clocks[i].ad, omap5_clocks[i].mask,
							omap5_clocks[i].value);
			if (omap5_clocks[i].ctrl == MODIFY_WAIT) {
				if (!wait_on_value(BIT(16) | BIT(17), 0,
						omap5_clocks[i].ad, LDELAY))
					printf("Clock enable failed for 0x%p\n",
						omap5_clocks[i].ad);
					return;
			}
			break;
		default:
			break;
		}
	}
}


void configure_core_dpll(dpll_param *dpll_param_p)
{
	/* Unlock the CORE dpll */
	sr32(CM_CLKMODE_DPLL_CORE, 0, 4, IDLE_BYPASS_FAST_RELOCK_MODE);
	if (!wait_on_value(BIT(0), 0, CM_IDLEST_DPLL_CORE, LDELAY)) {
		/* do nothing */
	}

	sr32(CM_CLKSEL_DPLL_CORE, 8, 11, dpll_param_p->m);
	sr32(CM_CLKSEL_DPLL_CORE, 0, 7, dpll_param_p->n);


	/* Setup post-dividers */
	if (dpll_param_p->m2 >= 0)
		writel(dpll_param_p->m2, CM_DIV_M2_DPLL_CORE);
	if (dpll_param_p->m3 >= 0)
		writel(dpll_param_p->m3, CM_DIV_M3_DPLL_CORE);
	if (dpll_param_p->h11 >= 0)
		writel(dpll_param_p->h11, CM_DIV_H11_DPLL_CORE);
	if (dpll_param_p->h12 >= 0)
		writel(dpll_param_p->h12, CM_DIV_H12_DPLL_CORE);
	if (dpll_param_p->h13 >= 0)
		writel(dpll_param_p->h13, CM_DIV_H13_DPLL_CORE);
	if (dpll_param_p->h14 >= 0)
		writel(dpll_param_p->h14, CM_DIV_H14_DPLL_CORE);
	if (dpll_param_p->h22 >= 0)
		writel(dpll_param_p->h22, CM_DIV_H22_DPLL_CORE);
	if (dpll_param_p->h23 >= 0)
		writel(dpll_param_p->h23, CM_DIV_H23_DPLL_CORE);

	return;
}

void configure_per_dpll(void)
{
	/* Put DPLL into bypass mode */
	set_modify(CM_CLKMODE_DPLL_PER, 0x00000007, 0x00000005);

	if (!wait_on_value(BIT(0), 0, CM_IDLEST_DPLL_PER, LDELAY)) {
		/* do nothing */
	}

	writel(0x00000004, CM_DIV_M2_DPLL_PER);
	writel(0x00000003, CM_DIV_M3_DPLL_PER);
	writel(0x00000006, CM_DIV_H11_DPLL_PER);
	writel(0x00000004, CM_DIV_H12_DPLL_PER);
	writel(0x00000000, CM_DIV_H14_DPLL_PER);
	writel(0x00000002, CM_L4PER_CLKSTCTRL);

	set_modify(CM_CLKSEL_DPLL_PER, 0x0007FF7F, 0x00001400);

	/* Put DPLL into lock mode */
	writel(0x00000007, CM_CLKMODE_DPLL_PER);

	/* Wait for DPLL to be locked */
	if (!wait_on_value(BIT(0), 1, CM_IDLEST_DPLL_PER, LDELAY)) {
		/* do nothing */
	}

	writel(0x00000001, CM_EMIF_EMIF1_CLKCTRL);
	writel(0x00000001, CM_EMIF_EMIF2_CLKCTRL);
	writel(0x00000002, CM_L4PER_UART1_CLKCTRL);
	writel(0x00000002, CM_L4PER_UART2_CLKCTRL);
	writel(0x00000002, CM_L4PER_UART3_CLKCTRL);
	writel(0x00000002, CM_L4PER_UART4_CLKCTRL);

	return;
}

void configure_mpu_dpll(void)
{
	/* Put DPLL into bypass mode */
	set_modify(CM_CLKMODE_DPLL_MPU, 0x00000007, 0x00000005);

	if (!wait_on_value(BIT(0), 0, CM_IDLEST_DPLL_MPU, LDELAY)) {
		/* do nothing */
	}

	/* Program DPLL frequency (M and N) */
	set_modify(CM_CLKSEL_DPLL_MPU, 0x0007FF7F, 0x00017708);

	/* Program DPLL_CLKOUT divider (M2 = 1) */
	writel(0x00000001, CM_DIV_M2_DPLL_MPU);

	/* Put DPLL into lock mode */
	writel(0x00000007, CM_CLKMODE_DPLL_MPU);

	/* Wait for DPLL to be locked */
	if (!wait_on_value(BIT(0), 1, CM_IDLEST_DPLL_MPU, LDELAY)) {
		/* do nothing */
	}

	return;
}

void configure_iva_dpll(dpll_param *dpll_param_p)
{
	/* Unlock the IVA dpll */
	sr32(CM_CLKMODE_DPLL_IVA, 0, 3, IDLE_BYPASS_FAST_RELOCK_MODE);
	if (!wait_on_value(BIT(0), 0, CM_IDLEST_DPLL_IVA, LDELAY)) {
		/* do nothing */
	}

	/* CM_BYPCLK_DPLL_IVA = CORE_X2_CLK/2 */
	sr32(CM_BYPCLK_DPLL_IVA, 0, 2, 0x1);

	/* Disable DPLL autoidle */
	sr32(CM_AUTOIDLE_DPLL_IVA, 0, 3, 0x0);

	sr32(CM_CLKSEL_DPLL_IVA, 8, 11, dpll_param_p->m);
	sr32(CM_CLKSEL_DPLL_IVA, 0, 7, dpll_param_p->n);
	sr32(CM_DIV_H11_DPLL_IVA, 0, 6, dpll_param_p->h11);
	sr32(CM_DIV_H11_DPLL_IVA, 9, 1, 0x1);
	sr32(CM_DIV_H12_DPLL_IVA, 0, 6, dpll_param_p->h12);
	sr32(CM_DIV_H12_DPLL_IVA, 9, 1, 0x1);

	/* Lock the iva dpll */
	sr32(CM_CLKMODE_DPLL_IVA, 0, 3, PLL_LOCK);
	if (!wait_on_value(BIT(0), 1, CM_IDLEST_DPLL_IVA, LDELAY)) {
		/* do nothing */
	}

	return;
}

void configure_abe_dpll(dpll_param *dpll_param_p)
{
	u32 value;

	/*
	* We need to enable some additional options to achieve
	* 196.608MHz from 32768 Hz
	*/
	value = readl(CM_CLKMODE_DPLL_ABE);
	writel(value | 0x00000f00, CM_CLKMODE_DPLL_ABE);

	/* Spend 4 REFCLK cycles at each stage */
	value = readl(CM_CLKMODE_DPLL_ABE);
	writel((value & ~(0x7 << 5)) | (0x1 << 5), CM_CLKMODE_DPLL_ABE);

	/* Select the right reference clk */
	value = readl(CM_CLKSEL_ABE_PLL_REF);
	writel((value & ~(0x1)) | (0x1 << 0), CM_CLKSEL_ABE_PLL_REF);

	sr32(CM_CLKMODE_DPLL_ABE, 0, 3, IDLE_BYPASS_FAST_RELOCK_MODE);
	if (!wait_on_value(BIT(1), 0, CM_IDLEST_DPLL_ABE, LDELAY)) {
		/* do nothing */
	}

	sr32(CM_CLKSEL_DPLL_ABE, 8, 11, dpll_param_p->m);
	sr32(CM_CLKSEL_DPLL_ABE, 0, 7, dpll_param_p->n);

	sr32(CM_CLKMODE_DPLL_ABE, 0, 3, PLL_LOCK);

	if (!wait_on_value(BIT(0), 1, CM_IDLEST_DPLL_ABE, LDELAY)) {
		/* do nothing */
	}

	writel(dpll_param_p->m2, CM_DIV_M2_DPLL_ABE);
	writel(dpll_param_p->m3, CM_DIV_M3_DPLL_ABE);

	return;
}

void configure_usb_dpll(dpll_param *dpll_param_p)
{
	u32 num = dpll_param_p->m * (19200000/1000);
	u32 den = (dpll_param_p->n + 1) * 250 * 1000;
	num += den - 1;
	u32 sd_div = num / den;

	sr32(CM_CLKSEL_DPLL_USB, 24, 8, sd_div);

	/* Unlock the USB dpll */
	sr32(CM_CLKMODE_DPLL_USB, 0, 3, IDLE_BYPASS_LOW_POWER_MODE);
	if (!wait_on_value(BIT(0), 0, CM_IDLEST_DPLL_USB, LDELAY)) {
		/* do nothing */
	}

	sr32(CM_CLKSEL_DPLL_USB, 8, 12, dpll_param_p->m);
	sr32(CM_CLKSEL_DPLL_USB, 0, 7, dpll_param_p->n);

	/* lock the dpll */
	writel(0x00000007, CM_CLKMODE_DPLL_USB);
	if (!wait_on_value(BIT(0), 1, CM_IDLEST_DPLL_USB, LDELAY)) {
		/* do nothing */
	}

	/* Setup post-dividers */
	if (dpll_param_p->m2 >= 0)
		writel(dpll_param_p->m2, CM_DIV_M2_DPLL_USB);

	return;
}

void scale_vcores(void)
{
	/* Configure SR I2C Mode */
	writel(0x00000000, PRM_VC_CFG_I2C_MODE);

	/* Configure SR I2C Clock */
	writel(0xfffffffb, PRM_VC_CFG_I2C_CLK);

	/* Configure Palmas TWL6035 */

	/* VDD_CORE - SMPS8_VOLTAGE */
	writel(0x013C3712, PRM_VC_VAL_BYPASS);
	if (!wait_on_value(0x01000000, 0, PRM_VC_VAL_BYPASS, LDELAY)) {
		/* do nothing */
	}
	set_modify(PRM_IRQSTATUS_MPU, 0x00000000, 0x00000000);

	/* VDD_MM:  SMPS45_VOLTAGE */
	writel(0x01382B12, PRM_VC_VAL_BYPASS);
	if (!wait_on_value(0x01000000, 0, PRM_VC_VAL_BYPASS, LDELAY)) {
		/* do nothing */
	}
	set_modify(PRM_IRQSTATUS_MPU, 0x00000000, 0x00000000);

	/* VDD_MPU:  SMPS12_VOLTAGE */
	writel(0x01382312, PRM_VC_VAL_BYPASS);
	if (!wait_on_value(0x01000000, 0, PRM_VC_VAL_BYPASS, LDELAY)) {
		/* do nothing */
	}
	set_modify(PRM_IRQSTATUS_MPU, 0x00000000, 0x00000000);

	return;
}

void prcm_init(void)
{
	configure_core_dpll(&core_dpll_params[0]);

	u32 temp = (CLKSEL_CORE_X2_DIV_1 << CLKSEL_CORE_SHIFT) |
	(CLKSEL_L3_CORE_DIV_2 << CLKSEL_L3_SHIFT) |
	(CLKSEL_L4_L3_DIV_2 << CLKSEL_L4_SHIFT);
	writel(temp, CM_CLKSEL_CORE);
	/* CORE DPLL has been configured but not locked */

	configure_per_dpll();
	/* PER DPLL has been configured and LOCKED */

	configure_mpu_dpll();
	/* MPU DPLL has been configured and LOCKED */

	configure_iva_dpll(&iva_dpll_params[0]);
	/* IVA DPLL has been configured and LOCKED */

	configure_abe_dpll(&abe_dpll_params);
	/* ABE DPLL has been configured and LOCKED */

	setup_emif_config();
	/* Put EMIF clock domain in sw wakeup mode */
	writel(0x00000002, CM_EMIF_CLKSTCTRL);
	writel(0x00001709, CM_SHADOW_FREQ_CONFIG1);
	if (!wait_on_value(BIT(0), 1, CM_SHADOW_FREQ_CONFIG1, LDELAY)) {
		/* do nothing */
	}
	/* core dpll has now been locked */

	configure_usb_dpll(&usb_dpll_params[0]);

	setup_clocks();

}
