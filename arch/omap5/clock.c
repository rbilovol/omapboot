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

#include <aboot/aboot.h>
#include <common/common_proc.h>
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
#define CORE_VOLTAGE	1040000

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
#define IVA_VOLTAGE	1040000

/* OPP NOM */
struct dpll_param abe_dpll_params = {
	750, 0, 1, 1, -1, -1, -1, -1, -1, -1
};

#define MPU_VOLTAGE	1040000

void setup_clocks(void)
{
	struct omap_clocks * oclock;
	for (oclock = &omap5_clocks[0]; oclock->ad > 0; oclock++) {
		switch (oclock->ctrl) {
		case WRITEL:
			writel(oclock->value, oclock->ad);
			break;
		case MODIFY:
		case MODIFY_WAIT:
			set_modify(oclock->ad, oclock->mask,
						oclock->value);
			if (oclock->ctrl == MODIFY_WAIT) {
				if (!check_loop(BIT(16) | BIT(17), 0,
						oclock->ad))
				;
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
	set_modify(CM_CLKMODE_DPLL_CORE, 0x0000000f, IDLE_BYPASS_FAST_RELOCK_MODE);
	if (!check_loop(BIT(0), 0, CM_IDLEST_DPLL_CORE)) {
		/* do nothing */
	}

	set_modify(CM_CLKSEL_DPLL_CORE, 0x0007ff00, dpll_param_p->m << 8);
	set_modify(CM_CLKSEL_DPLL_CORE, 0x0000007f, dpll_param_p->n);


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
	if (!check_loop(BIT(0), 0, CM_IDLEST_DPLL_PER)) {
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
	if (!check_loop(BIT(0), 1, CM_IDLEST_DPLL_PER)) {
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

	if (!check_loop(BIT(0), 0, CM_IDLEST_DPLL_MPU)) {
		/* do nothing */
	}

	/* Program DPLL frequency (M and N) */
	set_modify(CM_CLKSEL_DPLL_MPU, 0x0007FF7F, 0x00017708);

	/* Program DPLL_CLKOUT divider (M2 = 1) */
	writel(0x00000001, CM_DIV_M2_DPLL_MPU);

	/* Put DPLL into lock mode */
	writel(0x00000007, CM_CLKMODE_DPLL_MPU);

	/* Wait for DPLL to be locked */
	if (!check_loop(BIT(0), 1, CM_IDLEST_DPLL_MPU)) {
		/* do nothing */
	}

	return;
}

void configure_iva_dpll(dpll_param *dpll_param_p)
{
	/* Unlock the IVA dpll */
	set_modify(CM_CLKMODE_DPLL_IVA, 0x00000007, IDLE_BYPASS_FAST_RELOCK_MODE);
	if (!check_loop(BIT(0), 0, CM_IDLEST_DPLL_IVA)) {
		/* do nothing */
	}

	/* CM_BYPCLK_DPLL_IVA = CORE_X2_CLK/2 */
	set_modify(CM_BYPCLK_DPLL_IVA, 0x00000003, 0x1);
	/* Disable DPLL autoidle */
	set_modify(CM_AUTOIDLE_DPLL_IVA, 0x00000007, 0x0);
	set_modify(CM_CLKSEL_DPLL_IVA, 0x0007ff00, dpll_param_p->m << 8);
	set_modify(CM_CLKSEL_DPLL_IVA, 0x0000007f, dpll_param_p->n);
	set_modify(CM_DIV_H11_DPLL_IVA, 0x0000003f, dpll_param_p->h11);
	set_modify(CM_DIV_H11_DPLL_IVA, 0x00000200, 0x1 << 9);
	set_modify(CM_DIV_H12_DPLL_IVA, 0x0000003f, dpll_param_p->h12);
	set_modify(CM_DIV_H12_DPLL_IVA, 0x00000200, 0x1 << 9);

	/* Lock the iva dpll */
	set_modify(CM_CLKMODE_DPLL_IVA, 0x00000007, PLL_LOCK);
	if (!check_loop(BIT(0), 1, CM_IDLEST_DPLL_IVA)) {
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

	set_modify(CM_CLKMODE_DPLL_ABE, 0x00000007, IDLE_BYPASS_FAST_RELOCK_MODE);
	if (!check_loop(BIT(1), 0, CM_IDLEST_DPLL_ABE)) {
		/* do nothing */
	}

	set_modify(CM_CLKSEL_DPLL_ABE, 0x0007ff00, dpll_param_p->m << 8);
	set_modify(CM_CLKSEL_DPLL_ABE, 0x0000007f, dpll_param_p->n);
	set_modify(CM_CLKMODE_DPLL_ABE, 0x00000007, PLL_LOCK);
	if (!check_loop(BIT(0), 1, CM_IDLEST_DPLL_ABE)) {
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

	set_modify(CM_CLKSEL_DPLL_USB, 0xff000000, sd_div << 24);

	/* Unlock the USB dpll */
	set_modify(CM_CLKMODE_DPLL_USB, 0x00000007, IDLE_BYPASS_LOW_POWER_MODE);
	if (!check_loop(BIT(0), 0, CM_IDLEST_DPLL_USB)) {
		/* do nothing */
	}

	set_modify(CM_CLKSEL_DPLL_USB, 0x000fff00, dpll_param_p->m << 8);
	set_modify(CM_CLKSEL_DPLL_USB, 0x0000007f, dpll_param_p->n);

	/* lock the dpll */
	writel(0x00000007, CM_CLKMODE_DPLL_USB);
	if (!check_loop(BIT(0), 1, CM_IDLEST_DPLL_USB)) {
		/* do nothing */
	}

	/* Setup post-dividers */
	if (dpll_param_p->m2 >= 0)
		writel(dpll_param_p->m2, CM_DIV_M2_DPLL_USB);

	return;
}

static u8 get_twl6035_voltage(u32 uv)
{
	/* format the desired voltage for palmas */
	return (u8)((uv - 500000) / 10000) + 6;
}

static u32 get_twl6035_slewdelay(u32 opp_uv)
{
	/*
	 * compute the delay for voltage to stabilize
	 * depending upon the voltage transition
	 * Current levels with TWL6035 palmas for ES1.0:
	 * opp_boot: 1050000 uv
	 * slew rate = 5 mv per ms
	 * Revisit when ES2.0 is out
	 */
	u32 slew_rate = 5;
	u32 opp_boot_uv = 1050000;
	u32 delta_uv;
	u32 sdelay;

	if (opp_boot_uv > opp_uv)
		delta_uv = opp_boot_uv - opp_uv;
	else
		delta_uv = opp_uv - opp_boot_uv;

	sdelay = (delta_uv / slew_rate) / 1000;

	return sdelay;
}

static void set_vcore(u8 regaddr, u8 slaveaddr, u32 voltage)
{
	u8 valid = 0x1;
	u8  data;
	u32 cmd;

	/* Compute and send i2c command */
	data = get_twl6035_voltage(voltage);

	cmd = (valid << 24) | (data << 16) | (regaddr << 8) | slaveaddr;
	writel(cmd, PRM_VC_VAL_BYPASS);

	/* check if voltage Controller did not send out command */
	if (!check_loop(0x01000000, 0, PRM_VC_VAL_BYPASS)) {
		/* do nothing, serial not setup yet */
	}

	/* make sure voltage stabilized */
	ldelay(TIME_LOOP_RATIO * get_twl6035_slewdelay(voltage));

	/* clean up irq status */
	set_modify(PRM_IRQSTATUS_MPU, 0x00000000, 0x00000000);
}

void scale_vcores(void)
{
	u32 regaddr;
	u32 slaveaddr;

	/* Configure SR I2C Mode - FS mode */
	writel(0x00000000, PRM_VC_CFG_I2C_MODE);

	/* Configure SR I2C Clock - TOBEFIXED! */
	writel(0xfffffffb, PRM_VC_CFG_I2C_CLK);

	/* Configure Palmas TWL6035 */
	slaveaddr = 0x12;

	/* VDD_CORE - SMPS8_VOLTAGE */
	regaddr = 0x37;
	set_vcore(regaddr, slaveaddr, CORE_VOLTAGE);

	/* VDD_MM:  SMPS45_VOLTAGE */
	regaddr = 0x2B;
	set_vcore(regaddr, slaveaddr, IVA_VOLTAGE);

	/* VDD_MPU:  SMPS12_VOLTAGE */
	regaddr = 0x23;
	set_vcore(regaddr, slaveaddr, MPU_VOLTAGE);

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
	if (!check_loop(BIT(0), 1, CM_SHADOW_FREQ_CONFIG1)) {
		/* do nothing */
	}
	/* core dpll has now been locked */

	configure_usb_dpll(&usb_dpll_params[0]);

	setup_clocks();

}
