/*
 * clock.c
 *
 * Copyright(c) 2010 Texas Instruments.   All rights reserved.
 *
 * Texas Instruments, <www.ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
 * Rajendra Nayak <rnayak@ti.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Texas Instruments nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <aboot.h>
#include <common_proc.h>
#include <io.h>
#include <hw.h>

#define PLL_STOP		1 /* PER & IVA */
#define PLL_MN_POWER_BYPASS	4
#define PLL_LOW_POWER_BYPASS	5 /* MPU, IVA & CORE */
#define PLL_FAST_RELOCK_BYPASS	6 /* CORE */
#define PLL_LOCK		7 /* MPU, IVA, CORE & PER */

#define BIT0 (1<<0)
#define BIT16 (1<<16)
#define BIT17 (1<<17)
#define BIT18 (1<<18)

#define CONFIG_OMAP4_SDC 1

/* clk sel is 12M / 13M / 16.8M / 19.2M / 26M / 27M / 38.4M */
/* we only support 38.4M here */

/* #define CONFIG_MPU_1000 1 */

struct dpll_param mpu_dpll_param = {
#ifdef CONFIG_MPU_600
	0x7d, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
#elif CONFIG_MPU_1000
	0x69, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
#else /* 400MHz */
	0x1a, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,	
#endif
};

const struct dpll_param per_dpll_param = {
	0x14, 0x00, 0x08, 0x06, 0x0c, 0x09, 0x04, 0x05,
};

struct dpll_param iva_dpll_param = {
#ifdef CONFIG_OMAP4_SDC
	0x61, 0x03, 0x00, 0x00, 0x04, 0x07, 0x00, 0x00,
#else
	0x61, 0x03, 0x00, 0x00, 0x04, 0x07, 0x00, 0x00,
#endif
};

struct dpll_param core_dpll_param_ddr400mhz = {
	0x7d, 0x05, 0x01, 0x05, 0x08, 0x04, 0x06, 0x05,
};

struct dpll_param core_dpll_param_ddr466mhz = {
	0x36B, 0x23, 0x01, 0x06, 0x08, 0x04, 0x07, 0x05,
};

struct dpll_param abe_dpll_param = {
#ifdef CONFIG_OMAP4_SDC
	0x40, 0x18, 0x1, 0x1, 0x0, 0x0, 0x0, 0x0,
#else
	0x40, 0x18, 0x1, 0x1, 0x0, 0x0, 0x0, 0x0,
#endif
};

struct dpll_param usb_dpll_param = {
#ifdef CONFIG_OMAP4_SDC
	0x32, 0x1, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0,
#else
	0x32, 0x1, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0,
#endif
};

typedef struct dpll_param dpll_param;

static void configure_mpu_dpll(dpll_param *dpll_param_p,
			struct proc_specific_functions *proc_ops)
{
	/* Unlock the MPU dpll */
	set_modify(CM_CLKMODE_DPLL_MPU, 0x00000007, PLL_MN_POWER_BYPASS);
	check_loop(BIT0, 0, CM_IDLEST_DPLL_MPU);

	/* Disable DPLL autoidle */
	set_modify(CM_AUTOIDLE_DPLL_MPU, 0x00000007, 0x0);

	/* Set M,N,M2 values */
	set_modify(CM_CLKSEL_DPLL_MPU, 0x0007ff00, dpll_param_p->m << 8);

	set_modify(CM_CLKSEL_DPLL_MPU, 0x0000003f, dpll_param_p->n);

	set_modify(CM_DIV_M2_DPLL_MPU, 0x0000001f, dpll_param_p->m2);

	set_modify(CM_DIV_M2_DPLL_MPU, 0x00000100, 0x1 << 8);

	/* Lock the mpu dpll */
	set_modify(CM_CLKMODE_DPLL_MPU, 0x00000007, (PLL_LOCK | 10));
	check_loop(BIT0, 1, CM_IDLEST_DPLL_MPU);
}

static void configure_iva_dpll(dpll_param *dpll_param_p)
{
	/* Unlock the IVA dpll */
	set_modify(CM_CLKMODE_DPLL_IVA, 0x00000007, PLL_MN_POWER_BYPASS);

	check_loop(BIT0, 0, CM_IDLEST_DPLL_IVA);

	/* CM_BYPCLK_DPLL_IVA = CORE_X2_CLK/2 */
	set_modify(CM_BYPCLK_DPLL_IVA, 0x00000003, 0x1);

	/* Disable DPLL autoidle */
	set_modify(CM_AUTOIDLE_DPLL_IVA, 0x00000007, 0x0);

	/* Set M,N,M4,M5 */
	set_modify(CM_CLKSEL_DPLL_IVA, 0x0007ff00, dpll_param_p->m << 8);
	set_modify(CM_CLKSEL_DPLL_IVA, 0x0000007f, dpll_param_p->n);
	set_modify(CM_DIV_M4_DPLL_IVA, 0x0000001f, dpll_param_p->m4);
	set_modify(CM_DIV_M4_DPLL_IVA, 0x00000100, 0x1 << 8);
	set_modify(CM_DIV_M5_DPLL_IVA, 0x0000001f, dpll_param_p->m5);
	set_modify(CM_DIV_M5_DPLL_IVA, 0x00000100, 0x1 << 8);

	/* Lock the iva dpll */
	set_modify(CM_CLKMODE_DPLL_IVA, 0x00000007, PLL_LOCK);
	check_loop(BIT0, 1, CM_IDLEST_DPLL_IVA);
}

static void configure_per_dpll(const dpll_param *dpll_param_p)
{
	/* Unlock the PER dpll */
	set_modify(CM_CLKMODE_DPLL_PER, 0x00000007, PLL_MN_POWER_BYPASS);
	check_loop(BIT0, 0, CM_IDLEST_DPLL_PER);

	/* Disable autoidle */
	set_modify(CM_AUTOIDLE_DPLL_PER, 0x00000007, 0x0);
	set_modify(CM_CLKSEL_DPLL_PER, 0x0007ff00, dpll_param_p->m << 8);
	set_modify(CM_CLKSEL_DPLL_PER, 0x0000003f, dpll_param_p->n);
	set_modify(CM_DIV_M2_DPLL_PER, 0x0000001f, dpll_param_p->m2);
	set_modify(CM_DIV_M2_DPLL_PER, 0x00000100, 0x1 << 8);
	set_modify(CM_DIV_M3_DPLL_PER, 0x0000001f, dpll_param_p->m3);
	set_modify(CM_DIV_M3_DPLL_PER, 0x00000100, 0x1 << 8);
	set_modify(CM_DIV_M4_DPLL_PER, 0x0000001f, dpll_param_p->m4);
	set_modify(CM_DIV_M4_DPLL_PER, 0x00000100, 0x1 << 8);
	set_modify(CM_DIV_M5_DPLL_PER, 0x0000001f, dpll_param_p->m5);
	set_modify(CM_DIV_M5_DPLL_PER, 0x00000100, 0x1 << 8);
	set_modify(CM_DIV_M6_DPLL_PER, 0x0000001f, dpll_param_p->m6);
	set_modify(CM_DIV_M6_DPLL_PER, 0x00000100, 0x1 << 8);
	set_modify(CM_DIV_M7_DPLL_PER, 0x0000001f, dpll_param_p->m7);
	set_modify(CM_DIV_M7_DPLL_PER, 0x00000100, 0x1 << 8);

	/* Lock the per dpll */
	set_modify(CM_CLKMODE_DPLL_PER, 0x00000007, PLL_LOCK);
	check_loop(BIT0, 1, CM_IDLEST_DPLL_PER);
}

static void configure_abe_dpll(dpll_param *dpll_param_p)
{
	/* Select sys_clk as ref clk for ABE dpll */
	writel(0, CM_ABE_PLL_REF_CLKSEL);

	/* Unlock the ABE dpll */
	set_modify(CM_CLKMODE_DPLL_ABE, 0x00000007, PLL_MN_POWER_BYPASS);
	check_loop(BIT0, 0, CM_IDLEST_DPLL_ABE);

	/* Disable autoidle */
	set_modify(CM_AUTOIDLE_DPLL_ABE, 0x00000007, 0x0);

	set_modify(CM_CLKSEL_DPLL_ABE, 0x0007ff00, dpll_param_p->m << 8);
	set_modify(CM_CLKSEL_DPLL_ABE, 0x0000002f, dpll_param_p->n);

	/* Force DPLL CLKOUTHIF to stay enabled */
	set_modify(CM_DIV_M2_DPLL_ABE, 0x00000000, 0x500);
	set_modify(CM_DIV_M2_DPLL_ABE, 0x0000001f, dpll_param_p->m2);
	set_modify(CM_DIV_M2_DPLL_ABE, 0x00000100, 0x1 << 8);
	/* Force DPLL CLKOUTHIF to stay enabled */
	set_modify(CM_DIV_M3_DPLL_ABE, 0x00000000, 0x100);
	set_modify(CM_DIV_M3_DPLL_ABE, 0x0000001f, dpll_param_p->m3);
	set_modify(CM_DIV_M3_DPLL_ABE, 0x00000100, 0x1 << 8);

	/* Lock the abe dpll */
	set_modify(CM_CLKMODE_DPLL_ABE, 0x00000007, PLL_LOCK);
	check_loop(BIT0, 1, CM_IDLEST_DPLL_ABE);
}

static void configure_usb_dpll(dpll_param *dpll_param_p)
{
	/* Select the 60Mhz clock 480/8 = 60*/
	set_modify(CM_CLKSEL_USB_60MHz, 0x00000000, 0x1);

	/* Unlock the USB dpll */
	set_modify(CM_CLKMODE_DPLL_USB, 0x00000007, PLL_MN_POWER_BYPASS);
	check_loop(BIT0, 0, CM_IDLEST_DPLL_USB);

	/* Disable autoidle */
	set_modify(CM_AUTOIDLE_DPLL_USB, 0x00000007, 0x0);
	set_modify(CM_CLKSEL_DPLL_USB, 0x0007ff00, dpll_param_p->m << 8);
	set_modify(CM_CLKSEL_DPLL_USB, 0x0000003f, dpll_param_p->n);

	/* Force DPLL CLKOUT to stay active */
	set_modify(CM_DIV_M2_DPLL_USB, 0x00000000, 0x100);
	set_modify(CM_DIV_M2_DPLL_USB, 0x0000001f, dpll_param_p->m2);
	set_modify(CM_DIV_M2_DPLL_USB, 0x00000100, 0x1 << 8);
	set_modify(CM_CLKDCOLDO_DPLL_USB, 0x00000100, 0x1 << 8);

	/* Lock the usb dpll */
	set_modify(CM_CLKMODE_DPLL_USB, 0x00000007, PLL_LOCK);
	check_loop(BIT0, 1, CM_IDLEST_DPLL_USB);

	/* force enable the CLKDCOLDO clock */
	set_modify(CM_CLKDCOLDO_DPLL_USB, 0x00000000, 0x100);

}

void configure_core_dpll_no_lock(struct proc_specific_functions *proc_ops)
{
	int omap_rev;
	dpll_param *dpll_param_p;

	omap_rev = proc_ops->proc_get_proc_id();
	if (omap_rev >= OMAP_4470_ES1_DOT_0)
		dpll_param_p = &core_dpll_param_ddr466mhz;
	else
		dpll_param_p = &core_dpll_param_ddr400mhz;

	/* Get the sysclk speed from cm_sys_clksel
	 * Set it to 38.4 MHz, in case ROM code is bypassed
	 */
	writel(0x7,CM_SYS_CLKSEL);

	/* CORE_CLK=CORE_X2_CLK/2, L3_CLK=CORE_CLK/2, L4_CLK=L3_CLK/2 */
	set_modify(CM_CLKSEL_CORE, 0x00000000, 0x110);

	/* Unlock the CORE dpll */
	set_modify(CM_CLKMODE_DPLL_CORE, 0x00000007, PLL_MN_POWER_BYPASS);
	check_loop(BIT0, 0, CM_IDLEST_DPLL_CORE);

	/* Disable autoidle */
	set_modify(CM_AUTOIDLE_DPLL_CORE, 0x00000007, 0x0);
	set_modify(CM_CLKSEL_DPLL_CORE, 0x0007ff00, dpll_param_p->m << 8);
	set_modify(CM_CLKSEL_DPLL_CORE, 0x0000003f, dpll_param_p->n);
	set_modify(CM_DIV_M2_DPLL_CORE, 0x0000001f, dpll_param_p->m2);
	set_modify(CM_DIV_M3_DPLL_CORE, 0x0000001f, dpll_param_p->m3);
	set_modify(CM_DIV_M4_DPLL_CORE, 0x0000001f, dpll_param_p->m4);
	set_modify(CM_DIV_M5_DPLL_CORE, 0x0000001f, dpll_param_p->m5);
	set_modify(CM_DIV_M6_DPLL_CORE, 0x0000001f, dpll_param_p->m6);
	set_modify(CM_DIV_M7_DPLL_CORE, 0x0000001f, dpll_param_p->m7);
}

void lock_core_dpll(void)
{
	/* Lock the core dpll */
	set_modify(CM_CLKMODE_DPLL_CORE, 0x00000007, PLL_LOCK);
	check_loop(BIT0, 1, CM_IDLEST_DPLL_CORE);
}

void lock_core_dpll_shadow(struct proc_specific_functions *proc_ops)
{
	int omap_rev;
	dpll_param *dpll_param_p;
	u32 temp;
	temp = readl(CM_MEMIF_CLKSTCTRL);
	temp &= (~3);
	temp |= 2;
	writel(temp, CM_MEMIF_CLKSTCTRL);

	while(readl(CM_MEMIF_EMIF_1_CLKCTRL) & 0x30000)
		;

	while(readl(CM_MEMIF_EMIF_2_CLKCTRL) & 0x30000)
		;

	omap_rev = proc_ops->proc_get_proc_id();
	if (omap_rev >= OMAP_4470_ES1_DOT_0)
		dpll_param_p = &core_dpll_param_ddr466mhz;
	else
		dpll_param_p = &core_dpll_param_ddr400mhz;

	/* Lock the core dpll using freq update method */
	/*(CM_CLKMODE_DPLL_CORE) */
	writel(0x0A, 0x4A004120);

	/* CM_SHADOW_FREQ_CONFIG1: DLL_OVERRIDE = 1(hack), DLL_RESET = 1,
	 * DPLL_CORE_M2_DIV =1, DPLL_CORE_DPLL_EN = 0x7, FREQ_UPDATE = 1
	 */
	writel(0x70D | (dpll_param_p->m2 << 11), 0x4A004260);

	/* Wait for Freq_Update to get cleared: CM_SHADOW_FREQ_CONFIG1 */
	while((readl(0x4A004260) & 0x1) == 0x1)
		;

	/* Wait for DPLL to Lock : CM_IDLEST_DPLL_CORE */
	check_loop(BIT0, 1, CM_IDLEST_DPLL_CORE);
	//lock_core_dpll();

	while(readl(CM_MEMIF_EMIF_1_CLKCTRL) & 0x30000)
		;

	while(readl(CM_MEMIF_EMIF_2_CLKCTRL) & 0x30000)
		;

	writel(temp|3, CM_MEMIF_CLKSTCTRL);
}

static void enable_all_clocks(void)
{
	/* L4PER clocks */
	set_modify(CM_L4PER_CLKSTCTRL, 0x00000000, 0x2);
	set_modify(CM_L4PER_DMTIMER10_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_DMTIMER10_CLKCTRL);

	set_modify(CM_L4PER_DMTIMER11_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_DMTIMER11_CLKCTRL);

	set_modify(CM_L4PER_DMTIMER2_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_DMTIMER2_CLKCTRL);

	set_modify(CM_L4PER_DMTIMER3_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_DMTIMER3_CLKCTRL);

	set_modify(CM_L4PER_DMTIMER4_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_DMTIMER4_CLKCTRL);

	set_modify(CM_L4PER_DMTIMER9_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_DMTIMER9_CLKCTRL);

	/* GPIO clocks */
	set_modify(CM_L4PER_GPIO2_CLKCTRL, 0x00000000, 0x1);
	check_loop(BIT17|BIT16, 0, CM_L4PER_GPIO2_CLKCTRL);

	set_modify(CM_L4PER_GPIO3_CLKCTRL, 0x00000000, 0x1);
	check_loop(BIT17|BIT16, 0, CM_L4PER_GPIO3_CLKCTRL);

	set_modify(CM_L4PER_GPIO4_CLKCTRL, 0x00000000, 0x1);
	check_loop(BIT17|BIT16, 0, CM_L4PER_GPIO4_CLKCTRL);

	set_modify(CM_L4PER_GPIO4_CLKCTRL, 0x00000100, 0x1 << 8);

	set_modify(CM_L4PER_GPIO5_CLKCTRL, 0x00000000, 0x1);
	check_loop(BIT17|BIT16, 0, CM_L4PER_GPIO5_CLKCTRL);

	set_modify(CM_L4PER_GPIO6_CLKCTRL, 0x00000000, 0x1);
	check_loop(BIT17|BIT16, 0, CM_L4PER_GPIO6_CLKCTRL);

	set_modify(CM_L4PER_HDQ1W_CLKCTRL, 0x00000000, 0x2);

	/* I2C clocks */
	set_modify(CM_L4PER_I2C1_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_I2C1_CLKCTRL);

	set_modify(CM_L4PER_I2C2_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_I2C2_CLKCTRL);

	set_modify(CM_L4PER_I2C3_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_I2C3_CLKCTRL);

	set_modify(CM_L4PER_I2C4_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_I2C4_CLKCTRL);

	set_modify(CM_L4PER_MCBSP4_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_MCBSP4_CLKCTRL);

	/* MCSPI clocks */
	set_modify(CM_L4PER_MCSPI1_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_MCSPI1_CLKCTRL);

	set_modify(CM_L4PER_MCSPI2_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_MCSPI2_CLKCTRL);

	set_modify(CM_L4PER_MCSPI3_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_MCSPI3_CLKCTRL);

	set_modify(CM_L4PER_MCSPI4_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_MCSPI4_CLKCTRL);

	/* MMC clocks */
	set_modify(CM_L3INIT_HSMMC1_CLKCTRL, 0x00000003, 0x2);
	set_modify(CM_L3INIT_HSMMC1_CLKCTRL, 0x01000000, 0x1 << 24);
	set_modify(CM_L3INIT_HSMMC2_CLKCTRL, 0x00000003, 0x2);
	set_modify(CM_L3INIT_HSMMC2_CLKCTRL, 0x01000000, 0x1 << 24);

	set_modify(CM_L4PER_MMCSD3_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT18|BIT17|BIT16, 0, CM_L4PER_MMCSD3_CLKCTRL);

	set_modify(CM_L4PER_MMCSD4_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT18|BIT17|BIT16, 0, CM_L4PER_MMCSD4_CLKCTRL);

	set_modify(CM_L4PER_MMCSD5_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_MMCSD5_CLKCTRL);

	/* UART clocks */
	set_modify(CM_L4PER_UART1_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_UART1_CLKCTRL);

	set_modify(CM_L4PER_UART2_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_UART2_CLKCTRL);

	set_modify(CM_L4PER_UART3_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_UART3_CLKCTRL);

	set_modify(CM_L4PER_UART4_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_L4PER_UART4_CLKCTRL);

	/* WKUP clocks */
	set_modify(CM_WKUP_GPIO1_CLKCTRL, 0x00000000, 0x1);
	check_loop(BIT17|BIT16, 0, CM_WKUP_GPIO1_CLKCTRL);

	set_modify(CM_WKUP_TIMER1_CLKCTRL, 0x00000000, 0x01000002);
	check_loop(BIT17|BIT16, 0, CM_WKUP_TIMER1_CLKCTRL);

	set_modify(CM_WKUP_KEYBOARD_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_WKUP_KEYBOARD_CLKCTRL);

	set_modify(CM_SDMA_CLKSTCTRL, 0x00000000, 0x0);
	set_modify(CM_MEMIF_CLKSTCTRL, 0x00000000, 0x3);

	set_modify(CM_MEMIF_EMIF_1_CLKCTRL, 0x00000000, 0x1);
	check_loop(BIT17|BIT16, 0, CM_MEMIF_EMIF_1_CLKCTRL);

	set_modify(CM_MEMIF_EMIF_2_CLKCTRL, 0x00000000, 0x1);
	check_loop(BIT17|BIT16, 0, CM_MEMIF_EMIF_2_CLKCTRL);

	set_modify(CM_D2D_CLKSTCTRL, 0x00000000, 0x3);

	set_modify(CM_L3_2_GPMC_CLKCTRL, 0x00000000, 0x1);
	check_loop(BIT17|BIT16, 0, CM_L3_2_GPMC_CLKCTRL);

	set_modify(CM_L3INSTR_L3_3_CLKCTRL, 0x00000000, 0x1);
	check_loop(BIT17|BIT16, 0, CM_L3INSTR_L3_3_CLKCTRL);

	set_modify(CM_L3INSTR_L3_INSTR_CLKCTRL, 0x00000000, 0x1);
	check_loop(BIT17|BIT16, 0, CM_L3INSTR_L3_INSTR_CLKCTRL);

	set_modify(CM_L3INSTR_OCP_WP1_CLKCTRL, 0x00000000, 0x1);
	check_loop(BIT17|BIT16, 0, CM_L3INSTR_OCP_WP1_CLKCTRL);

	/* WDT clocks */
	set_modify(CM_WKUP_WDT2_CLKCTRL, 0x00000000, 0x2);
	check_loop(BIT17|BIT16, 0, CM_WKUP_WDT2_CLKCTRL);

	/* Select DPLL PER CLOCK as source for SGX FCLK */
	set_modify(CM_SGX_SGX_CLKCTRL, 0x01000000, 0x1 << 24);

	/* Enable clocks for USB fast boot to work */
	set_modify(CM_L3INIT_USBPHY_CLKCTRL, 0x00000000, 0x301);
	set_modify(CM_L3INIT_HSUSBOTG_CLKCTRL, 0x00000000, 0x1);

	return;
}

/* must be called from sram or flash */
void prcm_init(struct proc_specific_functions *proc_ops)
{
	u32 clk_index;
	/* Get the sysclk speed from cm_sys_clksel
	 * Set the CM_SYS_CLKSEL in case ROM code has not set
	 */
	writel(0x7,CM_SYS_CLKSEL);
	clk_index = readl(CM_SYS_CLKSEL);
	if (!clk_index)
		return;

	/* Configure all DPLL's at 100% OPP */
	configure_mpu_dpll(&mpu_dpll_param, proc_ops);
	configure_iva_dpll(&iva_dpll_param);
	configure_per_dpll(&per_dpll_param);
	configure_abe_dpll(&abe_dpll_param);
	configure_usb_dpll(&usb_dpll_param);

	enable_all_clocks();
}

static void omap_vc_init(u8 hscll, u8 hsclh, u8 scll, u8 sclh)
{
	u32 val;

	val = 0x00 << PRM_VC_CFG_I2C_MODE_HSMCODE_SHIFT;
	if (hscll || hsclh)
		val |= PRM_VC_CFG_I2C_MODE_HSMODEEN_BIT;

	writel(val, PRM_VC_CFG_I2C_MODE);

	hscll &= PRM_VC_CFG_I2C_CLK_HSCLL_MASK;
	hsclh &= PRM_VC_CFG_I2C_CLK_HSCLH_MASK;
	scll &= PRM_VC_CFG_I2C_CLK_SCLL_MASK;
	sclh &= PRM_VC_CFG_I2C_CLK_SCLH_MASK;

	val = hscll << PRM_VC_CFG_I2C_CLK_HSCLL_SHIFT |
	    hsclh << PRM_VC_CFG_I2C_CLK_HSCLH_SHIFT |
	    scll << PRM_VC_CFG_I2C_CLK_SCLL_SHIFT |
	    sclh << PRM_VC_CFG_I2C_CLK_SCLH_SHIFT;
	writel(val, PRM_VC_CFG_I2C_CLK);
}

void scale_vcores(struct proc_specific_functions *proc_ops)
{
	/*
	 * Dont use HSMODE, scll=0x60, sclh=0x26
	 * Note on HSMODE = 0:
	 * This allows us to allow a voltage domain to scale while we do i2c
	 * operation for the next domain - Please verify to ensure
	 * adequate delays are present in the case of slower ramp time for PMIC.
	 * This settings allow upto ~100Usec latency covered while i2c operation
	 * is in progress with the above configuration.
	 * Note #2: this latency will also depend on i2c_clk configuration as
	 * well.
	 */
	omap_vc_init(0x00, 0x00, 0x60, 0x26);

	/* set VCORE1 force VSEL */
	/* PRM_VC_VAL_BYPASS) */
	writel(0x3A5512, 0x4A307BA0);

	writel(readl(0x4A307BA0) | 0x1000000, 0x4A307BA0);
	while (readl(0x4A307BA0) & 0x1000000)
		;

	/* PRM_IRQSTATUS_MPU */
	writel(readl(0x4A306010), 0x4A306010);


	/* FIXME: set VCORE2 force VSEL, Check the reset value */
	/* PRM_VC_VAL_BYPASS) */
	writel(0x295B12, 0x4A307BA0);
	writel(readl(0x4A307BA0) | 0x1000000, 0x4A307BA0);
	while (readl(0x4A307BA0) & 0x1000000)
		;

	/* PRM_IRQSTATUS_MPU */
	writel(readl(0x4A306010), 0x4A306010);

	/*set VCORE3 force VSEL */
	/* PRM_VC_VAL_BYPASS */
	writel(0x2A6112, 0x4A307BA0);

	writel(readl(0x4A307BA0) | 0x1000000, 0x4A307BA0);

	while (readl(0x4A307BA0) & 0x1000000)
		;

	/* PRM_IRQSTATUS_MPU */
	writel(readl(0x4A306010), 0x4A306010);
}
