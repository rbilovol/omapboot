/*
 * hw.h
 *
 * Copyright(c) 2011 Texas Instruments.   All rights reserved.
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

#ifndef _OMAP54XX_HW_H
#define  _OMAP54XX_HW_H

#define CONTROL_ID_CODE (0x4A002204)
#define CONTROL_STATUS	(0x4A002134)
#define CONTROL_STD_FUSE_DIE_ID_0 (0x4A002000)

#define OMAP54XX_L4_PER (0x48000000)

/* UART */
#define OMAP54XX_UART1			(OMAP54XX_L4_PER+0x6a000)
#define OMAP54XX_UART2			(OMAP54XX_L4_PER+0x6c000)
#define OMAP54XX_UART3			(OMAP54XX_L4_PER+0x20000)
#define OMAP54XX_UART4			(OMAP54XX_L4_PER+0x6e000)
#define OMAP54XX_UART5			(OMAP54XX_L4_PER+0x66000)
#define OMAP54XX_UART6			(OMAP54XX_L4_PER+0x68000)

/* MMC */
#define OMAP_HSMMC1_BASE        0x4809C000
#define OMAP_HSMMC2_BASE        0x480B4000
#define OMAP_HSMMC3_BASE        0x480AD000
#define OMAP_HSMMC4_BASE        0x480D1000
#define OMAP_HSMMC5_BASE        0x480D5000

#define OMAP_HSMMC_BLK_OFFSET	0x0204
#define OMAP_HSMMC1_BLK		(OMAP_HSMMC1_BASE + OMAP_HSMMC_BLK_OFFSET)
#define OMAP_HSMMC2_BLK		(OMAP_HSMMC2_BASE + OMAP_HSMMC_BLK_OFFSET)

#define PRM_BASE			0x4AE06000
#define PRM_DEVICE_BASE			(PRM_BASE + 0x1B00)
#define PRM_RSTCTRL			PRM_DEVICE_BASE
#define PRM_RSTCTRL_RESET_WARM_BIT	(1<<0)
#define PRM_RSTCTRL_RESET_COLD_BIT	(1<<1)
#define PRM_RSTST			(PRM_DEVICE_BASE + 0x4)
#define PRM_RSTST_RESET_COLD_BIT	(1<<0)
#define PRM_RSTST_RESET_WARM_BIT	(1<<1)

#define PUBLIC_SAR_RAM_1_FREE		(0x4AE26000 + 0xA0C)

#endif
