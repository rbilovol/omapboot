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

#define CONFIG_BLAZE

#define CONFIG_BOARD_MACH_TYPE		3429
#define CONFIG_IS_OMAP4

#define CONFIG_BAUDRATE			115200

#define CONFIG_SERIAL_BASE		OMAP44XX_UART3
#define CONFIG_SERIAL_CLK_HZ		48000000

#define CONFIG_RAM_HANDLERS		0x4030D020
#define CONFIG_RAM_VECTORS		0x4030D000
#define CONFIG_STACK_TOP		0x4030D000

#define CONFIG_ADDR_DOWNLOAD		0x82000000

#define CONFIG_ADDR_ATAGS		0x80000100
#define CONFIG_ADDR_KERNEL		0x80008000
#define CONFIG_ADDR_RAMDISK		0x81000000

#define CONFIG_FASTBOOT			/* enable FASTBOOT for BLAZE */

#ifdef CONFIG_FASTBOOT
	#define PRODUCT_NAME		"blaze"
	#define FASTBOOT_VERSION	"0.5"
	#define MANUFACTURER_NAME	"Texas Instruments"
#else
	#define PRODUCT_NAME		""
	#define FASTBOOT_VERSION	""
	#define MANUFACTURER_NAME	""
#endif
