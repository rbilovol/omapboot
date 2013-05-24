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

#define CONFIG_PANDA

#define CONFIG_BOARD_MACH_TYPE		2791
#define CONFIG_IS_OMAP4
#define CONFIG_DEVICE_TREE		1

#define CONFIG_BAUDRATE			115200

#define CONFIG_SERIAL_BASE		OMAP44XX_UART3
#define CONFIG_SERIAL_CLK_HZ		48000000

#define CONFIG_ADDR_SBOOT		0x90000000

#define CONFIG_RAM_HANDLERS		0x4030D020
#define CONFIG_RAM_VECTORS		0x4030D000
#define CONFIG_STACK_SDRAM		0x83000000

#define MEMORY_SIZE			0x40000000
#define MEMORY_BASE			0x80000000
#define CONFIG_ADDR_ATAGS		(MEMORY_BASE + 0x100)
#define CONFIG_ADDR_KERNEL		(MEMORY_BASE + 0x8000)
#define CONFIG_ADDR_RAMDISK		(MEMORY_BASE + 0x01000000)
#define CONFIG_ADDR_DOWNLOAD		(MEMORY_BASE + 0x02000000)

#define CONFIG_FASTBOOT			/* enable FASTBOOT for PANDA */

#ifdef CONFIG_FASTBOOT
	#define PRODUCT_NAME		"panda"
	#define FASTBOOT_VERSION	"0.5"
	#define MANUFACTURER_NAME	"Texas Instruments"
#else
	#define PRODUCT_NAME		""
	#define FASTBOOT_VERSION	""
	#define MANUFACTURER_NAME	""
#endif

#define CONFIG_OMAP4_ANDROID_CMD_LINE
#define CONFIG_USE_CH			1

#define EXTENDED_CMDLINE	"mem=1024M@0x80000000 " \
				"mem=512M@0xA0000000 " \
				"omapfb.vram=0:4M " \
				"earlyprintk " \
				"vram=16M\0" ;
