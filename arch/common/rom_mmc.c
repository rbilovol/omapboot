/*
 * Copyright (C) 2012, Texas Instruments, Inc.
 * Texas Instruments, <www.ti.com>
 * Author: Olivier Deprez <o-deprez@ti.com>
 * Contributor: Christina Warren <cawarren@ti.com>
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

#include <common/usbboot_common.h>
#include <common/omap_rom.h>
#include <common/mmc.h>

#if defined CONFIG_IS_OMAP4
#include <omap4/hw.h>
#elif defined CONFIG_IS_OMAP5
#include <omap5/hw.h>
#endif

#include "config.h"

int mmc_open(u8 device, struct mmc *mmc)
{
	struct mem_device *md;
	struct mmc_devicedata *dd;
	u16 options;
	int n;

	if (!((device == DEVICE_SDCARD) || (device == DEVICE_EMMC))) {
		printf("unsupported mmsd device\n");
		return -1;
	}

	n = rom_get_mem_driver(&mmc->io, device);
	if (n) {
		printf("rom_get_mem_driver failed\n");
		return n;
	}

	/* clear memory device descriptor */
	md = &mmc->dread;
	memset(md, 0, sizeof(struct mem_device));

	/*initialize device data buffer*/
	dd = (void *) 0x80000000;
	memset(dd, 0, 2500);

	options			= 1;
	md->initialized		= 0;
	md->device_type		= device;
	md->xip_device		= 0;
	md->search_size		= 0;
	md->base_address	= 0;
	md->hs_toc_mask		= 0;
	md->gp_toc_mask		= 0;
	md->boot_options	= &options;
	md->device_data		= dd;

	n = mmc->io->init(md);
	if (n) {
		printf("mmc->io->init failed\n");
		return n;
	}

	/* force raw mode operation (if ever a filesystem was detected at
	init)*/
	dd->mode = MMCSD_MODE_RAW;

	switch (dd->type) {

	/* Configure SD card to 4b @ 19.2 MHz */
	case MMCSD_TYPE_SD:

		/*configure larger bus width*/
		n = mmc_configure(mmc, MMCSD_CONFIGID_SETBUSWIDTH,
				MMCSD_4BIT_BUS_WIDTH_SUPPORTED);
		if (n) {
			printf("mmc_configure: failed to configure "
							"buswidth\n");
			return n;
		}

		/*configure higher clock*/
		n = mmc_configure(mmc, MMCSD_CONFIGID_SETCLOCK,
				MMCSD_CLOCK_DIVIDER_19_2MHZ);
		if (n) {
			printf("mmc_configure: failed to set clock\n");
			return n;
			}

		break;


	/* Configure eMMC to 8b @ 48 MHz DDR */
	case MMCSD_TYPE_MMC:

		/*configure 8b ddr mode*/
		n = mmc_configure(mmc, MMCSD_CONFIGID_SETDDRMODE,
				MMCSD_8BIT_DDR_BUS_WIDTH_SUPPORTED);
		if (n) {
			printf("mmc_configure: failed to set DDR "
							"mode\n");
			return n;
		}

		/*configure higher clock*/
		n = mmc_configure(mmc, MMCSD_CONFIGID_SETCLOCK,
					MMCSD_CLOCK_DIVIDER_48MHz);
		if (n) {
			printf("mmc_configure: failed to set clock\n");
			return n;
		}

		break;

	default:
		printf("Unsupported mmcsd device!\n");
		break;
	}

	return 0;
}

int mmc_read(struct mmc *mmc, u32 start, u32 count, void *data)
{
	struct read_desc rd;
	int n;

	rd.sector_start	= start;
	rd.sector_count	= count;
	rd.destination	= data;

	n = mmc->io->read(&mmc->dread, &rd);
	if (n) {
		printf("mmc_read failed\n");
		return n;
	}

	return 0;
}

int mmc_configure(struct mmc *mmc, u32 id, u32 value)
{
	struct mmc_config config;
	config.configid = id;
	config.value = value;
	int n;

	n = mmc->io->configure(&mmc->dread, &config);
	if (n) {
		printf("mmc_configure failed\n");
		return n;
	}

	return 0;
}

int mmc_info(struct mmc *mmc)
{
	struct mmc_devicedata *dd;
	struct mem_device *md;

	/* Get the device data structure */
	md = &mmc->dread;
	dd = md->device_data;

	return 0;
}

int mmc_write(struct mmc *mmc, u32 start, u32 count, void *data)
{
	int (*rom_hal_mmchs_sendcommand)(u32 moduleid, u32 cmd, u32 arg,
								u32 *resp);
	int (*rom_hal_mmchs_writedata)(u32 moduleid, u32 *buf);
	struct bootloader_ops *boot_ops = (void *) 0x84100000;
	struct mmc_devicedata *dd;
	struct mem_device *md;
	u32 arg;
	u32 resp[4];
	int n;

	const u32 rom_hal_mmchs_writedata_addr[] = {
		0,           /*OMAP_REV_INVALID*/
		(0x25c2c|1), /*OMAP_4430_ES1_DOT_0*/
		(0x2ddd8|1), /*OMAP_4430_ES2_DOT_0*/
		(0x2ddd8|1), /*OMAP_4430_ES2_DOT_1*/
		(0x2df58|1), /*OMAP_4430_ES2_DOT_2*/
		(0x2df58|1), /*OMAP_4430_ES2_DOT_3*/
		(0x36028|1), /*OMAP_4460_ES1_DOT_0*/
		(0x36028|1),  /*OMAP_4460_ES1_DOT_1*/
		(0x36028|1),  /*4470 placeholder*/
		(0x3ee18|1),  /*OMAP_5430_ES1_DOT_0*/
		(0x3ee18|1)  /*OMAP_5432_ES1_DOT_0*/
	};

	const u32 rom_hal_mmchs_sendcommand_addr[] = {
		0,           /*OMAP_REV_INVALID*/
		(0x25aa8|1), /*OMAP_4430_ES1_DOT_0*/
		(0x2dc54|1), /*OMAP_4430_ES2_DOT_0*/
		(0x2dc54|1), /*OMAP_4430_ES2_DOT_1*/
		(0x2ddd4|1), /*OMAP_4430_ES2_DOT_2*/
		(0x2ddd4|1), /*OMAP_4430_ES2_DOT_3*/
		(0x35ea4|1), /*OMAP_4460_ES1_DOT_0*/
		(0x35ea4|1),  /*OMAP_4460_ES1_DOT_1*/
		(0x35ea4|1),  /*4470 placeholder*/
		(0x3ec8c|1),  /*OMAP_5430_ES1_DOT_0*/
		(0x3ec8c|1)  /*OMAP_5432_ES1_DOT_0*/
	};

	boot_ops->proc_ops = init_processor_id_funcs();
	/*TODO check for valid functions */
	rom_hal_mmchs_writedata   =
			API(&rom_hal_mmchs_writedata_addr[boot_ops->proc_ops->proc_get_proc_id()]);

	rom_hal_mmchs_sendcommand =
			API(&rom_hal_mmchs_sendcommand_addr[boot_ops->proc_ops->proc_get_proc_id()]);

	/* Get the device data structure */
	md = &mmc->dread;
	dd = md->device_data;

	if (dd->addressing == MMCSD_ADDRESSING_SECTOR)
		/* In case of sector addressing,
		the address given is the sector nb */
		arg = start;
	else
		/* In case of byte addressing,
		the address given is start sector * MMCSD_SECTOR_SIZE */
		arg = start << MMCSD_SECTOR_SIZE_SHIFT;

	if (md->device_type == DEVICE_SDCARD)
		*((volatile u32*)OMAP_HSMMC1_BLK) |= count<<16;
	else
		*((volatile u32*)OMAP_HSMMC2_BLK) |= count<<16;

	/* Send the CMD25 write command */
	n = rom_hal_mmchs_sendcommand(dd->moduleid,
			MMCSD_CMD25 | MMCHS_MMCHS_CMD_BCE_ENABLE |
			MMCHS_MMCHS_CMD_ACEN_ENABLECMD12 |
			MMCHS_MMCHS_CMD_MSBS_MULTIBLK, arg, resp);
	if (n) {
		printf("mmc_sendcommand failed\n");
		return n;
	}

	/* Write the data */
	n = rom_hal_mmchs_writedata(dd->moduleid, data);
	if (n) {
		printf("mmc_writedata failed\n");
		return n;
	}

	return 0;
}
