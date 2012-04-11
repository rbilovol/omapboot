/*
 * Copyright (c) 2010, The Android Open Source Project.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Neither the name of The Android Open Source Project nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
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
 *
 */

#include <aboot/aboot.h>
#include <aboot/types.h>
#include <aboot/io.h>
#include <aboot/common.h>
#include <aboot/bootimg.h>

#if defined CONFIG_IS_OMAP4
#include <omap4/hw.h>
#elif defined CONFIG_IS_OMAP5
#include <omap5/hw.h>
#endif

#include <common/omap_rom.h>
#include <libc/string.h>
#include "config.h"

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

#if defined CONFIG_FASTBOOT
#include <common/fastboot.h>
#include <common/sparse_format.h>

struct usb usb;
struct mmc mmc;
struct mmc_devicedata *dd;

/* To support the Android-style naming of flash */
#define MAX_PTN 16
static fastboot_ptentry ptable[MAX_PTN];
static struct fastboot_ptentry *e;

static u8 *transfer_buffer = (void *) 0x82000000;
static u8 *read_buffer = (void *) 0x83000000;
static char *dsize;
static u32 getsize;
static u32 sector;
static sparse_header_t *sparse_header;

char *get_serial_number(void)
{
	static char serialno[20];
	u32 val[4] = { 0 };
	u32 reg;

	reg = CONTROL_STD_FUSE_DIE_ID_0;
	val[0] = readl(reg);
	val[1] = readl(reg + 0x8);
	val[2] = readl(reg + 0xC);
	val[3] = readl(reg + 0x10);
	DBG("Device Serial Number: %08X%08X\n", val[3], val[2]);
	sprintf(serialno, "%08X%08X", val[3], val[2]);

	return serialno;
}

static char *get_proc_type(void)
{
	static char proc_type[8];
	int proc = get_omap_type();

	switch (proc) {
	case OMAP_TYPE_EMU:
		strcpy(proc_type, "EMU");
		break;
	case OMAP_TYPE_SEC:
		strcpy(proc_type, "HS");
		break;
	case OMAP_TYPE_GP:
		strcpy(proc_type, "GP");
		break;
	default:
		strcpy(proc_type, "unknown");
		break;
	}

	return proc_type;
}

static char *get_cpurevision(void)
{
	static char cpu_rev[8];
	int cpu = get_omap_rev();

	switch (cpu) {
	case OMAP_4430_ES1_DOT_0:
		strcpy(cpu_rev, "ES1.0");
		break;
	case OMAP_4430_ES2_DOT_0:
		strcpy(cpu_rev, "ES2.0");
		break;
	case OMAP_4430_ES2_DOT_1:
		strcpy(cpu_rev, "ES2.1");
		break;
	case OMAP_4430_ES2_DOT_2:
		strcpy(cpu_rev, "ES2.2");
		break;
	case OMAP_4430_ES2_DOT_3:
		strcpy(cpu_rev, "ES2.3");
		break;
	case OMAP_4460_ES1_DOT_0:
		strcpy(cpu_rev, "ES1.0");
		break;
	case OMAP_4460_ES1_DOT_1:
		strcpy(cpu_rev, "ES1.1");
		break;
	case OMAP_5430_ES1_DOT_0:
	case OMAP_5432_ES1_DOT_0:
		strcpy(cpu_rev, "ES1.0");
		break;
	default:
		printf("OMAP_REV_INVALID\n");
		strcpy(cpu_rev, "invalid");
		break;
	}

	return cpu_rev;
}

static char *get_procversion(void)
{
	static char proc_ver[8];
	int cpu = get_omap_rev();

	switch (cpu) {
	case OMAP_4430_ES1_DOT_0:
		strcpy(proc_ver, "4430");
		break;
	case OMAP_4430_ES2_DOT_0:
		strcpy(proc_ver, "4430");
		break;
	case OMAP_4430_ES2_DOT_1:
		strcpy(proc_ver, "4430");
		break;
	case OMAP_4430_ES2_DOT_2:
		strcpy(proc_ver, "4430");
		break;
	case OMAP_4430_ES2_DOT_3:
		strcpy(proc_ver, "4430");
		break;
	case OMAP_4460_ES1_DOT_0:
		strcpy(proc_ver, "4460");
		break;
	case OMAP_4460_ES1_DOT_1:
		strcpy(proc_ver, "4460");
		break;
	case OMAP_5430_ES1_DOT_0:
		strcpy(proc_ver, "5430");
		break;
	case OMAP_5432_ES1_DOT_0:
		strcpy(proc_ver, "5432");
		break;
	default:
		printf("OMAP_REV_INVALID\n");
		strcpy(proc_ver, "invalid");
		break;
	}

	return proc_ver;
}

static int fastboot_tx_status(const char *buffer, unsigned int buffer_size)
{
	/* send response back to host */
	static char response[65];
	strcpy(response, buffer);
	usb_write(&usb, response, strlen(response));
	return 0;
}

static int fastboot_getvar(const char *rx_buffer, char *tx_buffer)
{
	char serial[20];
	char proctype[8];
	char cpurev[8];
	char procver[8];
	char response[65];

	strcpy(response, "OKAY");

	DBG("fastboot_getvar()\n");

	if (!memcmp(rx_buffer, "version-bootloader", 18))
		strcpy(response + 4, ABOOT_VERSION);
	else if (!memcmp(rx_buffer, "version", 7))
		strcpy(response + 4, FASTBOOT_VERSION);
	else if (!memcmp(rx_buffer, "product", 7))
		strcpy(response + 4, PRODUCT_NAME);
	else if (!memcmp(rx_buffer, "serialno", 8)) {
		strcpy(serial, get_serial_number());
		strcpy(response + 4, serial);
	} else if (!memcmp(rx_buffer, "cpurev", 6)) {
		strcpy(cpurev, get_cpurevision());
		strcpy(response + 4, cpurev);
	} else if (!memcmp(rx_buffer, "secure", 6)) {
		strcpy(proctype, get_proc_type());
		strcpy(response + 4, proctype);
	} else if (!memcmp(rx_buffer, "cpu", 3)) {
		strcpy(procver, get_procversion());
		strcpy(response + 4, procver);
	} else if (!memcmp(rx_buffer, "downloadsize", 12)) {
		if (getsize)
			sprintf(response + 4, "%08x", getsize);
	} else if (!strcmp(rx_buffer, "userdata_size")) {
		strcpy(response + 4, get_ptn_size(response + strlen(response),
								"userdata"));
	} else if (!strcmp(rx_buffer, "all")) {
		/* product name */
		strcpy(response, "INFO");
		strcpy(response + strlen(response), "product: ");
		strcpy(response + strlen(response), PRODUCT_NAME);
		fastboot_tx_status(response, strlen(response));
		/* processor version */
		strcpy(response, "INFO");
		strcpy(procver, get_procversion());
		strcpy(response + strlen(response), "cpu: ");
		strcpy(response + strlen(response), procver);
		fastboot_tx_status(response, strlen(response));
		/* cpu revision */
		strcpy(response, "INFO");
		strcpy(cpurev, get_cpurevision());
		strcpy(response + strlen(response), "cpurev: ");
		strcpy(response + strlen(response), cpurev);
		fastboot_tx_status(response, strlen(response));
		/* device is GP/EMU/HS */
		strcpy(response, "INFO");
		strcpy(proctype, get_proc_type());
		strcpy(response + strlen(response), "secure: ");
		strcpy(response + strlen(response), proctype);
		fastboot_tx_status(response, strlen(response));
		strcpy(response, "INFO");
		/*serial number */
		strcpy(serial, get_serial_number());
		strcpy(response + strlen(response), "serialno: ");
		strcpy(response + strlen(response), serial);
		fastboot_tx_status(response, strlen(response));

		strcpy(response, "OKAY");
	} else
		printf("fastboot_getvar():unsupported variable\n");

	fastboot_tx_status(response, strlen(response));

	return 0;
}

void fastboot_flash_add_ptn(fastboot_ptentry *ptn, int count)
{
	if (count < MAX_PTN) {
		memcpy(ptable + count, ptn, sizeof(*ptn));
	count++;
	}
}

void fastboot_flash_dump_ptn(int count)
{
	unsigned int n;
	for (n = 0; n < count; n++) {
		fastboot_ptentry *ptn = ptable + n;
		printf("ptn %d name='%s' start=%d len=%d\n",
		n, ptn->name, ptn->start, ptn->length);
	}
}

fastboot_ptentry *fastboot_flash_find_ptn(const char *name)
{
    unsigned int n;

	for (n = 0; n < MAX_PTN; n++) {
		/* Make sure a substring is not accepted */
		if (strlen(name) == strlen(ptable[n].name)) {
			if (0 == strcmp(ptable[n].name, name))
				return ptable + n;
		}
	}
	return 0;
}

fastboot_ptentry *fastboot_flash_get_ptn(unsigned int n, int count)
{
	if (n < count)
		return ptable + n;
	else
		return 0;
}

static int download_image(void)
{
	int ret = 0;
	int size_of_dsize = 0;
	int count = 0;
	char response[65];

	size_of_dsize = strlen(dsize);
	count = size_of_dsize;

	getsize =
		get_downloadsize_from_string(size_of_dsize, dsize);
	if (getsize == 0) {
		sprintf(response, "FAILdata invalid size");
		return -1;
	} else
		sprintf(response, "DATA%08x", getsize);

	fastboot_tx_status(response, strlen(response));

	/* read the data */
	printf("Reading %u amount of data ...\n", getsize);
	ret = usb_read(&usb, transfer_buffer, getsize);
	if (ret < 0) {
		printf("failed to read the fastboot command\n");
		strcpy(response, "FAIL");
		return ret;
	} else {
		strcpy(response, "OKAY");
		fastboot_tx_status(response, strlen(response));
	}

	return ret;

}

static int flash_sparse_formatted_image(void)
{
	int ret = 0;
	u32 chunk = 0;
	u32 chunk_data_sz = 0;
	u32 num_sectors = 0;
	u32 out_blocks = 0;
	u32 total_blocks = 0;
	char response[65];

	chunk_header_t *chunk_header;

	if ((sparse_header->total_blks * sparse_header->blk_sz) > e->length) {
		printf("Image size exceeds %d limit\n", e->length);
		strcpy(response, "FAIL");
		return -1;
	}

	if ((sparse_header->major_version != 1) ||
		(sparse_header->file_hdr_sz != sizeof(sparse_header_t)) ||
		(sparse_header->chunk_hdr_sz != sizeof(chunk_header_t))) {
			printf("Invalid sparse format\n");
			strcpy(response, "FAIL");
			return -1;
	}

	/* Read and skip over sparse image header */
	transfer_buffer += sparse_header->file_hdr_sz;

	if (sparse_header->file_hdr_sz > sizeof(sparse_header_t)) {
		/* Skip the remaining bytes in a header
		that is longer than we expected */
		transfer_buffer += (sparse_header->file_hdr_sz -
					sizeof(sparse_header_t));
	}

	DBG("=== Sparse Image Header ===\n");
	DBG("sizeof(sparse_header_t) = %d\n", sizeof(sparse_header_t));
	DBG("magic: 0x%x\n", sparse_header->magic);
	DBG("major_version: 0x%x\n", sparse_header->major_version);
	DBG("minor_version: 0x%x\n", sparse_header->minor_version);
	DBG("file_hdr_sz: %d\n", sparse_header->file_hdr_sz);
	DBG("chunk_hdr_sz: %d\n", sparse_header->chunk_hdr_sz);
	DBG("blk_sz: %d\n", sparse_header->blk_sz);
	DBG("total_blks: %d\n", sparse_header->total_blks);
	DBG("total_chunks: %d\n", sparse_header->total_chunks);

	/* Start processing chunks */
	for (chunk = 0; chunk < sparse_header->total_chunks; chunk++) {

		/* Read and skip over chunk header */
		chunk_header = (chunk_header_t *) transfer_buffer;
		transfer_buffer += sizeof(chunk_header_t);

		DBG("=== Chunk Header ===\n");
		DBG("sizeof(sparse_header_t) = %d"
					"\n", sizeof(chunk_header_t));
		DBG("chunk_type: 0x%x\n", chunk_header->chunk_type);
		DBG("chunk_sz: 0x%x\n", chunk_header->chunk_sz);
		DBG("total_sz: 0x%x\n", chunk_header->total_sz);

		if (sparse_header->chunk_hdr_sz > sizeof(chunk_header_t)) {

			/* Skip the remaining bytes in a header that is longer
			than we	expected */
			transfer_buffer += (sparse_header->chunk_hdr_sz -
						sizeof(chunk_header_t));
		}

		chunk_data_sz = (sparse_header->blk_sz *
						chunk_header->chunk_sz);
		DBG("chunk_data_sz = %d\n", chunk_data_sz);

		num_sectors = (chunk_data_sz/512);
		printf("writing to sector %d and # of sectors %d\n",
							sector, num_sectors);

		switch (chunk_header->chunk_type) {

		case CHUNK_TYPE_RAW:

			if (chunk_header->total_sz !=
				(sparse_header->chunk_hdr_sz +
							chunk_data_sz)) {
				DBG("bogus chunk size for chunk type RAW "
							"%d\n", chunk);
			}

			out_blocks += chunk_data_sz;

			ret = mmc_write(&mmc, sector, num_sectors,
							transfer_buffer);
			if (ret != 0) {
				printf("mmc write failed\n");
				return ret;
			}

			#ifdef DEBUG
			/*read back the data and compare */
			for (sector_count = 0; sector_count < num_sectors;
							sector_count++) {

				ret = mmc_read(&mmc, sector+sector_count, 1,
					read_buffer + (sector_count*512));
				if (ret != 0) {
					printf("mmc read failed\n");
					return ret;
				}
				if (memcmp(read_buffer + (sector_count*512),
				transfer_buffer + (sector_count*512), 512)) {
					printf("mmc data mismatch sector %d\n",
							sector + sector_count);
				}
			}
			#endif

			total_blocks += chunk_header->chunk_sz;
			sector += (chunk_data_sz/512);
			transfer_buffer += chunk_data_sz;
			break;

		case CHUNK_TYPE_DONT_CARE:

			total_blocks += chunk_header->chunk_sz;
			sector += (chunk_data_sz/512);
			DBG("bogus chunk size for chunktype DONT_CARE %d\n",
									chunk);
			out_blocks += chunk_data_sz;

			break;

		case CHUNK_TYPE_CRC:

			if (chunk_header->total_sz !=
					sparse_header->chunk_hdr_sz) {
				DBG("bogus chunk size for chunktype CRC %d"
								"\n", chunk);
			}

			total_blocks += chunk_header->chunk_sz;
			sector += (chunk_data_sz/512);
			transfer_buffer += chunk_data_sz;
			out_blocks += chunk_data_sz;

			break;

		default:

			sprintf(response, "FAILUnknown chunk type");
			break;

		return -1;
		}
	}

	printf("Wrote %d blocks, expected to write %d blocks \n", total_blocks,
						sparse_header->total_blks);

	printf("out blocks = %d\n", out_blocks);
	strcpy(response, "OKAY");
	fastboot_tx_status(response, strlen(response));

	return ret;
}

static int fastboot_update_zimage(char *response)
{
	boot_img_hdr *hdr = (boot_img_hdr *) read_buffer;
	u8 *ramdisk_buffer;
	u32 ramdisk_sector_start, ramdisk_sectors;
	u32 kernel_sector_start, kernel_sectors;
	u32 hdr_sectors = 0;
	u32 sectors_per_page = 0;

	e = fastboot_flash_find_ptn("boot");
	if (NULL == e) {
		sprintf(response, "FAILCannot find boot partition");
		return (-1);
	}
	/* Read the boot image header */
	hdr_sectors = CEIL(sizeof(struct boot_img_hdr), 512);
	if (mmc_read(&mmc, e->start, hdr_sectors, (void *)hdr)) {
		sprintf(response, "FAILCannot read hdr from boot partition");
		return (-1);
	}

	/* Extract ramdisk location and read it into local buffer */
	sectors_per_page = hdr->page_size / 512;
	ramdisk_sector_start = e->start + sectors_per_page;
	ramdisk_sector_start += CEIL(hdr->kernel_size, hdr->page_size)*
						sectors_per_page;
	ramdisk_sectors = CEIL(hdr->ramdisk_size, hdr->page_size)*
						sectors_per_page;

	ramdisk_buffer = (u8 *)hdr;
	ramdisk_buffer += hdr_sectors *512;
	if (mmc_read(&mmc, ramdisk_sector_start,
		ramdisk_sectors, ramdisk_buffer)) {
		sprintf(response, "FAILCannot read ramdisk from boot partition");
		return (-1);
	}

	/* Change the boot img hdr */
	hdr->kernel_size = getsize;
	if (mmc_write(&mmc, e->start,
		hdr_sectors, (void *)hdr)) {
		sprintf(response, "FAILCannot writeback boot img hdr");
		return (-1);
	}

	/* Write the new downloaded kernel*/
	kernel_sector_start = e->start + sectors_per_page;
	kernel_sectors = CEIL(hdr->kernel_size, hdr->page_size)*
					sectors_per_page;
	if (mmc_write(&mmc, kernel_sector_start, kernel_sectors,
			transfer_buffer)) {
		sprintf(response, "FAILCannot write new kernel");
		return (-1);
	}

	/* Write the saved Ramdisk back */
	ramdisk_sector_start = e->start + sectors_per_page;
	ramdisk_sector_start += CEIL(hdr->kernel_size, hdr->page_size)*
						sectors_per_page;
	if (mmc_write(&mmc, ramdisk_sector_start, ramdisk_sectors,
						ramdisk_buffer)) {
		sprintf(response, "FAILCannot write back original ramdisk");
		return (-1);
	}

	sprintf(response, "OKAY");
	return (0);
}

static int fastboot_update_ramdisk(char *response)
{
	boot_img_hdr *hdr = (boot_img_hdr *) read_buffer;
	u32 ramdisk_sector_start, ramdisk_sectors;
	u32 hdr_sectors = 0;
	u32 sectors_per_page = 0;

	e = fastboot_flash_find_ptn("boot");
	if (NULL == e) {
		sprintf(response, "FAILCannot find boot partition");
		return -1;
	}
	/* Read the boot image header */
	hdr_sectors = CEIL(sizeof(struct boot_img_hdr), 512);
	if (mmc_read(&mmc, e->start, hdr_sectors, (void *)hdr)) {
		sprintf(response, "FAILCannot read hdr from boot partition");
		return -1;
	}

	/* Calculate ramdisk location */
	sectors_per_page = hdr->page_size / 512;
	ramdisk_sector_start = e->start + sectors_per_page;
	ramdisk_sector_start += CEIL(hdr->kernel_size, hdr->page_size)*
						sectors_per_page;
	ramdisk_sectors = CEIL(getsize, hdr->page_size)*
				       sectors_per_page;

	/* Make sure ramdisk image is not too large */
	if ((e->start + e->length/512) <
		(ramdisk_sector_start + ramdisk_sectors)) {
		sprintf(response, "FAILNew Ramdisk too large");
		return -1;
	}

	/* Change the boot img hdr */
	hdr->ramdisk_size = getsize;
	if (mmc_write(&mmc, e->start,
		hdr_sectors, (void *)hdr)) {
		sprintf(response, "FAILCannot writeback boot img hdr");
		return -1;
	}

	/* Write the new ramdisk image */
	if (mmc_write(&mmc, ramdisk_sector_start, ramdisk_sectors,
					transfer_buffer)) {
		sprintf(response, "FAILCannot write new ramdisk");
		return -1;
	}

	sprintf(response, "OKAY");
	return 0;
}


static int flash_non_sparse_formatted_image(void)
{
	int ret = 0;
	u32 num_sectors = 0;
	char response[65];

	num_sectors = CEIL(getsize, 512);

	if (num_sectors > (getsize / 512)) {
		/* do nothing */
	} else
		num_sectors = (getsize / 512);

	printf("writing to sector %d\n and # of sectors = %d\n", sector,
							num_sectors);

	ret = mmc_write(&mmc, sector, num_sectors, transfer_buffer);
	if (ret != 0) {
		printf("mmc write failed\n");
		return ret;
	}

	#ifdef DEBUG
		for (sector_count = 0; sector_count < num_sectors;
						sector_count++) {

			/*read back the data and compare */
			ret = mmc_read(&mmc, sector+sector_count, 1,
				read_buffer + (sector_count*512));
			if (ret != 0) {
				printf("mmc read failed\n");
				return ret;
			}

			if (memcmp(read_buffer + (sector_count*512),
				transfer_buffer + (sector_count*512), 512)) {
				printf("mmc data mismatch sector %d\n",
							sector+sector_count);
			}
		}
	#endif

	strcpy(response, "OKAY");
	fastboot_tx_status(response, strlen(response));

	return ret;
}

void do_fastboot(void)
{
	int ret = 0;
	char cmd[65];
	int cmdsize = 0;

	/* Use 65 instead of 64, null gets dropped
	strcpy's need the extra byte */
	char response[65];

	getsize = 0;

	/* enable irqs */
	enable_irqs();

	while (1) {

		cmdsize = 0;
		memset(&cmd, 0, 64);
		memset(&response, 0, 64);

		/* receive the fastboot command size from host*/
		ret = usb_read(&usb, &cmdsize, 4);
		if (ret < 0) {
			printf("failed to read the fastboot command size\n");
			strcpy(response, "FAIL");
			goto fail;
		}

		/* receive the fastboot command from host */
		ret = usb_read(&usb, &cmd, cmdsize);
		if (ret < 0) {
			printf("failed to read the fastboot command\n");
			strcpy(response, "FAIL");
			goto fail;
		}

		if (memcmp(cmd, "getvar:", 7) == 0) {

			strcpy(response, "OKAY");

			fastboot_getvar(cmd + 7, response);

		} /* getvar if loop ends */

		if (memcmp(cmd, "oem ", 4) == 0) {

			ret = 0;

			/* fastboot oem format */
			if (memcmp(cmd, "oem format", 6) == 0) {

				ret = fastboot_oem();
				if (ret != 0) {
					printf("fastboot_oem() failed\n");
					strcpy(response, "FAIL");
					goto fail;
				} else {
					strcpy(response, "OKAY");
					fastboot_tx_status(response,
							strlen(response));
				}

			} /* "oem format" if loop ends */

			else if (memcmp(cmd, "oem recovery", 8) == 0) {

				strcpy(response, "OKAY");

				fastboot_tx_status(response, strlen(response));

				/* close the usb connection */
				usb_close(&usb);

				/* Clear all reset reasons */
				writel(0xfff, PRM_RSTST);

				strcpy((char *)PUBLIC_SAR_RAM_1_FREE,
							"recovery");

				/* now warm reset the silicon */
				writel(PRM_RSTCTRL_RESET_WARM_BIT,
							PRM_RSTCTRL);
				/* we never return */
				while (1)
					;

			} /* "oem recovery if loop ends */

			else if (memcmp(cmd, "oem unlock", 6) == 0) {

				printf("\nfastboot oem unlock not implemented "
								"yet!\n");

				strcpy(response, "FAIL");

				fastboot_tx_status(response, strlen(response));

			} /* "oem unlock if loop ends */

			else {

				printf("\nfastboot: does not understand %s\n"\
									, cmd);
				strcpy(response, "FAIL");

				fastboot_tx_status(response, strlen(response));
			}

		} /* "oem" if loop ends */

		if (memcmp(cmd, "download:", 9) == 0) {

			ret = 0;
			dsize = &cmd[10];
			ret = download_image();
			if (ret != 0)
				goto fail;
			else
				printf("Finished downloading...\n");

		} /* "download" if loop ends */

		if (memcmp(cmd, "flash:", 6) == 0) {

			ret = 0;

			if (getsize == 0)
				goto fail;

			/* Init the MMC and load the partition table */
			ret = board_mmc_init();
			if (ret != 0) {
				printf("board_mmc_init() failed\n");
				sprintf(response,
					"FAILUnable to init the MMC");
				goto fail;
			}
			if ((memcmp(cmd+6, "zimage", 6) == 0) ||
				(memcmp(cmd+6, "zImage", 6) == 0)){
				fastboot_update_zimage(response);
				fastboot_tx_status(response, strlen(response));
				continue;
			} else if (memcmp(cmd+6, "ramdisk", 7) == 0) {
				fastboot_update_ramdisk(response);
				fastboot_tx_status(response, strlen(response));
				continue;
			}

			e = fastboot_flash_find_ptn(&cmd[6]);

			if (e == 0) {
				char ptn_name[20];
				strncpy(ptn_name, cmd+6, cmdsize-6);

				printf("Partition: %s does not exist\n",
								ptn_name);
				sprintf(response,
					"FAILpartition does not exist");
				fastboot_tx_status(response, strlen(response));
				continue;

			} else if (getsize > e->length) {
				printf("Image is too large for partition\n");
				sprintf(response, "FAILimage is too large for "
								"partition");
				fastboot_tx_status(response, strlen(response));
				continue;

			} else
				printf("writing to partition %s, begins at "
					"sector%d and is %d long \n", e->name,
							e->start, e->length);

			/* store the start address of the partition */
			sector = e->start;

			sparse_header = (sparse_header_t *) transfer_buffer;

			/*check if we have a sparse compressed image */
			if (sparse_header->magic == SPARSE_HEADER_MAGIC) {

				ret = flash_sparse_formatted_image();
				if (ret != 0)
					goto fail;
				else
					printf("Done flashing the sparse "
					"formatted image to %s\n", e->name);
			} else {
				/* normal flashing case */
				ret = flash_non_sparse_formatted_image();
				if (ret != 0)
					goto fail;
				else
					printf("Done flashing the non-sparse "
					"formatted image to %s\n", e->name);
			}

		} /* "flash" if loop ends */

		if (memcmp(cmd, "boot", 4) == 0) {

			char start[32];
			char *booti_args[4] = { "booti", NULL, "boot", NULL };
			booti_args[1] = start;
			sprintf(start, "0x%x", transfer_buffer);

			strcpy(response, "OKAY");
			fastboot_tx_status(response, strlen(response));

			printf("booting kernel...\n");

			do_booti((char *)booti_args);

		} /* "boot" if loop ends */

	} /* while(1) loop ends */

fail:
	/* send response back to host */
	fastboot_tx_status(response, strlen(response));
	printf("\nsomething bad happened\n");
	while (1)	/* stay here */
		;
} /* do_fastboot() ends */


#endif
