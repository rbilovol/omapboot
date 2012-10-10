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

#include <aboot.h>
#include <bootimg.h>
#include <io.h>
#include <types.h>

#include <alloc.h>
#include <omap_rom.h>
#include <usbboot_common.h>
#include <common_proc.h>

#include <string.h>

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

#if defined CONFIG_FASTBOOT
#include <fastboot.h>
#include <fastboot_common.h>

static struct fastboot_data fb_data_data;

static struct fastboot_data *fb_data = &fb_data_data;
static void *transfer_buffer;
static void *read_buffer;

static char *get_rom_version(void)
{
	if (!fb_data->proc_ops->proc_get_rom_version)
		return "not supported";

	return fb_data->proc_ops->proc_get_rom_version();
}

char *get_serial_number(void)
{
	if (!fb_data->proc_ops->proc_get_serial_num)
		return "not supported";

	return fb_data->proc_ops->proc_get_serial_num();
}

static char *get_proc_type(void)
{
	if (!fb_data->proc_ops->proc_get_type)
		return "not supported";

	return fb_data->proc_ops->proc_get_type();
}

static char *get_cpu_rev(void)
{
	if (!fb_data->proc_ops->proc_get_revision)
		return "not supported";

	return fb_data->proc_ops->proc_get_revision();
}

static char *get_proc_version(void)
{
	if (!fb_data->proc_ops->proc_get_version)
		return "not supported";

	return fb_data->proc_ops->proc_get_version();
}

static int fastboot_tx_status(const char *buffer, unsigned int buffer_size,
								struct usb *usb)
{
	/* send response back to host */
	usb_write(usb, (void *)buffer, strlen(buffer));

	return 0;
}

static int *fastboot_alloc_mem(void)
{
	void *mem_ptr;

	mem_ptr = (void *) alloc_memory(fb_data->getsize);
	if (mem_ptr != NULL)
		return mem_ptr;
	else {
		printf("unable to allocate memory requested\n");
		return NULL;
	}
}

#ifdef DEBUG
static void debug_compare(u8 *tb, u32 sector, u32 sectors, int sector_size)
{
	int i, ret;
	u8 *rbp, *tbp;
	u8 *rb;
	rb = (u8 *) alloc_memory(sectors * sector_size);
	if (!rb) {
		printf("%s: Mem Alloc failed\n", __func__);
		return;
	}
	ret = fb_data->storage_ops->read(sector, sectors, rb);
	if (ret) {
		printf("%s: Storage Read failed\n");
		goto compare_fail;
	}
	rbp = rb;
	tbp = tb;
	for (i = 0; i < sectors; i++) {
		if (memcmp(rbp, tbp, sector_size)) {
			printf("Compare fail at sector: %d\n",
				sector + i);
			goto compare_fail;
		}
		rbp += sector_size;
		tbp += sector_size;
	}
	printf("Compare sector: %d, sectors:%d Success\n", sector, sectors);
compare_fail:
	free_memory(rb);
}
#endif

static int fastboot_free_mem(void *mem_ptr)
{
	int ret = 0;

	ret = free_memory(mem_ptr);
	if (ret != 0) {
		printf("unable to free memory allocated\n");
		return ret;
	} else
		return 0;
}

static int fastboot_getvar(const char *rx_buffer, char *tx_buffer,
								struct usb *usb)
{
	strcpy(tx_buffer, "OKAY");

	if (!memcmp(rx_buffer, "version-bootloader", 18))
		strcpy(tx_buffer + 4, ABOOT_VERSION);
	else if (!memcmp(rx_buffer, "version", 7))
		strcpy(tx_buffer + 4, FASTBOOT_VERSION);
	else if (!memcmp(rx_buffer, "romversion", 10))
		strcpy(tx_buffer + 4, get_rom_version());
	else if (!memcmp(rx_buffer, "product", 7))
		strcpy(tx_buffer + 4, PRODUCT_NAME);
	else if (!memcmp(rx_buffer, "serialno", 8)) {
		strcpy(tx_buffer + 4, get_serial_number());
	} else if (!memcmp(rx_buffer, "cpurev", 6)) {
		strcpy(tx_buffer + 4, get_cpu_rev());
	} else if (!memcmp(rx_buffer, "secure", 6)) {
		strcpy(tx_buffer + 4, get_proc_type());
	} else if (!memcmp(rx_buffer, "cpu", 3)) {
		strcpy(tx_buffer + 4, get_proc_version());
	} else if (!memcmp(rx_buffer, "pmicrev", 7)) {
		strcpy(tx_buffer + 4, pmic_get_silicon_revision());
	} else if (!memcmp(rx_buffer, "downloadsize", 12)) {
		if (fb_data->getsize)
			sprintf(tx_buffer + 4, "%08x", fb_data->getsize);
	} else if (!strcmp(rx_buffer, "userdata_size")) {
		strcpy(tx_buffer + 4, get_ptn_size(fb_data,
		     tx_buffer + strlen(tx_buffer), "userdata"));
	} else if (!strcmp(rx_buffer, "flash_slot")) {
		if (fb_data->board_ops->board_set_flash_slot) {
			dev_to_devstr(fb_data->board_ops->
				board_get_flash_slot(), tx_buffer + 4);
		}
	} else if (!strcmp(rx_buffer, "all")) {
		/* product name */
		strcpy(tx_buffer, "INFO");
		strcpy(tx_buffer + strlen(tx_buffer), "product: ");
		strcpy(tx_buffer + strlen(tx_buffer), PRODUCT_NAME);
		fastboot_tx_status(tx_buffer, strlen(tx_buffer), usb);
		/* processor version */
		strcpy(tx_buffer, "INFO");
		strcpy(tx_buffer + strlen(tx_buffer), "cpu: ");
		strcpy(tx_buffer + strlen(tx_buffer), get_proc_version());
		fastboot_tx_status(tx_buffer, strlen(tx_buffer), usb);
		/* cpu revision */
		strcpy(tx_buffer, "INFO");
		strcpy(tx_buffer + strlen(tx_buffer), "cpurev: ");
		strcpy(tx_buffer + strlen(tx_buffer), get_cpu_rev());
		fastboot_tx_status(tx_buffer, strlen(tx_buffer), usb);
		/* device is GP/EMU/HS */
		strcpy(tx_buffer, "INFO");
		strcpy(tx_buffer + strlen(tx_buffer), "secure: ");
		strcpy(tx_buffer + strlen(tx_buffer), get_proc_type());
		fastboot_tx_status(tx_buffer, strlen(tx_buffer), usb);
		strcpy(tx_buffer, "INFO");
		/*serial number */
		strcpy(tx_buffer + strlen(tx_buffer), "serialno: ");
		strcpy(tx_buffer + strlen(tx_buffer), get_serial_number());
		fastboot_tx_status(tx_buffer, strlen(tx_buffer), usb);
		/* rom version */
		strcpy(tx_buffer, "INFO");
		strcpy(tx_buffer + strlen(tx_buffer), "rom version: ");
		strcpy(tx_buffer + strlen(tx_buffer), get_rom_version());
		fastboot_tx_status(tx_buffer, strlen(tx_buffer), usb);
		/* pmic silicon revision */
		strcpy(tx_buffer, "INFO");
		strcpy(tx_buffer + strlen(tx_buffer), "pmicrev: ");
		strcpy(tx_buffer + strlen(tx_buffer),
						pmic_get_silicon_revision());
		fastboot_tx_status(tx_buffer, strlen(tx_buffer), usb);

		strcpy(tx_buffer, "OKAY");
	} else {
		DBG("fastboot_getvar():unsupported variable\n");
		sprintf(tx_buffer, "FAILUnsupported Variable %s", rx_buffer);
	}

	fastboot_tx_status(tx_buffer, strlen(tx_buffer), usb);

	return 0;
}

static int fastboot_oem(const char *cmd, char *response, struct usb *usb)
{
	int ret = -1;
	u8 dev = 0;

	if (memcmp(cmd, "format", 6) == 0) {
		ret = do_gpt_format(fb_data);
		if (ret)
			strcpy(response, "FAILGPT format failed");
		else
			strcpy(response, "OKAY");

	} else if (memcmp(cmd, "unlock", 6) == 0) {
		DBG("\nfastboot oem unlock not implemented yet!\n");
		strcpy(response, "FAILoem unlock not implemented");

	} else if (memcmp(cmd,  "set_flash_slot", 14) == 0) {
		ret = devstr_to_dev(cmd + 15, &dev);
		if (ret)
			strcpy(response, "FAILNot Supported");
		else {
			if (fb_data->board_ops->board_set_flash_slot) {
				fb_data->storage_ops =
				fb_data->board_ops->board_set_flash_slot(dev,
				fb_data->proc_ops, fb_data->storage_ops);
				if (!fb_data->storage_ops)
					strcpy(response, "FAILUnable to set "
								"flash slot");
				else
					strcpy(response, "OKAY");

			} else
				strcpy(response, "FAILboard_set_flash_slot not "
								"supported");
		}
	} else {
		DBG("\nfastboot: does not understand %s\n", cmd);
		sprintf(response, "FAILUnknown oem command %s", cmd);
	}

	fastboot_tx_status(response, strlen(response), usb);

	return ret;
}

static int download_image(char *dsize, char *response, struct usb *usb)
{
	int ret = 0;
	int size_of_dsize = 0;

	size_of_dsize = strlen(dsize);

	fb_data->getsize =
		get_downloadsize_from_string(size_of_dsize, dsize);

	if (fb_data->getsize == 0) {
		sprintf(response, "FAILinvalid data size %x",
					fb_data->getsize);
		ret = -1;
		goto out;
	} else {
		sprintf(response, "DATA%08x", fb_data->getsize);
		fastboot_tx_status(response, strlen(response), usb);
	}

	transfer_buffer = fastboot_alloc_mem();
	if (transfer_buffer == NULL) {
		sprintf(response, "FAILINVALID transfer_buffer");
		ret = -1;
		goto out;
	}

	/* read the data */
	printf("Reading %u amount of data ...\n", fb_data->getsize);
	ret = usb_read(usb, transfer_buffer, fb_data->getsize);
	if (ret < 0) {
		DBG("failed to read the fastboot command\n");
		strcpy(response, "FAILUnable to read FASTBOOT command");
		goto out;
	} else
		strcpy(response, "OKAY");

out:
	if (ret < 0) {
		ret = fastboot_free_mem(transfer_buffer);
		if (ret != 0) {
			strcpy(response, "INFO");
			strcpy(response + strlen(response),
					"Unable to free tranfer_buffer");
			fastboot_tx_status(response, strlen(response), usb);
		}
	}

	fastboot_tx_status(response, strlen(response), usb);

	return ret;
}

static int flash_sparse_formatted_image(struct usb *usb)
{
	int ret = 0;
	u32 chunk = 0;
	u32 chunk_data_sz = 0;
	u32 num_sectors = 0;
	u32 out_blocks = 0;
	u32 total_blocks = 0;
	int sector_size;
	char response[65];
	void *ptr_to_buffer = transfer_buffer;
	chunk_header_t *chunk_header;

	strcpy(response, "OKAY");

	if ((fb_data->sparse_header->total_blks *
			fb_data->sparse_header->blk_sz) > fb_data->e->length) {
		printf("Image size exceeds %d limit\n", fb_data->e->length);
		sprintf(response, "FAILImage size exceeds limit %d",
						(u32)fb_data->e->length);
		ret = -1;
		goto out;
	}

	if ((fb_data->sparse_header->major_version != 1) ||
		(fb_data->sparse_header->file_hdr_sz !=
						sizeof(sparse_header_t)) ||
		(fb_data->sparse_header->chunk_hdr_sz !=
						sizeof(chunk_header_t))) {
			printf("Invalid sparse format\n");
			strcpy(response, "FAILINVALID sparse format");
			ret = -1;
			goto out;
	}

	/* Read and skip over sparse image header */
	transfer_buffer += fb_data->sparse_header->file_hdr_sz;

	if (fb_data->sparse_header->file_hdr_sz > sizeof(sparse_header_t)) {
		/* Skip the remaining bytes in a header
		that is longer than we expected */
		transfer_buffer += (fb_data->sparse_header->file_hdr_sz -
					sizeof(sparse_header_t));
	}

	DBG("=== Sparse Image Header ===\n");
	DBG("sizeof(sparse_header_t) = %d\n", sizeof(sparse_header_t));
	DBG("magic: 0x%x\n", fb_data->sparse_header->magic);
	DBG("major_version: 0x%x\n", fb_data->sparse_header->major_version);
	DBG("minor_version: 0x%x\n", fb_data->sparse_header->minor_version);
	DBG("file_hdr_sz: %d\n", fb_data->sparse_header->file_hdr_sz);
	DBG("chunk_hdr_sz: %d\n", fb_data->sparse_header->chunk_hdr_sz);
	DBG("blk_sz: %d\n", fb_data->sparse_header->blk_sz);
	DBG("total_blks: %d\n", fb_data->sparse_header->total_blks);
	DBG("total_chunks: %d\n", fb_data->sparse_header->total_chunks);

	/* Start processing chunks */
	sector_size = fb_data->storage_ops->get_sector_size();
	for (chunk = 0; chunk < fb_data->sparse_header->total_chunks; chunk++) {

		/* Read and skip over chunk header */
		chunk_header = (chunk_header_t *) transfer_buffer;
		transfer_buffer += sizeof(chunk_header_t);

		DBG("=== Chunk Header ===\n");
		DBG("sizeof(sparse_header_t) = %d"
					"\n", sizeof(chunk_header_t));
		DBG("chunk_type: 0x%x\n", chunk_header->chunk_type);
		DBG("chunk_sz: 0x%x\n", chunk_header->chunk_sz);
		DBG("total_sz: 0x%x\n", chunk_header->total_sz);

		if (fb_data->sparse_header->chunk_hdr_sz >
						sizeof(chunk_header_t)) {

			/* Skip the remaining bytes in a header that is longer
			than we	expected */
			transfer_buffer +=
					(fb_data->sparse_header->chunk_hdr_sz-
							sizeof(chunk_header_t));
		}

		chunk_data_sz = (fb_data->sparse_header->blk_sz *
						chunk_header->chunk_sz);
		DBG("chunk_data_sz = %d\n", chunk_data_sz);

		num_sectors = (chunk_data_sz/sector_size);
		DBG("writing to sector %d and # of sectors %d\n",
			fb_data->sector, num_sectors);

		switch (chunk_header->chunk_type) {

		case CHUNK_TYPE_RAW:

			if (chunk_header->total_sz !=
				(fb_data->sparse_header->chunk_hdr_sz +
							chunk_data_sz)) {
				DBG("bogus chunk size for chunk type RAW "
							"%d\n", chunk);
			}

			out_blocks += chunk_data_sz;

			ret = fb_data->storage_ops->write(fb_data->sector,
					num_sectors, transfer_buffer);
			if (ret != 0) {
				printf("mmc write failed\n");
				strcpy(response, "FAILMMC write FAILED "
								"sparse");
				goto out;
			}
#ifdef DEBUG
			debug_compare(transfer_buffer, fb_data->sector,
						num_sectors, sector_size);
#endif
			total_blocks += chunk_header->chunk_sz;
			fb_data->sector += (chunk_data_sz / sector_size);
			transfer_buffer += chunk_data_sz;
			break;

		case CHUNK_TYPE_DONT_CARE:

			total_blocks += chunk_header->chunk_sz;
			fb_data->sector += (chunk_data_sz / sector_size);
			DBG("bogus chunk size for chunktype DONT_CARE %d\n",
									chunk);
			out_blocks += chunk_data_sz;

			break;

		case CHUNK_TYPE_CRC:

			if (chunk_header->total_sz !=
					fb_data->sparse_header->chunk_hdr_sz) {
				DBG("bogus chunk size for chunktype CRC %d"
								"\n", chunk);
			}

			total_blocks += chunk_header->chunk_sz;
			fb_data->sector += (chunk_data_sz / sector_size);
			transfer_buffer += chunk_data_sz;
			out_blocks += chunk_data_sz;

			break;

		default:

			sprintf(response, "FAILUnknown chunk type");
			ret = -1;
			break;

		goto out;
		}
	}

	DBG("Wrote %d blocks, expected to write %d blocks\n", total_blocks,
		fb_data->sparse_header->total_blks);
	DBG("out blocks = %d\n", out_blocks);

out:
	/* reset the pointer to the top */
	transfer_buffer = ptr_to_buffer;

	ret = fastboot_free_mem(transfer_buffer);
	if (ret != 0) {
		strcpy(response, "INFO");
		strcpy(response + strlen(response),
				"Unable to free tranfer_buffer");
		fastboot_tx_status(response, strlen(response), usb);
	}

	fastboot_tx_status(response, strlen(response), usb);

	return ret;
}

static u32 fastboot_get_boot_ptn(boot_img_hdr *hdr, char *response,
								struct usb *usb)
{
	u32 hdr_sectors = 0;
	int ret = -1;
	u32 sector_size;

	strcpy(response, "OKAY");

	fb_data->e = fastboot_flash_find_ptn("boot");
	if (NULL == fb_data->e) {
		strcpy(response, "FAILCannot find boot partition");
		goto out;
	}

	/* Read the boot image header */
	sector_size = fb_data->storage_ops->get_sector_size();
	hdr_sectors = CEIL(sizeof(struct boot_img_hdr), sector_size);
	if (fb_data->storage_ops->read(fb_data->e->start,
			hdr_sectors, (void *)hdr)) {
		strcpy(response, "FAILCannot read hdr from boot partition");
		goto out;
	}

	if (memcmp(hdr->magic, "ANDROID!", 8) != 0) {
		printf("booti: bad boot image magic\n");
		strcpy(response, "FAILBoot partition not initialized");
		goto out;
	}

	return hdr_sectors;

out:
	strcpy(response, "INFO");
	fastboot_tx_status(response, strlen(response), usb);

	return ret;
}

static int fastboot_update_zimage(char *response, struct usb *usb)
{
	boot_img_hdr *hdr = NULL;
	u8 *ramdisk_buffer;
	u32 ramdisk_sector_start, ramdisk_sectors;
	u32 kernel_sector_start, kernel_sectors;
	u32 hdr_sectors = 0;
	u32 sectors_per_page = 0;
	int ret = 0;

	strcpy(response, "OKAY");

	read_buffer = fastboot_alloc_mem();
	if (read_buffer == NULL) {
		strcpy(response, "FAILINVALID read_buffer");
		ret = -1;
		goto out;
	}

	hdr = (boot_img_hdr *) read_buffer;

	hdr_sectors = fastboot_get_boot_ptn(hdr, response, usb);
	if (hdr_sectors <= 0) {
		sprintf(response + strlen(response),
			"FAILINVALID number of boot sectors %d", hdr_sectors);
		ret = -1;
		goto out;
	}

	/* Extract ramdisk location and read it into local buffer */
	sectors_per_page = hdr->page_size / 512;
	ramdisk_sector_start = fb_data->e->start + sectors_per_page;
	ramdisk_sector_start += CEIL(hdr->kernel_size, hdr->page_size)*
						sectors_per_page;
	ramdisk_sectors = CEIL(hdr->ramdisk_size, hdr->page_size)*
						sectors_per_page;

	ramdisk_buffer = (u8 *)hdr;
	ramdisk_buffer += (hdr_sectors * 512);
	if (fb_data->storage_ops->read(ramdisk_sector_start,
		ramdisk_sectors, ramdisk_buffer)) {
		sprintf(response, "FAILCannot read ramdisk from boot "
								"partition");
		ret = -1;
		goto out;
	}

	/* Change the boot img hdr */
	hdr->kernel_size = fb_data->getsize;
	if (fb_data->storage_ops->write(fb_data->e->start,
		hdr_sectors, (void *)hdr)) {
		sprintf(response, "FAILCannot writeback boot img hdr");
		ret = -1;
		goto out;
	}

	/* Write the new downloaded kernel*/
	kernel_sector_start = fb_data->e->start + sectors_per_page;
	kernel_sectors = CEIL(hdr->kernel_size, hdr->page_size)*
					sectors_per_page;
	if (fb_data->storage_ops->write(kernel_sector_start, kernel_sectors,
			transfer_buffer)) {
		sprintf(response, "FAILCannot write new kernel");
		ret = -1;
		goto out;
	}

	/* Write the saved Ramdisk back */
	ramdisk_sector_start = fb_data->e->start + sectors_per_page;
	ramdisk_sector_start += CEIL(hdr->kernel_size, hdr->page_size)*
						sectors_per_page;
	if (fb_data->storage_ops->write(ramdisk_sector_start, ramdisk_sectors,
						ramdisk_buffer)) {
		sprintf(response, "FAILCannot write back original ramdisk");
		ret = -1;
		goto out;
	}

out:
	ret = fastboot_free_mem(read_buffer);
	if (ret != 0) {
		strcpy(response, "INFO");
		strcpy(response + strlen(response),
				"Unable to free read_buffer");
		fastboot_tx_status(response, strlen(response), usb);
	}

	fastboot_tx_status(response, strlen(response), usb);

	return ret;
}

static int fastboot_update_ramdisk(char *response, struct usb *usb)
{
	boot_img_hdr *hdr = NULL;
	u32 ramdisk_sector_start, ramdisk_sectors;
	u32 hdr_sectors = 0;
	u32 sectors_per_page = 0;
	int ret = 0;

	strcpy(response, "OKAY");

	read_buffer = fastboot_alloc_mem();
	if (read_buffer == NULL) {
		strcpy(response, "FAILINVALID read_buffer");
		ret = -1;
		goto out;
	}

	hdr = (boot_img_hdr *) read_buffer;

	hdr_sectors = fastboot_get_boot_ptn(hdr, response, usb);
	if (hdr_sectors <= 0) {
		sprintf(response + strlen(response),
			"FAILINVALID number of boot sectors %d", hdr_sectors);
		ret = -1;
		goto out;
	}

	/* Calculate ramdisk location */
	sectors_per_page = hdr->page_size / 512;
	ramdisk_sector_start = fb_data->e->start + sectors_per_page;
	ramdisk_sector_start += CEIL(hdr->kernel_size, hdr->page_size)*
						sectors_per_page;
	ramdisk_sectors = CEIL(fb_data->getsize, hdr->page_size)*
				       sectors_per_page;

	/* Make sure ramdisk image is not too large */
	if ((fb_data->e->start + fb_data->e->length / 512) <
		(ramdisk_sector_start + ramdisk_sectors)) {
		sprintf(response, "FAILNew Ramdisk too large");
		ret = -1;
		goto out;
	}

	/* Change the boot img hdr */
	hdr->ramdisk_size = fb_data->getsize;
	if (fb_data->storage_ops->write(fb_data->e->start,
		hdr_sectors, (void *)hdr)) {
		sprintf(response, "FAILCannot writeback boot img hdr");
		ret = -1;
		goto out;
	}

	/* Write the new ramdisk image */
	if (fb_data->storage_ops->write(ramdisk_sector_start, ramdisk_sectors,
					transfer_buffer)) {
		sprintf(response, "FAILCannot write new ramdisk");
		ret = -1;
		goto out;
	}

out:
	ret = fastboot_free_mem(read_buffer);
	if (ret != 0) {
		strcpy(response, "INFO");
		strcpy(response + strlen(response),
				"Unable to free read_buffer");
		fastboot_tx_status(response, strlen(response), usb);
	}

	fastboot_tx_status(response, strlen(response), usb);

	return ret;
}


static int flash_non_sparse_formatted_image(struct usb *usb)
{
	int ret = 0;
	u32 num_sectors = 0;
	char response[65];

	strcpy(response, "OKAY");

	num_sectors = CEIL(fb_data->getsize, 512);

	if (num_sectors > (fb_data->getsize / 512)) {
		/* do nothing */
	} else
		num_sectors = (fb_data->getsize / 512);

	DBG("writing to sector %d\n and # of sectors = %d\n",
		(int)fb_data->sector, (int)num_sectors);

	ret = fb_data->storage_ops->write(fb_data->sector,
			num_sectors, transfer_buffer);
	if (ret != 0) {
		printf("mmc write failed\n");
		strcpy(response, "FAILMMC write FAILED non sparse");
		goto out;
	}

#ifdef DEBUG
	debug_compare(transfer_buffer, fb_data->sector, num_sectors, 512);
#endif
out:
	ret = fastboot_free_mem(transfer_buffer);
	if (ret != 0) {
		strcpy(response, "INFO");
		strcpy(response + strlen(response),
				"Unable to free tranfer_buffer");
		fastboot_tx_status(response, strlen(response), usb);
	}

	fastboot_tx_status(response, strlen(response), usb);

	return ret;
}

static int erase_section(unsigned int start, u64 length)
{
	u64 sectors = 0;
	int ret=0;

	sectors = length / 512;

	ret = fb_data->storage_ops->erase((u64)start, sectors);
	return ret;
}

static int fastboot_flash(char *cmd, char *response, struct usb *usb)
{
	int ret = 0;

	if ((memcmp(cmd, "zimage", 6) == 0) ||
		(memcmp(cmd, "zImage", 6) == 0)) {
		ret = fastboot_update_zimage(response, usb);

		return ret;

	} else if (memcmp(cmd, "ramdisk", 7) == 0) {
		ret = fastboot_update_ramdisk(response, usb);

		return ret;
	}

	fb_data->e = fastboot_flash_find_ptn(cmd);

	if (fb_data->e == NULL) {
		char ptn_name[20];
		strncpy(ptn_name, cmd, strlen(cmd));

		DBG("Partition: %s does not exist\n", ptn_name);
		sprintf(response, "FAILpartition does not exist %s", ptn_name);
		fastboot_tx_status(response, strlen(response), usb);

		return 0;

	} else if (fb_data->getsize > fb_data->e->length) {
		DBG("Image is too large for partition\n");
		sprintf(response, "FAILimage is too large for partition");
		fastboot_tx_status(response, strlen(response), usb);

		return 0;

	} else
		printf("writing to partition %s, begins at " \
			"sector: %d and is %d long\n",
			(char *) fb_data->e->name,
			(int)fb_data->e->start,
			(int)fb_data->e->length);

	/* store the start address of the partition */
	fb_data->sector = fb_data->e->start;

	fb_data->sparse_header = (sparse_header_t *)transfer_buffer;

	/*check if we have a sparse compressed image */
	if (fb_data->sparse_header->magic == SPARSE_HEADER_MAGIC) {

		ret = flash_sparse_formatted_image(usb);
		if (ret != 0)
			return -1;
		else
			printf("Done flashing the sparse " \
			"formatted image to %s\n", (char *) fb_data->e->name);
	} else {
		/* normal flashing case */
		ret = flash_non_sparse_formatted_image(usb);
		if (ret != 0)
			return -1;
		else
			printf("Done flashing the non-sparse " \
			"formatted image to %s\n", (char *) fb_data->e->name);
	}

	return ret;
}

static int fastboot_boot(struct bootloader_ops *boot_ops, char *cmd,
						char *response)
{
	struct usb *usb = &boot_ops->usb;
	strcpy(response, "OKAY");
	fastboot_tx_status(response, strlen(response), usb);

	usb_close(usb);

	printf("booting kernel...\n");
	do_booti(boot_ops, "ram", transfer_buffer);

	return 0;
}

static int fastboot_erase(char *cmd, char *response, struct usb *usb)
{
	int ret = 0;

	fb_data->e = fastboot_flash_find_ptn(cmd);
	if (fb_data->e == NULL)
		sprintf(response, "FAILPartition %s does not exist", cmd);
	else {
		ret = erase_section(fb_data->e->start, fb_data->e->length);
		if (ret)
			printf(response, "FAILUnable to "
						"erase partition %s", cmd);

		else
			sprintf(response, "OKAY");
	}

	fastboot_tx_status(response, strlen(response), usb);

	return ret;
}

void do_fastboot(struct bootloader_ops *boot_ops)
{
	int ret = 0;
	char cmd[65];

	int cmdsize = 0;
	/* Use 65 instead of 64, null gets dropped
	strcpy's need the extra byte */
	char response[65];
	struct usb *usb = &boot_ops->usb;

	fb_data->getsize = 0;

	if (!boot_ops->proc_ops)
		return;
	else
		fb_data->proc_ops = boot_ops->proc_ops;

	if (!boot_ops->board_ops)
		return;
	else
		fb_data->board_ops = boot_ops->board_ops;

	if (!boot_ops->storage_ops)
		return;
	else
		fb_data->storage_ops = boot_ops->storage_ops;

	load_ptbl(fb_data->storage_ops, 1);

	/* enable irqs */
	enable_irqs();

	serial_puts("Entering fastboot mode...\n");

	while (1) {

		cmdsize = 64;
		memset(&cmd, 0, 64);
		memset(&response, 0, 64);

		/* omap4 rom_usb required an extra step in the exchange:
			a request for the size to transfer.
			Without this step, the rom code will hang.
			This requires to use an older fastboot host
			utility were this ws implemented (DB72) */

		if (boot_ops->board_ops->board_fastboot_size_request) {
			ret = boot_ops->board_ops->
			board_fastboot_size_request(usb, &cmdsize, 4);
			if (ret < 0) {
				printf("failed to get fastboot command size\n");
				strcpy(response, "FAIL");
				goto fail;
			}
		}

		/* receive the fastboot command from host */
		ret = usb_read(usb, &cmd, cmdsize);
		if (ret < 0) {
			printf("failed to read the fastboot command\n");
			strcpy(response, "FAIL");
			goto fail;
		}

		if (memcmp(cmd, "getvar:", 7) == 0) {
			ret = fastboot_getvar(cmd + 7, response, usb);
		} else if (memcmp(cmd, "oem ", 4) == 0) {
			ret = fastboot_oem(cmd + 4, response, usb);
		} else if (memcmp(cmd, "download:", 9) == 0) {
			ret = download_image(cmd + 9, response, usb);
		} else if (memcmp(cmd, "flash:", 6) == 0) {
			ret = fastboot_flash(cmd + 6, response, usb);
		} else if (memcmp(cmd, "erase:", 6) == 0) {
			ret = fastboot_erase(cmd + 6, response, usb);
		} else if (memcmp(cmd, "boot", 4) == 0) {
			ret = fastboot_boot(boot_ops, cmd + 4, response);
		}

		if (ret < 0)
			goto fail;

	} /* while(1) loop ends */

fail:
	/* send response back to host */
	fastboot_tx_status(response, strlen(response), usb);
	printf("\nsomething bad happened\n");
	while (1)	/* stay here */
		;
} /* do_fastboot() ends */
#endif
