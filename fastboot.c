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
#include <aboot/bootimg.h>
#include <aboot/io.h>
#include <aboot/types.h>

#include <common/alloc.h>
#include <common/omap_rom.h>
#include <common/usbboot_common.h>

#include <libc/string.h>

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

#if defined CONFIG_FASTBOOT
#include <common/fastboot.h>

struct usb usb;
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

static int fastboot_tx_status(const char *buffer, unsigned int buffer_size)
{
	/* send response back to host */
	usb_write(&usb, (void *)buffer, strlen(buffer));
	return 0;
}

static void dev_to_devstr(u8 dev, char *devstr)
{
	switch (dev) {
	case DEVICE_EMMC:
		strcpy(devstr, "EMMC");
		break;
	case DEVICE_SDCARD:
		strcpy(devstr, "SD");
		break;
	default:
		strcpy(devstr, "Unknown");
		break;
	}
}

static int devstr_to_dev(const char *devstr, u8 *dev)
{
	int ret = 0;
	if (!strcmp(devstr, "EMMC"))
		*dev = DEVICE_EMMC;
	else if (!strcmp(devstr, "SD"))
		*dev = DEVICE_SDCARD;
	else
		ret = -1;
	return ret;
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

static int fastboot_getvar(const char *rx_buffer, char *tx_buffer)
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
		fastboot_tx_status(tx_buffer, strlen(tx_buffer));
		/* processor version */
		strcpy(tx_buffer, "INFO");
		strcpy(tx_buffer + strlen(tx_buffer), "cpu: ");
		strcpy(tx_buffer + strlen(tx_buffer), get_proc_version());
		fastboot_tx_status(tx_buffer, strlen(tx_buffer));
		/* cpu revision */
		strcpy(tx_buffer, "INFO");
		strcpy(tx_buffer + strlen(tx_buffer), "cpurev: ");
		strcpy(tx_buffer + strlen(tx_buffer), get_cpu_rev());
		fastboot_tx_status(tx_buffer, strlen(tx_buffer));
		/* device is GP/EMU/HS */
		strcpy(tx_buffer, "INFO");
		strcpy(tx_buffer + strlen(tx_buffer), "secure: ");
		strcpy(tx_buffer + strlen(tx_buffer), get_proc_type());
		fastboot_tx_status(tx_buffer, strlen(tx_buffer));
		strcpy(tx_buffer, "INFO");
		/*serial number */
		strcpy(tx_buffer + strlen(tx_buffer), "serialno: ");
		strcpy(tx_buffer + strlen(tx_buffer), get_serial_number());
		fastboot_tx_status(tx_buffer, strlen(tx_buffer));
		/* rom version */
		strcpy(tx_buffer, "INFO");
		strcpy(tx_buffer + strlen(tx_buffer), "rom version: ");
		strcpy(tx_buffer + strlen(tx_buffer), get_rom_version());
		fastboot_tx_status(tx_buffer, strlen(tx_buffer));

		strcpy(tx_buffer, "OKAY");
	} else
		printf("fastboot_getvar():unsupported variable\n");

	fastboot_tx_status(tx_buffer, strlen(tx_buffer));

	return 0;
}

static void fastboot_oem(struct fastboot_data *fb_data,
			const char *cmd,
			char *response)
{
	int ret = -1;
	u8 dev = 0;

	if (memcmp(cmd, "format", 6) == 0) {
		ret = do_gpt_format(fb_data);
		if (ret)
			strcpy(response, "FAIL");
		else
			strcpy(response, "OKAY");
	} else if (memcmp(cmd, "unlock", 6) == 0) {
		printf("\nfastboot oem unlock not implemented yet!\n");
		strcpy(response, "FAILNot Implemented");
	} else if (memcmp(cmd,  "set_flash_slot", 14) == 0) {
		ret = devstr_to_dev(cmd + 15, &dev);
		if (ret)
			strcpy(response, "FAILNot Supported");
		else {
			if (fb_data->board_ops->board_set_flash_slot) {
				if (fb_data->board_ops->
						board_set_flash_slot(dev))
					strcpy(response, "FAILNot Supported");
				else
					strcpy(response, "OKAY");
			} else
				strcpy(response, "FAILNot Supported");
		}
	} else {
		printf("\nfastboot: does not understand %s\n", cmd);
		strcpy(response, "FAILUnknown command");
	}
}

void fastboot_flash_add_ptn(fastboot_ptentry *ptn, int count)
{
	if (count < MAX_PTN) {
		memcpy(fb_data->fb_ptable + count, ptn, sizeof(*ptn));
	count++;
	}
}


fastboot_ptentry *fastboot_flash_find_ptn(const char *name)
{
    unsigned int n;

	for (n = 0; n < MAX_PTN; n++) {
		/* Make sure a substring is not accepted */
		if (strlen(name) == strlen(fb_data->fb_ptable[n].name)) {
			if (0 == strcmp(fb_data->fb_ptable[n].name, name))
				return fb_data->fb_ptable + n;
		}
	}

	return NULL;
}

fastboot_ptentry *fastboot_flash_get_ptn(unsigned int n, int count)
{
	if (n < count)
		return fb_data->fb_ptable + n;
	else
		return NULL;
}

static int download_image(void)
{
	int ret = 0;
	int size_of_dsize = 0;
	int count = 0;
	char response[65];

	size_of_dsize = strlen(fb_data->dsize);
	count = size_of_dsize;

	fb_data->getsize =
		get_downloadsize_from_string(size_of_dsize, fb_data->dsize);

	if (fb_data->getsize == 0) {
		sprintf(response, "FAILinvalid data size %x",
					fb_data->getsize);
		ret = -1;
		goto out;
	} else {
		sprintf(response, "DATA%08x", fb_data->getsize);
		fastboot_tx_status(response, strlen(response));
	}

	transfer_buffer = fastboot_alloc_mem();
	if (transfer_buffer == NULL) {
		sprintf(response, "FAILINVALID transfer_buffer");
		ret = -1;
		goto out;
	}

	/* read the data */
	printf("Reading %u amount of data ...\n", fb_data->getsize);
	ret = usb_read(&usb, transfer_buffer, fb_data->getsize);
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
			fastboot_tx_status(response, strlen(response));
		}
	}

	fastboot_tx_status(response, strlen(response));
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
	void *ptr_to_buffer = transfer_buffer;
	chunk_header_t *chunk_header;

	strcpy(response, "OKAY");

	if ((fb_data->sparse_header->total_blks * fb_data->sparse_header->blk_sz) > fb_data->e->length) {
		printf("Image size exceeds %d limit\n", fb_data->e->length);
		sprintf(response, "FAILImage size exceeds limit %d",
						(u32)fb_data->e->length);
		ret = -1;
		goto out;
	}

	if ((fb_data->sparse_header->major_version != 1) ||
		(fb_data->sparse_header->file_hdr_sz != sizeof(sparse_header_t)) ||
		(fb_data->sparse_header->chunk_hdr_sz != sizeof(chunk_header_t))) {
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

		if (fb_data->sparse_header->chunk_hdr_sz > sizeof(chunk_header_t)) {

			/* Skip the remaining bytes in a header that is longer
			than we	expected */
			transfer_buffer += (fb_data->sparse_header->chunk_hdr_sz -
						sizeof(chunk_header_t));
		}

		chunk_data_sz = (fb_data->sparse_header->blk_sz *
						chunk_header->chunk_sz);
		DBG("chunk_data_sz = %d\n", chunk_data_sz);

		num_sectors = (chunk_data_sz/512);
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
			int sector_count;

			read_buffer = fastboot_alloc_mem();
			if (read_buffer == NULL) {
				sprintf(response, "FAILINVALID read_buffer");
				ret = -1;
				goto out;
			}

			/*read back the data and compare */
			for (sector_count = 0; sector_count < num_sectors;
							sector_count++) {

				ret = fb_data->storage_ops->read
				(fb_data->sector + sector_count, 1,
					read_buffer + (sector_count*512));
				if (ret != 0) {
					printf("mmc read failed\n");
					strcpy(response, "FAILMMC read FAILED "
								"sparse");
					goto out;
				}

				if (memcmp(read_buffer + (sector_count*512),
				transfer_buffer + (sector_count*512), 512)) {
					printf("data mismatch sector %d\n",
					fb_data->sector + sector_count);

					strcpy(response, "INFO");
					sprintf(response + strlen(response),
						"data mismatch sector %d",
						fb_data->sector+sector_count);
					fastboot_tx_status(response,
							strlen(response));

				}
			}
			#endif

			total_blocks += chunk_header->chunk_sz;
			fb_data->sector += (chunk_data_sz/512);
			transfer_buffer += chunk_data_sz;
			break;

		case CHUNK_TYPE_DONT_CARE:

			total_blocks += chunk_header->chunk_sz;
			fb_data->sector += (chunk_data_sz/512);
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
			fb_data->sector += (chunk_data_sz/512);
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
		fastboot_tx_status(response, strlen(response));
	}

#ifdef DEBUG
	ret = fastboot_free_mem(read_buffer);
	if (ret != 0) {
		strcpy(response, "INFO");
		strcpy(response + strlen(response),
				"Unable to free read_buffer");
		fastboot_tx_status(response, strlen(response));
	}
#endif

	fastboot_tx_status(response, strlen(response));

	return ret;
}

static u32 fastboot_get_boot_ptn(boot_img_hdr *hdr, char *response)
{
	u32 hdr_sectors = 0;
	int ret = -1;

	strcpy(response, "OKAY");

	fb_data->e = fastboot_flash_find_ptn("boot");
	if (NULL == fb_data->e) {
		strcpy(response, "FAILCannot find boot partition");
		goto out;
	}

	/* Read the boot image header */
	hdr_sectors = CEIL(sizeof(struct boot_img_hdr), 512);
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
	fastboot_tx_status(response, strlen(response));
	return ret;
}

static int fastboot_update_zimage(char *response)
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

	hdr_sectors = fastboot_get_boot_ptn(hdr, response);
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
	ramdisk_buffer += hdr_sectors *512;
	if (fb_data->storage_ops->read(ramdisk_sector_start,
		ramdisk_sectors, ramdisk_buffer)) {
		sprintf(response, "FAILCannot read ramdisk from boot partition");
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
		fastboot_tx_status(response, strlen(response));
	}

	fastboot_tx_status(response, strlen(response));

	return ret;
}

static int fastboot_update_ramdisk(char *response)
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

	hdr_sectors = fastboot_get_boot_ptn(hdr, response);
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
	if ((fb_data->e->start + fb_data->e->length/512) <
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
		fastboot_tx_status(response, strlen(response));
	}

	fastboot_tx_status(response, strlen(response));

	return ret;
}


static int flash_non_sparse_formatted_image(void)
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
	int sector_count;

	read_buffer = fastboot_alloc_mem();
	if (read_buffer == NULL) {
		sprintf(response, "FAILINVALID read_buffer");
		ret = -1;
		goto out;
	}

	for (sector_count = 0; sector_count < num_sectors; sector_count++) {
		/*read back the data and compare */
		ret = fb_data->storage_ops->read(fb_data->sector +
			sector_count, 1, read_buffer + (sector_count*512));
		if (ret != 0) {
			printf("mmc read failed\n");
			strcpy(response, "FAILMMC read FAILED non sparse");
			goto out;
		}

		if (memcmp(read_buffer + (sector_count*512),
			transfer_buffer + (sector_count*512), 512)) {
			printf("data mismatch sector %d\n",
				fb_data->sector+sector_count);

			strcpy(response, "INFO");
			sprintf(response + strlen(response),
					"data mismatch sector %d",
					fb_data->sector+sector_count);
			fastboot_tx_status(response, strlen(response));
		}
	}
	#endif

out:
	ret = fastboot_free_mem(transfer_buffer);
	if (ret != 0) {
		strcpy(response, "INFO");
		strcpy(response + strlen(response),
				"Unable to free tranfer_buffer");
		fastboot_tx_status(response, strlen(response));
	}

#ifdef DEBUG
	ret = fastboot_free_mem(read_buffer);
	if (ret != 0) {
		strcpy(response, "INFO");
		strcpy(response + strlen(response),
				"Unable to free read_buffer");
		fastboot_tx_status(response, strlen(response));
	}
#endif

	fastboot_tx_status(response, strlen(response));

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

void do_fastboot(struct bootloader_ops *boot_ops)
{
	int ret = 0;
	char cmd[65];
	int cmdsize = 0;

	/* Use 65 instead of 64, null gets dropped
	strcpy's need the extra byte */
	char response[65];

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
		} else if (memcmp(cmd, "oem ", 4) == 0) {
			fastboot_oem(fb_data, cmd + 4, response);
			fastboot_tx_status(response, strlen(response));
		} else if (memcmp(cmd, "download:", 9) == 0) {

			ret = 0;
			fb_data->dsize = &cmd[10];
			ret = download_image();
			if (ret != 0)
				goto fail;
			else
				printf("Finished downloading...\n");

		} else if (memcmp(cmd, "flash:", 6) == 0) {

			ret = 0;

			if (fb_data->getsize == 0)
				goto fail;

			if ((memcmp(cmd+6, "zimage", 6) == 0) ||
				(memcmp(cmd+6, "zImage", 6) == 0)){
				fastboot_update_zimage(response);
				continue;
			} else if (memcmp(cmd+6, "ramdisk", 7) == 0) {
				fastboot_update_ramdisk(response);
				continue;
			}

			fb_data->e = fastboot_flash_find_ptn(&cmd[6]);

			if (fb_data->e == NULL) {
				char ptn_name[20];
				strncpy(ptn_name, cmd+6, cmdsize-6);

				printf("Partition: %s does not exist\n",
								ptn_name);
				sprintf(response,
					"FAILpartition does not exist");
				fastboot_tx_status(response, strlen(response));
				continue;

			} else if (fb_data->getsize > fb_data->e->length) {
				printf("Image is too large for partition\n");
				sprintf(response, "FAILimage is too large for "
								"partition");
				fastboot_tx_status(response, strlen(response));
				continue;

			} else
				printf("writing to partition %s, begins at "
					"sector: %d and is %d long\n",
					fb_data->e->name,
					(int)fb_data->e->start,
					(int)fb_data->e->length);

			/* store the start address of the partition */
			fb_data->sector = fb_data->e->start;

			fb_data->sparse_header = (sparse_header_t *)transfer_buffer;

			/*check if we have a sparse compressed image */
			if (fb_data->sparse_header->magic == SPARSE_HEADER_MAGIC) {

				ret = flash_sparse_formatted_image();
				if (ret != 0)
					goto fail;
				else
					printf("Done flashing the sparse "
					"formatted image to %s\n", fb_data->e->name);
			} else {
				/* normal flashing case */
				ret = flash_non_sparse_formatted_image();
				if (ret != 0)
					goto fail;
				else
					printf("Done flashing the non-sparse "
					"formatted image to %s\n", fb_data->e->name);
			}

		} else if (memcmp(cmd, "erase:", 6) == 0) {
			ret = 0;
			fb_data->e = fastboot_flash_find_ptn(&cmd[6]);
			if (fb_data->e == NULL)
				sprintf(response, "FAILPartition %s "
					"does not exist", &cmd[6]);
			else {
				ret = erase_section(fb_data->e->start, fb_data->e->length);
				if (ret)
					sprintf(response, "FAILUnable to "
					"erase partition %s", &cmd[6]);

				else
					sprintf(response, "OKAY");
			}
			fastboot_tx_status(response, strlen(response));
		}
		else if (memcmp(cmd, "boot", 4) == 0) {

			strcpy(response, "OKAY");
			fastboot_tx_status(response, strlen(response));

			usb_close(&usb);

			printf("booting kernel...\n");

			do_booti("ram", transfer_buffer);

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
