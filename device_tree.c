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

#include <libc/string.h>

#include <aboot/bootimg.h>

#include <common/boot_settings.h>
#include <common/device_tree.h>
#include <common/fastboot.h>
#include <common/alloc.h>

struct device_tree_data {
	struct fastboot_ptentry *pte;
	int dev_tree_sz;
	int dev_tree_load_addr;
	int page_size;
};

static struct device_tree_data *dt_data;

/**
 * DOC: Introduction
 * device_tree.c - Device tree support
**/

/**
 * find_dev_tree(void) - Find the device tree.
 *
 * This API will search multiple partitions looking for a device tree.
 * The search order is:
 * 1.  Look for a partition called device tree.  This would just be the
 *     device tree blob and will not have any associated header.  So whatever
 *     the contents is here is assumed to be a device tree.
 *
 * 2.  Look for an enviroment partition.  This partition may contain a device
 *     tree.  So if we find the partition there may or may not be a device tree
 *     contained within this partition.  Only if the device tree size is
 *     populated in the enviroment structure do we actually say we have a device
 *     tree.
 *
 * 3.  Finally look in the boot partition at the second data section.  The
 *     boot image may have contain a second data section.  Like the enviroment
 *     partition is the size is valid then we assume it to be device tree.
 *
 * Of course the code is not smart enough to actually tell if the data that
 * exists is a device tree.  But patches welcome for differentiation.
 *
 * Returns 0 if the device tree is found or -1 if no device tree is found.
 **/
static int find_dev_tree(struct bootloader_ops *boot_ops)
{
	struct fastboot_ptentry *pte;
	enviro_img_hdr *env_hdr;
	boot_img_hdr *boot_hdr;
	int ret = 0;
	u64 num_sectors = 0;
	int sector_sz = 0;
	u32 addr = CONFIG_ADDR_DOWNLOAD;

	sector_sz = boot_ops->storage_ops->get_sector_size();

	dt_data = (void *) alloc_memory(sizeof(struct device_tree_data));
	if (dt_data == NULL) {
		printf("unable to allocate memory requested: dt_data\n");
		return -1;
	}

	pte = fastboot_flash_find_ptn("device_tree");
	if (pte) {
		dt_data->pte = pte;
		dt_data->page_size = pte->length;
		dt_data->dev_tree_sz = pte->length;
		dt_data->dev_tree_load_addr = DEVICE_TREE;
		goto out;
	}

	pte = fastboot_flash_find_ptn("enviroment");
	if (pte) {
		env_hdr = (enviro_img_hdr *)addr;
		num_sectors =  sizeof(enviro_img_hdr) / sector_sz;
		if (num_sectors <= 0)
			num_sectors = 1;

		ret = boot_ops->storage_ops->read(pte->start, num_sectors,
							(void *) env_hdr);
		if (ret != 0) {
			printf("%s: failed to read enviroment header\n",
				__func__);
			goto out;
		}

		ret = memcmp(env_hdr->magic, ENVIRO_MAGIC, ENVIRO_MAGIC_SIZE);
		if (ret != 0) {
			printf("%s: bad enviroment magic\n", __func__);
			goto out;
		}

		if (env_hdr->dev_tree_size) {
			dt_data->pte = pte;
			dt_data->page_size = env_hdr->page_size;
			dt_data->dev_tree_sz = env_hdr->dev_tree_size;
			dt_data->dev_tree_load_addr = env_hdr->dev_tree_addr;
			goto out;
		}
		ret = -1;
		goto out;
	}

	pte = fastboot_flash_find_ptn("boot");
	if (pte) {
		boot_hdr = (boot_img_hdr *) addr;
		num_sectors =  sizeof(boot_img_hdr) / sector_sz;
		if (num_sectors <= 0)
			num_sectors = 1;
		ret = boot_ops->storage_ops->read(pte->start, num_sectors,
							(void *) boot_hdr);
		if (ret != 0) {
			printf("booti: failed to read bootimg header\n");
			goto out;
		}

		ret = memcmp(boot_hdr->magic, BOOT_MAGIC, BOOT_MAGIC_SIZE);
		if (ret != 0) {
			printf("booti: bad boot image magic\n");
			goto out;
		}

		if (boot_hdr->second_size) {
			dt_data->pte = pte;
			dt_data->page_size = boot_hdr->page_size;
			dt_data->dev_tree_sz = boot_hdr->second_size;
			dt_data->dev_tree_load_addr = boot_hdr->second_addr;
			goto out;
		}

		ret = -1;
		goto out;
	}


out:
	return ret;
}

/**
 * load_dev_tree(void) - Load the device tree if found.
 *
 * If a device tree is found within a partition then it is loaded into
 * the device tree load address.
 *
 * The load address depends on the partition it was found in:
 * Default is #define DEVICE_TREE for device_tree partition
 * where the device tree is just the compiled binary or undefined.
 * Otherwise the address is read from the associated header.
 *
 * Returns the load addres in memory of the device tree.
 **/
u32 load_dev_tree(struct bootloader_ops *boot_ops)
{
	int ret = 0;
	int sector;
	int num_sectors;
	int sector_sz = 0;
	u32 dt_load_addr;

	ret = find_dev_tree(boot_ops);
	if (ret < 0) {
		printf("%s: Device tree not supported\n", __func__);
		dt_data->dev_tree_load_addr = ATAGS_ARGS;
		goto out;
	}

	sector_sz = boot_ops->storage_ops->get_sector_size();
	sector = dt_data->pte->start + (dt_data->page_size / sector_sz);

	num_sectors = CEIL(dt_data->dev_tree_sz, sector_sz);
	if (num_sectors <= (dt_data->dev_tree_sz / sector_sz))
		num_sectors = (dt_data->dev_tree_sz / sector_sz);

	ret = boot_ops->storage_ops->read(sector, num_sectors,
					(void *)dt_data->dev_tree_load_addr);

	printf("dev_tree @ %08x (%d)\n",
		dt_data->dev_tree_load_addr,
		dt_data->dev_tree_sz);

out:
	if (dt_data->dev_tree_load_addr)
		dt_load_addr = dt_data->dev_tree_load_addr;
	else
		return -1;

	ret = free_memory(dt_data);
	if (ret != 0) {
		printf("unable to free memory allocated: dt_data\n");
		return ret;
	}

	return dt_load_addr;
}
