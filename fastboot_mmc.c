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
#include <common.h>
#include <io.h>
#include <types.h>
#include <alloc.h>

#include <string.h>

#include <common_proc.h>
#include <omap_rom.h>

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

#if defined CONFIG_FASTBOOT
#include <fastboot.h>

static struct partition *partitions;

static void init_mbr(u8 *mbr, u64 blocks)
{
	DBG("init_mbr\n");

	mbr[0x1be] = 0x00; /* nonbootable */
	mbr[0x1bf] = 0xFF; /* bogus CHS */
	mbr[0x1c0] = 0xFF;
	mbr[0x1c1] = 0xFF;

	mbr[0x1c2] = 0xEE; /* GPT partition */
	mbr[0x1c3] = 0xFF; /* bogus CHS */
	mbr[0x1c4] = 0xFF;
	mbr[0x1c5] = 0xFF;

	mbr[0x1c6] = 0x01; /* start */
	mbr[0x1c7] = 0x00;
	mbr[0x1c8] = 0x00;
	mbr[0x1c9] = 0x00;

	blocks = (blocks > 0xFFFFFFFF) ? 0xFFFFFFFF : blocks;
	memcpy(mbr + 0x1ca, &blocks, sizeof(u32));

	mbr[0x1fe] = 0x55;
	mbr[0x1ff] = 0xaa;
}

static void start_ptbl(struct ptable *ptbl, u64 blocks)
{
	struct efi_header *hdr = &ptbl->header;

	DBG("start_ptbl\n");

	memset(ptbl, 0, sizeof(*ptbl));

	init_mbr(ptbl->mbr, blocks - 1);

	memcpy(hdr->magic, "EFI PART", 8);
	hdr->version = EFI_VERSION;
	hdr->header_sz = sizeof(struct efi_header);
	hdr->crc32 = 0;
	hdr->reserved = 0;
	hdr->header_lba = 1;
	hdr->backup_lba = blocks - 1;
	hdr->first_lba = 34;
	hdr->last_lba = blocks - 1;
	memcpy(hdr->volume_uuid, random_uuid, 16);
	hdr->entries_lba = 2;
	hdr->entries_count = EFI_ENTRIES;
	hdr->entries_size = sizeof(struct efi_entry);
	hdr->entries_crc32 = 0;

	DBG("magic		= %s \n",  hdr->magic);
	DBG("version		= %u \n",  hdr->version);
	DBG("header_sz	= %u \n",  hdr->header_sz);
	DBG("crc32		= %u \n",  hdr->crc32);
	DBG("reserved	= %u \n",  hdr->reserved);
	DBG("header_lba	= %u \n",  hdr->header_lba);
	DBG("backup_lba	= %u \n",  hdr->backup_lba);
	DBG("first_lba	= %u \n",  hdr->first_lba);
	DBG("last_lba	= %u \n",  hdr->last_lba);
	DBG("entries_lba	= %u \n",  hdr->entries_lba);
	DBG("entries_count	= %u \n",  hdr->entries_count);
	DBG("entries_size	= %u \n",  hdr->entries_size);
	DBG("entries_crc32	= %u \n",  hdr->entries_crc32);
}

static void end_ptbl(struct ptable *ptbl)
{
	struct efi_header *hdr = &ptbl->header;
	u32 n;

	DBG("end_ptbl\n");

	n = crc32(0, NULL, 0);
	n = crc32(n, (void *) ptbl->entry, sizeof(ptbl->entry));
	hdr->entries_crc32 = n;

	n = crc32(0, NULL, 0);
	n = crc32(0, (void *) &ptbl->header, sizeof(ptbl->header));
	hdr->crc32 = n;
}

static int add_ptn(struct ptable *ptbl, u64 first, u64 last, const char *name)
{
	struct efi_header *hdr = &ptbl->header;
	struct efi_entry *entry = ptbl->entry;
	u32 n; int i = 0;

	DBG("add_ptn\n");

	if (first < 34) {
		printf("partition '%s' overlaps partition table\n", name);
		return -1;
	}

	if (last > hdr->last_lba) {
		printf("partition '%s' does not fit\n", name);
		return -1;
	}

	for (n = 0; n < EFI_ENTRIES; n++, entry++) {
		if (entry->last_lba)
			continue;
		memcpy(entry->type_uuid, partition_type, 16);
		memcpy(entry->uniq_uuid, random_uuid, 16);
		entry->uniq_uuid[0] = n;
		entry->first_lba = first;
		entry->last_lba = last;

		/* Converting partition name to simple unicode
		as expected by the kernel */
		while (i <= EFI_NAMELEN && *name) {
			entry->name[i] = name[i];
			if (name[i] == 0)
				break;
			entry->name[i+1] = '0';
			i++;
		}

		return 0;
	}

	printf("out of partition table entries\n");
	return -1;
}

int do_gpt_format(struct fastboot_data *fb_data)
{
	/* For testing need to pass this in better */
	struct ptable *ptbl;
	u64 total_sectors = 0;
	u64 next;
	int n;
	u32 sector_sz = fb_data->storage_ops->get_sector_size();
	u64 ptbl_sectors = 0;
	int ret = 0;

#ifdef DEBUG
	int i = 0; int j = 0;
	u8 *data;
	u32 *blocksp = &total_sectors;
	data = (u8 *)alloc_memory(sizeof(struct ptable));
#endif

	DBG("do_format\n");

	ptbl_sectors = sizeof(struct ptable) / sector_sz;

	total_sectors = fb_data->storage_ops->get_total_sectors();
	DBG("sector_sz %u\n", sector_sz);
	DBG("total_sectors 0x%x%08x\n", blocksp[1], blocksp[0]);

	ptbl = (struct ptable *) alloc_memory(sizeof(struct ptable));
	if (!ptbl) {
		printf("%s: Unable to alloc mem for ptbl\n", __func__);
		return -1;
	}
	start_ptbl(ptbl, total_sectors);
	if (fb_data->board_ops->board_get_part_tbl)
		partitions = fb_data->board_ops->board_get_part_tbl();

	n = 0;
	next = 0;
	for (n = 0, next = 0; partitions[n].name; n++) {
		u64 sz_sectors = 0;
		sz_sectors = (u64)partitions[n].size_kb << 1;
		if (!strcmp(partitions[n].name, "-")) {
			next += sz_sectors;
			continue;
		}
		if (sz_sectors == 0)
			sz_sectors = total_sectors - next;

		if (add_ptn(ptbl, next, next + sz_sectors - 1,
				partitions[n].name)) {
			printf("Unable to add_ptn\n");
			ret = -1;
			goto format_err;
		}

		next += sz_sectors;
	}

	end_ptbl(ptbl);

	DBG("writing ptable to disk: %d #of sectors\n", ptbl_sectors);
	ret = fb_data->storage_ops->write(0, ptbl_sectors, (void *)ptbl);
	if (ret) {
		printf("Write PTBL failed\n");
		goto format_err;
	}

	DBG("writing the GUID Table disk ...\n");
#ifdef DEBUG
	ret = fb_data->storage_ops->read(0, ptbl_sectors, (void *)data);
	if (ret != 0) {
		printf("error reading MBR\n");
		return ret;
	} else {
			printf("printing ptable\n");
			for (i = 0; i < sizeof(struct ptable); i++)
				printf("%02X ", data[i]);
			printf("\n");
		}
#endif
	DBG("\nnew partition table:\n");
	ret = load_ptbl(fb_data->storage_ops, 0);
	if (ret != 0) {
		printf("Failed to load partition table\n");
		goto format_err;
	}
format_err:
	free_memory(ptbl);
#ifdef DEBUG
	free_memory(data);
#endif
	return ret;
}

static u64 get_entry_size_kb(struct efi_entry *entry, const char *ptn)
{
	int ret = 0;
	char name[16];
	u64 sz = 0;

	ret = memcmp(entry->type_uuid, partition_type, sizeof(partition_type));
	if (ret != 0)
		return 0;

	strcpy(name, (convert_ptn_name_to_unicode(entry)));

	if (!strcmp(name, ptn))
		sz = (entry->last_lba - entry->first_lba)/2;

	return sz;
}

char *get_ptn_size(struct fastboot_data *fb_data, char *buf, const char *ptn)
{
	int i = 0;
	int ret = 0;
	u32 sz_mb;
	u64 sz = 0;
	struct ptable *gpt;
	int sector_sz = fb_data->storage_ops->get_sector_size();
	u64 ptbl_sectors = 0;

	gpt = (struct ptable *)alloc_memory(sizeof(struct ptable));
	if (!gpt) {
		printf("Unable to alloc memory for gpt\n");
		return buf;
	}

	if (sector_sz != 512) {
		printf("Unknown sector size: %d\n", sector_sz);
		goto ptn_error;
	} else
		ptbl_sectors = sizeof(struct ptable) >> 9;

	ret = fb_data->storage_ops->read(0, ptbl_sectors, (void *)gpt);
	if (ret != 0) {
		printf("error reading primary GPT\n");
		goto ptn_error;
	}

	if (memcmp(gpt->header.magic, "EFI PART", 8)) {
		printf("efi partition table not found\n");
		goto ptn_error;
	}

	for (i = 0; i < EFI_ENTRIES; i++) {
		sz = get_entry_size_kb(&gpt->entry[i], ptn);
		if (sz)
			break;
	}

	if (sz >= 0xFFFFFFFF) {
		sz_mb = (u32)(sz >> 20);
		DBG("sz is > 0xFFFFFFFF\n");
		sprintf(buf, "0x%d MB", sz_mb);
	} else {
		DBG("Size of the partition = %d KB\n", (u32)sz);
		sprintf(buf, "%d KB", (u32)sz);
	}

ptn_error:
	free_memory(gpt);
	return buf;
}

#endif
