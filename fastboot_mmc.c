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
#include <libc/string.h>
#include <aboot/common.h>
#include <common/omap_rom.h>
#include "config.h"

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

#if defined CONFIG_FASTBOOT
#include <common/fastboot.h>

#define EFI_VERSION 0x00010000
#define EFI_ENTRIES 128
#define EFI_NAMELEN 36

struct mmc mmc;
struct mmc_devicedata *dd;
u64 sz;
int count;

static const u8 partition_type[16] = {
	0xa2, 0xa0, 0xd0, 0xeb, 0xe5, 0xb9, 0x33, 0x44,
	0x87, 0xc0, 0x68, 0xb6, 0xb7, 0x26, 0x99, 0xc7,
};

static const u8 random_uuid[16] = {
	0xff, 0x1f, 0xf2, 0xf9, 0xd4, 0xa8, 0x0e, 0x5f,
	0x97, 0x46, 0x59, 0x48, 0x69, 0xae, 0xc3, 0x4e,
};

struct efi_entry {
	u8 type_uuid[16];
	u8 uniq_uuid[16];
	u64 first_lba;
	u64 last_lba;
	u64 attr;
	u16 name[EFI_NAMELEN];
};

struct efi_header {
	u8 magic[8];

	u32 version;
	u32 header_sz;

	u32 crc32;
	u32 reserved;

	u64 header_lba;
	u64 backup_lba;
	u64 first_lba;
	u64 last_lba;

	u8 volume_uuid[16];

	u64 entries_lba;

	u32 entries_count;
	u32 entries_size;
	u32 entries_crc32;
} __attribute__((packed));

struct ptable {
	u8 mbr[512];
	union {
		struct efi_header header;
		u8 block[512];
	};
	struct efi_entry entry[EFI_ENTRIES];
};

struct partition {
	const char *name;
	u32 size_kb;
};

#if defined CONFIG_BLAZE || defined CONFIG_BLAZE_TABLET
static struct partition partitions[] = {
	{ "-", 128 },
	{ "xloader", 128 },
	{ "bootloader", 256 },
	/* "misc" partition is required for recovery */
	{ "misc", 128 },
	{ "-", 384 },
	{ "efs", 16384 },
	{ "crypto", 16 },
	{ "recovery", 8*1024 },
	{ "boot", 8*1024 },
	{ "system", 512*1024 },
	{ "cache", 256*1024 },
	{ "userdata", 0},
	{ 0, 0 },
};
#elif defined CONFIG_PANDA
static struct partition partitions[] = {
	{ "-", 128 },
	{ "xloader", 128 },
	{ "bootloader", 256 },
	{ "-", 512 },
	{ "recovery", 8*1024 },
	{ "boot", 8*1024 },
	{ "system", 512*1024 },
	{ "cache", 256*1024 },
	{ "userdata", 0},
	{ 0, 0 },
};
#elif defined CONFIG_OMAP5EVM || defined CONFIG_OMAP5UEVM
/*
  Increasing the size of the xloader partition
  so that the bootloader is now located at 0x300,
  which is where SPL expects U-BOOT to be.
*/
static struct partition partitions[] = {
	{ "-", 128 },
	{ "xloader", 256 },
	{ "bootloader", 256 },
	/* "misc" partition is required for recovery */
	{ "misc", 128 },
	{ "-", 384 },
	{ "efs", 16384 },
	{ "crypto", 16 },
	{ "recovery", 8*1024 },
	{ "boot", 8*1024 },
	{ "system", 512*1024 },
	{ "cache", 256*1024 },
	{ "userdata", 0},
	{ 0, 0 },
};
#endif

static struct ptable the_ptable;

static void init_mbr(u8 *mbr, u32 blocks)
{
	printf("init_mbr\n");

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

	memcpy(mbr + 0x1ca, &blocks, sizeof(u32));

	mbr[0x1fe] = 0x55;
	mbr[0x1ff] = 0xaa;
}

static void start_ptbl(struct ptable *ptbl, unsigned blocks)
{
	struct efi_header *hdr = &ptbl->header;

	printf("start_ptbl\n");

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

	printf("end_ptbl\n");

	n = crc32(0, 0, 0);
	n = crc32(n, (void *) ptbl->entry, sizeof(ptbl->entry));
	hdr->entries_crc32 = n;

	n = crc32(0, 0, 0);
	n = crc32(0, (void *) &ptbl->header, sizeof(ptbl->header));
	hdr->crc32 = n;
}

int add_ptn(struct ptable *ptbl, u64 first, u64 last, const char *name)
{
	struct efi_header *hdr = &ptbl->header;
	struct efi_entry *entry = ptbl->entry;
	u32 n; int i = 0;

	printf("add_ptn\n");

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

static char *convert_ptn_name_to_unicode(struct efi_entry *entry)
{
	int i = 0;
	static char name[16];

	/* copying a simple unicode partition name */
	while (i < (sizeof(entry->name)-1)) {
		name[i] = entry->name[i];
		i++;
		if (entry->name[i] == 0)
			break;
	}

	name[i] = 0;

	return name;
}

void import_efi_partition(struct efi_entry *entry)
{
	struct fastboot_ptentry e;
	int ret = 0;

	ret = memcmp(entry->type_uuid, partition_type, sizeof(partition_type));
	if (ret != 0) {
		DBG("memcmp failed, ret = %d. entry->type_uuid and "
			"partition_type are mismatched.\n", ret);
		return;
	}

	strcpy(e.name, (convert_ptn_name_to_unicode(entry)));

	e.start = entry->first_lba;
	e.length = (entry->last_lba - entry->first_lba + 1) * 512;
	e.flags = 0;

	if (!strcmp(e.name, "environment"))
		e.flags |= FASTBOOT_PTENTRY_FLAGS_WRITE_ENV;
	fastboot_flash_add_ptn(&e, count);

	if (e.length > 0x100000)
		printf("%8d %7dM %s\n", e.start,
			(u32)(e.length/0x100000), e.name);
	else
		printf("%8d %7dK %s\n", e.start,
			(u32)(e.length/0x400), e.name);
}

static int load_ptbl(void)
{
	static u8 data[512];
	static struct efi_entry entry[4];
	int m = 0; int r = 0; int j = 0;

	count = 0;

	printf("load_ptbl\n");

	r = mmc_read(&mmc, 0, 1, data);
	if (r != 0) {
		printf("error reading MBR\n");
		return r;
	}

	r = mmc_read(&mmc, 1, 1, data);
	if (r != 0) {
		printf("error reading primary GPT header\n");
		return r;
	}

	if (memcmp(data, "EFI PART", 8)) {
		printf("efi partition table not found\n");
		return -1;
	} else
		printf("efi partition table found\n");

	for (j = 2; j < 34; j++) {
		r = mmc_read(&mmc, j, 1, entry);
		if (r != 0) {
			printf("error reading partition entries\n");
			return r;
		}

		for (m = 0; m < 4; m++) {
			import_efi_partition(entry + m);
			count++;
		}
	}

	return 0;
}

static int do_format(void)
{
	struct ptable *ptbl = &the_ptable;
	u32 blocks = 0;
	u32 next;
	int n;
	int size = 0;
	int num_sectors = 0;
	int ret = 0;

	#ifdef DEBUG
	int i = 0; int j = 0;
	u32 sector_sz = 0;
	static u8 data[512];
	#endif

	printf("do_format\n");

	ret = mmc_open(device, &mmc);
	if (ret != 0) {
		printf("mmc init failed, retrying ...\n");
		ret = mmc_open(device, &mmc);
		if (ret != 0) {
			printf("mmc init failed on retry, exiting!\n");
			return ret;
		}
	}

	mmc_info(&mmc);
	dd = mmc.dread.device_data;

	if (dd->mode != 1)
		dd->mode = 1; /*MMCSD_MODE_RAW*/

	blocks = dd->size;
	DBG("sector_sz %u\n", sector_sz);
	DBG("blocks %u\n", blocks);

	start_ptbl(ptbl, blocks);

	n = 0;
	next = 0;
	for (n = 0, next = 0; partitions[n].name; n++) {
		u32 sz = partitions[n].size_kb * 2;
		if (!strcmp(partitions[n].name, "-")) {
			next += sz;
			continue;
		}
		if (sz == 0)
			sz = blocks - next;

		if (add_ptn(ptbl, next, next + sz - 1, partitions[n].name))
			return -1;

		next += sz;
	}

	end_ptbl(ptbl);

	count = 0;

	size = sizeof(struct ptable);
	num_sectors = (size/512);

	printf("writing ptable to disk: %d #of bytes to %d # of sectors\n",
							size, num_sectors);

	printf("writing the MBR to disk ... \n");
	ret = mmc_write(&mmc, 0, 1, ptbl->mbr);
	if (ret != 0) {
		printf("mmc write failed\n");
		return ret;
	}

	#ifdef DEBUG
	ret = mmc_read(&mmc, 0, 1, data);
	if (ret != 0) {
		printf("error reading MBR\n");
		return ret;
	} else {
			printf("printing sector 0 ==> MBR\n");
			for (i = 0; i < 512; i++)
				printf("%02X ", data[i]);
			printf("\n");
		}
	#endif

	printf("writing the GPT table to disk ... \n");
	ret = mmc_write(&mmc, 1, 1, &ptbl->header);
	if (ret != 0) {
		printf("mmc write failed\n");
		return ret;
	}

	#ifdef DEBUG
	ret = mmc_read(&mmc, 1, 1, data);
	if (ret != 0) {
		printf("error reading GPT table\n");
		return ret;
	} else {
			printf("printing sector 1 ==> GPT header\n");
			for (i = 0; i < 512; i++)
				printf("%02X ", data[i]);
			printf("\n");
		}
	#endif

	ret = mmc_write(&mmc, 2, sizeof(ptbl->entry)/512, &ptbl->entry);
	if (ret != 0) {
		printf("mmc write failed\n");
		return ret;
	}

	#ifdef DEBUG
	for (j = 2; j < 34; j++) {
		ret = mmc_read(&mmc, j, 1, data);
		if (ret != 0) {
			printf("error reading partition table\n");
			return ret;
		} else {
			printf("printing sector %d \n", j);
			for (i = 0; i < 512; i++)
				printf("%02X ", data[i]);
			printf("\n");
		}
	}
	#endif

	printf("\nnew partition table:\n");
	ret = load_ptbl();
	if (ret != 0) {
		printf("Failed to load partition table\n");
		return ret;
	}

	return 0;
}

int fastboot_oem(void)
{
	int ret = 0;

	printf("fastboot_oem\n");

	ret = do_format();
	if (ret != 0)
		printf("do_format() failed\n");

	return ret;
}

int board_mmc_init(void)
{
	int ret = 0;

	printf("board_mmc_init\n");

	ret = mmc_open(device, &mmc);
	if (ret != 0) {
		printf("mmc init failed, retrying ...\n");
		ret = mmc_open(device, &mmc);
		if (ret != 0) {
			printf("mmc init failed on retry, exiting!\n");
			return ret;
		}
	}

	printf("\nefi partition table:\n");
	ret =  load_ptbl();
	if (ret != 0) {
		printf("Failed to load the partition table\n");
		return ret;
	}

	return 0;
}

void get_entry_size(struct efi_entry *entry, const char *ptn)
{
	int ret = 0;
	char name[16];

	ret = memcmp(entry->type_uuid, partition_type, sizeof(partition_type));
	if (ret != 0)
		return;

	strcpy(name, (convert_ptn_name_to_unicode(entry)));

	if (!strcmp(name, ptn))
		sz = (entry->last_lba - entry->first_lba)/2;

	return;
}

char *get_ptn_size(char *buf, const char *ptn)
{
	static u8 data[512];
	static struct efi_entry entry[4];
	int m = 0; int r = 0; int j = 0;
	u32 *sz_ptr; sz = 0;

	r = mmc_open(device, &mmc);
	if (r != 0) {
		printf("mmc init failed, retrying ...\n");
		r = mmc_open(device, &mmc);
		if (r != 0) {
			printf("mmc init failed on retry, exiting!\n");
			return buf;
		}
	}

	r = mmc_read(&mmc, 1, 1, data);
	if (r != 0) {
		printf("error reading primary GPT header\n");
		return buf;
	}

	if (memcmp(data, "EFI PART", 8)) {
		printf("efi partition table not found\n");
		return buf;
	} else
		printf("efi partition table found\n");

	for (j = 2; j < 34; j++) {
		r = mmc_read(&mmc, j, 1, entry);
		if (r != 0) {
			printf("error reading partition entries\n");
			return buf;
		}

		for (m = 0; m < 4; m++) {
			if (sz)
				break;
			get_entry_size(entry + m, ptn);
		}
	}

	if (sz >= 0xFFFFFFFF) {
		sz_ptr = &sz;
		DBG("sz is > 0xFFFFFFFF\n");
		sprintf(buf, "0x%08x , %08x KB", sz_ptr[1], sz_ptr[0]);
	} else {
		DBG("Size of the partition = %d KB\n", (u32)sz);
		sprintf(buf, "%d KB", (u32)sz);
	}

	return buf;

}

#endif
