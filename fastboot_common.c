/*
* Copyright (C) 2012, Texas Instruments, Inc.
* Texas Instruments, <www.ti.com>
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

#include <aboot.h>
#include <common.h>
#include <io.h>
#include <types.h>

#include <string.h>

#include <common_proc.h>
#include <omap_rom.h>
#include <alloc.h>

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

#if defined CONFIG_FASTBOOT
#include <fastboot.h>
#include <fastboot_common.h>

struct fastboot_ptentry fb_ptable[MAX_PTN];

void fastboot_flash_add_ptn(fastboot_ptentry *ptn, int count)
{
	if (count < MAX_PTN)
		memcpy(fb_ptable + count, ptn, sizeof(*ptn));
}

fastboot_ptentry *fastboot_flash_find_ptn(const char *name)
{
	unsigned int n;

	for (n = 0; n < MAX_PTN; n++) {
		/* Make sure a substring is not accepted */
		if (strlen(name) == strlen(fb_ptable[n].name)) {
			if (0 == strcmp(fb_ptable[n].name, name))
				return fb_ptable + n;
		}
	}

	return NULL;
}

char *convert_ptn_name_to_unicode(struct efi_entry *entry)
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

static void import_efi_partition(struct efi_entry *entry, int count, u8 silent)
{
	struct fastboot_ptentry e;
	int ret = 0;

	ret = memcmp(entry->type_uuid, partition_type, sizeof(partition_type));
	if (ret != 0) {
		DBG("memcmp failed for count=%d, ret = %d. entry->type_uuid "
			"and partition_type are mismatched.\n", count, ret);
		return;
	}

	strcpy(e.name, (convert_ptn_name_to_unicode(entry)));

	e.start = entry->first_lba;
	e.length = (entry->last_lba - entry->first_lba + 1) * 512;
	e.flags = 0;

	if (!strcmp(e.name, "environment"))
		e.flags |= FASTBOOT_PTENTRY_FLAGS_WRITE_ENV;
	fastboot_flash_add_ptn(&e, count);

	if (!silent) {
		if (e.length > 0x100000)
			DBG("%8d %7dM %s\n", e.start,
				(u32)(e.length/0x100000), e.name);
		else
			DBG("%8d %7dK %s\n", e.start,
				(u32)(e.length/0x400), e.name);
	}
}

int load_ptbl(struct storage_specific_functions *storage, u8 silent)
{
	u32 sector_sz = storage->get_sector_size();
	u64 ptbl_sectors = 0;
	int i = 0, r = 0;

	struct ptable *gpt;
	int gpt_size = sizeof(struct ptable);

	gpt =  (struct ptable *) alloc_memory(gpt_size);
	if (!gpt) {
		r = 0;
		goto fail;
	}

	ptbl_sectors = (u64)(gpt_size / sector_sz);

	r = storage->read(0, ptbl_sectors, (void *)gpt);
	if (r != 0) {
		printf("error reading GPT\n");
		return r;
		goto fail;
	}

	if (memcmp(gpt->header.magic, "EFI PART", 8)) {
		if (!silent)
			printf("efi partition table not found\n");
		r = -1;
		goto fail;
	}

	for (i = 0; i < EFI_ENTRIES; i++)
		import_efi_partition(&gpt->entry[i], i, silent);

fail:
	free_memory((void *)gpt);
	return r;
}

#endif
