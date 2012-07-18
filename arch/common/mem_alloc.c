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

#include <aboot/aboot.h>
#include <common/usbboot_common.h>
#include <common/alloc.h>

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

__attribute__((__section__(".sdram")))
static struct mem_alloc_header head;

static u32 memory_available;
static void *data = &head;
static void *next = &head;

void init_memory_alloc(void)
{
	/* Initially the entire heap is FREE */
	memory_available = HEAP_SIZE;
	head.section_size = 0;
	head.status = 0;

	head.next = NULL;
	head.data = NULL;

	return;
}

static int align_size_requested(int size)
{
	if (size % 4)
		size = ((size >> 2) + 1) << 2;

	return size;
}

static void create_new_hdr(void)
{
	struct mem_alloc_header *new;

	new = next;
	new->status = 0;
	new->section_size = 0;
	new->data = NULL;
	new->next = NULL;

}

int *alloc_memory(int size)
{
	void *ret_ptr = NULL;
	struct mem_alloc_header *hdr = &head;

	if (size < 1) {
		DBG("user has requested an invalid amount of memory\n");
		return NULL;
	}

	size = align_size_requested(size);

	DBG("user has requested %d bytes and %d byte(s) are available\n",
		size, memory_available);

	if (memory_available < size) {
		printf("ERROR!! not enough memory available for allocation\n");

		return NULL;
	}

#ifdef DEBUG
	if (memory_available == HEAP_SIZE)
		DBG("all heap mem is available\n");
	else if (memory_available < HEAP_SIZE)
		DBG("some heap mem is available, some heap mem is in use\n");
#endif

	/*
	* traverse the heap to find an available section:
	* start at the top of the heap
	* check each header status to find one unused
	* if we find an unused section, check if it can fit
	* "size" bytes, else allocate a new section at the
	* bottom of the heap
	*/

	while (ret_ptr == NULL) {

		if (!hdr->status) {
			DBG("current section is not in use\n");

			if ((!hdr->section_size) ||
				(size <= hdr->section_size)) {
				DBG("current section has not been allocated, "
				"we can fit %d bytes in this section\n", size);

				memory_available -= size;
				hdr->status = 1;
				hdr->section_size = size;

				data = hdr;
				next = hdr;

				next += sizeof(head) + (size * sizeof(u32));
				hdr->next = next;

				data += sizeof(head);
				hdr->data = data;

				create_new_hdr();

				ret_ptr = data;

			} else if (size > hdr->section_size) {
				DBG("we cannot fit %d bytes in a section of "
				"size %d bytes\n", size, hdr->section_size);

				DBG("...skip to next section\n");
				hdr = (struct mem_alloc_header *) hdr->next;

				if (!hdr->next) {
					DBG("since hdr->next is NULL, "
						"create new header\n");

					next = hdr;
					next += sizeof(head) +
						(size * sizeof(u32));

					create_new_hdr();

					data = hdr;
					next = hdr;

				}
			}
		} else {
			DBG("current section in use, skip to next section\n");
			hdr = (struct mem_alloc_header *) hdr->next;

			if (!hdr->next) {
				DBG("since hdr->next is NULL, "
					"create new header\n");
				next = hdr;
				next += sizeof(head) + (size * sizeof(u32));

				create_new_hdr();

				data = hdr;
				next = hdr;
			}
		}
	}

	data = &head;
	next = &head;

	return ret_ptr;
}

int *zalloc_memory(int size)
{
	/* call alloc_memory() and then memset the
	*  allocated memory to zero
	*  This is an expensive operation to do, so use this judiciously!!!
	*/

	void *ptr_mem;

	ptr_mem = alloc_memory(size);
	if (ptr_mem != NULL) {
		memset(ptr_mem, 0, size);
		return ptr_mem;
	} else {
		printf("invalid memory section\n");
		return NULL;
	}
}

int free_memory(void *ptr_to_data)
{
	if (ptr_to_data == NULL) {
		DBG("cannot free invalid pointer\n");
		return -1;
	}

	/* ToDo : Check if the pointer passed in is within the heap */

	void *free = (void *) (ptr_to_data - 0x10);
	struct mem_alloc_header *hdr = free;

	/* mark the section as unused */
	hdr->status = 0;

	/* add the section size back to available memory pool */
	memory_available += hdr->section_size;

	DBG("user has freed memory section 0x%x of size 0x%x\n",
					free, hdr->section_size);

	return 0;
}

