/*
* Copyright (C) 2013 Texas Instruments, Inc.
* All rights reserved.
*
* Author: Ruslan Bilovol <ruslan.bilovol@ti.com>
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

#include <string.h>
#include <alloc.h>
#include <device_tree_utils.h>

#define ERR(x...) printf("ERROR: " x)

#define DEBUG 1

#ifdef DEBUG
#define DBG(x...) printf("debug: " x)
#else
#define DBG(x...)
#endif /* DEBUG */

#define DT_BLOB_HEADER_GETVAR(blob,var) __be32_to_cpu(((struct fdt_header *)blob)->var)

static u32 dt_check_header(void *dt_blob)
{
	if (DT_BLOB_HEADER_GETVAR(dt_blob, magic) != FDT_MAGIC)
		return -1;

	return 0;
}

/*
 * Returns offset inside of structure block
 */
static u32 dt_find_node(void *dt_blob, u32 dt_parent_node_off, char *node)
{
	void *dt_struct_start = dt_blob + DT_BLOB_HEADER_GETVAR(dt_blob, off_dt_struct);
	u32 dt_struct_size = DT_BLOB_HEADER_GETVAR(dt_blob, size_dt_struct);
	int i;

	if (!node) {
		ERR("%s: node=NULL\n", __func__);
		return -1;
	}


	for (i = 0; i < dt_struct_size - dt_parent_node_off; i+=4) {
		void *dt_struct_curr = dt_struct_start + dt_parent_node_off + i;

		if ( __be32_to_cpu(*((int *)(dt_struct_curr))) == FDT_BEGIN_NODE) {
			if (!(strcmp((dt_struct_curr + 4), node))) {
				DBG("%s: found node '%s' offset = %d\n", __func__, node, (dt_struct_curr - dt_struct_start));
				return (dt_struct_curr - dt_struct_start);
			}
		}
	}

	ERR("%s: cannot find node '%s'\n", __func__, node);
	return -1;
}

static u32 dt_find_property(void *dt_blob, u32 dt_node_off, char *property)
{

	void *dt_struct_start = dt_blob + DT_BLOB_HEADER_GETVAR(dt_blob, off_dt_struct);
	u32 dt_struct_size = DT_BLOB_HEADER_GETVAR(dt_blob, size_dt_struct);
	void *dt_strings_start = dt_blob + DT_BLOB_HEADER_GETVAR(dt_blob, off_dt_strings);

	int i;

	DBG("%s: looking for property '%s'\n", __func__, property);

	for (i = dt_node_off; i < dt_struct_size; i+=4) {
		void *dt_struct_curr = dt_struct_start + i;

		if ( __be32_to_cpu(*((int *)(dt_struct_curr))) == FDT_PROP) {
			struct fdt_property *fdt_prop = (struct fdt_property *)(dt_struct_curr + 4);

			if (!strcmp((__be32_to_cpu(fdt_prop->nameoff) + dt_strings_start), property)) {
				DBG("%s: found property '%s' offset = %d\n", __func__, property, (dt_struct_curr - dt_struct_start));
				return (dt_struct_curr - dt_struct_start);
			}
		}
	}

	ERR("%s: cannot find property '%s'\n", __func__, property);
	return -1;
}

static u32 dt_set_property(void *dt_blob, u32 dt_prop_off, u32 value)
{

	void *dt_struct_start = dt_blob + DT_BLOB_HEADER_GETVAR(dt_blob, off_dt_struct);
	u32 dt_struct_size = DT_BLOB_HEADER_GETVAR(dt_blob, size_dt_struct);
	void *dt_property_start;
	u32 *dt_property_value;

	if (dt_prop_off > dt_struct_size) {
		ERR("%s: out of bound: size=%d, requested offset=%d\n", __func__, dt_struct_size, dt_prop_off);
		return -1;
	}

	dt_property_start = dt_struct_start + dt_prop_off;
	dt_property_value = (u32 *)(dt_property_start + sizeof(struct fdt_property) + 4);

	*dt_property_value = __cpu_to_be32(value);

	return 0;
}

u32 dt_find_and_replace(void *dt_blob, char *property, u32 value)
{
	u32 dt_node_off = 0;
	u32 dt_prop_off = 0;
	char dt_prop_path[128];
	char dt_prop_name[32];
	u32 i, retval;

	retval = dt_check_header(dt_blob);
	if (retval) {
		ERR("%s: wrong DT blob structure\n", __func__);
		return retval;
	}

	DBG("%s: setup property %s = <0x%x>\n", __func__, property, value);

	for (i = strlen(property); i > 0; i--) {
		if (!strncmp((property + i), "/", 1)) {
			strcpy(dt_prop_name, property + i + 1);
			DBG("%s: parsed property name = '%s'\n", __func__, dt_prop_name);

			strncpy(dt_prop_path, property, i + 1);
			dt_prop_path[i + 1] = '\0';
			DBG("%s: parsed property path = '%s'\n", __func__, dt_prop_path);
			break;
		}
	}

	{
		char *start_pos = dt_prop_path + 1; /* Drop first '/' */

		while (start_pos < dt_prop_path + strlen(dt_prop_path) - 1) {
			char node[32];
			for (i = 0; i < strlen(start_pos); i++) {
				if (!strncmp((start_pos + i), "/", 1)) {
					strncpy(node, start_pos, i);
					node[i] = '\0';
					dt_node_off = dt_find_node(dt_blob, dt_node_off, node);
					if (dt_node_off == -1) {
						ERR("%s: no such node '%s'\n", __func__, node);
						return -1;
					}

					start_pos = start_pos + i + 1;
					break;
				}
			}
		};
	}

	dt_prop_off = dt_find_property(dt_blob, dt_node_off, dt_prop_name);
	dt_set_property(dt_blob, dt_prop_off, value);

	return 0;
}

