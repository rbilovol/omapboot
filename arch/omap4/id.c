/*
 * Copyright (C) 2011 The Android Open Source Project
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

/* omap44xx ID detection and reporting */

#include <string.h>

#include <aboot.h>
#include <io.h>

#include <usbboot_common.h>
#include <omap_rom.h>

#include <hw.h>

#define DEVICETYPE_MASK	(0x7 << 8)

struct omap_id {
	omap_rev    rev_num;
	u32         rev_reg_val;
};

static struct omap_id  map[] = {
        { OMAP_4430_ES1_DOT_0, 0x0B85202F },
        { OMAP_4430_ES2_DOT_0, 0x1B85202F },
        { OMAP_4430_ES2_DOT_1, 0x3B95C02F },
        { OMAP_4430_ES2_DOT_2, 0x4B95C02F },
        { OMAP_4430_ES2_DOT_3, 0x6B95C02F },
        { OMAP_4460_ES1_DOT_0, 0x0B94E02F },
	{ OMAP_4460_ES1_DOT_1, 0x2B94E02F },
	{ OMAP_4470_ES1_DOT_0, 0x0B97502F },
        { OMAP_REV_INVALID,    0x00000000 },
};

static int get_omap_rev(void)
{
	u8 i;
	u32 id_code;

	id_code = readl(CONTROL_ID_CODE);

	for (i = 0; map[i].rev_num != OMAP_REV_INVALID; i++) {
		if (map[i].rev_reg_val == id_code) {
			return map[i].rev_num;
		}
	}

	return OMAP_REV_INVALID;
}

static omap_type get_omap_type(void)
{
	unsigned int value;

	value = readl(CONTROL_STATUS);
	value &= DEVICETYPE_MASK;
	return value >> 8;
}

static char *get_cpu_revision(void)
{
	static char cpu_rev[8];
	int cpu = get_omap_rev();

	switch (cpu) {
	case OMAP_4430_ES1_DOT_0:
	case OMAP_4460_ES1_DOT_0:
	case OMAP_4470_ES1_DOT_0:
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
	case OMAP_4460_ES1_DOT_1:
		strcpy(cpu_rev, "ES1.1");
		break;
	default:
		printf("OMAP_REV_INVALID\n");
		strcpy(cpu_rev, "invalid");
		break;
	}

	return cpu_rev;
}

char *get_cpu_version(void)
{
	static char proc_ver[8];
	int cpu = get_omap_rev();

	switch (cpu) {
	case OMAP_4430_ES1_DOT_0:
	case OMAP_4430_ES2_DOT_0:
	case OMAP_4430_ES2_DOT_1:
	case OMAP_4430_ES2_DOT_2:
	case OMAP_4430_ES2_DOT_3:
		strcpy(proc_ver, "4430");
		break;
	case OMAP_4460_ES1_DOT_0:
	case OMAP_4460_ES1_DOT_1:
		strcpy(proc_ver, "4460");
		break;
	case OMAP_4470_ES1_DOT_0:
		strcpy(proc_ver, "4470");
		break;
	default:
		printf("OMAP_REV_INVALID\n");
		strcpy(proc_ver, "invalid");
		break;
	}

	return proc_ver;
}

static char *get_cpu_type(void)
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

static char *get_serial_no(void)
{
	static char serialno[20];
	u32 val[4] = { 0 };
	u32 reg;

	reg = CONTROL_STD_FUSE_DIE_ID_0;
	val[0] = readl(reg);
	val[1] = readl(reg + 0x8);
	val[2] = readl(reg + 0xC);
	val[3] = readl(reg + 0x10);
	printf("Device Serial Number: %08X%08X\n", val[3], val[2]);
	sprintf(serialno, "%08X%08X", val[3], val[2]);

	return serialno;
}

static u32 get_public_rom_base(void)
{
	u32 public_rom_base = PUBLIC_API_BASE_4430;

	if (get_omap_rev() >= OMAP_4460_ES1_DOT_1)
		public_rom_base = PUBLIC_API_BASE_4460;

	return public_rom_base;

}

static char *get_rom_version(void)
{
	static char rom_version[5];

	if (get_omap_rev() < OMAP_4460_ES1_DOT_0)
		sprintf(rom_version, "%x", readl(OMAP4430_ROM_CODE_VERSION));
	else {
		/*
		* In the ROM Memory MAP, the offset for rom version is the
		* same for both 4460 and 4470
		*/
		sprintf(rom_version, "%x", readl(OMAP4460_ROM_CODE_VERSION));
	}

	return rom_version;
}

static struct proc_specific_functions omap4_id_funcs = {
	.proc_get_serial_num = get_serial_no,
	.proc_get_type = get_cpu_type,
	.proc_get_revision = get_cpu_revision,
	.proc_get_version= get_cpu_version,
	.proc_get_proc_id = get_omap_rev,
	.proc_get_api_base = get_public_rom_base,
	.proc_get_rom_version = get_rom_version,
};

void* init_processor_id_funcs(void)
{
	return &omap4_id_funcs;
}
