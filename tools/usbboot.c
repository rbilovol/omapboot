/*
 * Copyright (C) 2010 The Android Open Source Project
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>

#include "usb.h"

typedef struct tocentry {
	unsigned offset;
	unsigned length;
	unsigned flags;
	unsigned align;
	unsigned spare;
	char name[12];
} tocentry;

#define USE_TOC 0

static char *usb_boot_read_chip_info(usb_handle *usb)
{
	static char proc_type[8];
	uint32_t msg_getid = 0xF0030003;
	uint8_t id[81];
	uint8_t *crc1;
	uint8_t gp_device_crc1[4] = {0, 0, 0, 0};
	int i;

#define OFF_CHIP	0x04
#define OFF_ID		0x0F
#define OFF_MPKH	0x26
	memset(id, 0xee, 81);
	fprintf(stderr,"reading ASIC ID\n");
	usb_write(usb, &msg_getid, sizeof(msg_getid));
	usb_read(usb, id, sizeof(id));

	fprintf(stderr,"CHIP: %02x%02x\n", id[OFF_CHIP+0], id[OFF_CHIP+1]);
	fprintf(stderr,"IDEN: ");
	for (i = 0; i < 20; i++)
		fprintf(stderr,"%02x", id[OFF_ID+i]);
	fprintf(stderr,"\nMPKH: ");
	for (i = 0; i < 32; i++)
		fprintf(stderr,"%02x", id[OFF_MPKH+i]);
	fprintf(stderr,"\nCRC0: %02x%02x%02x%02x\n",
		id[73], id[74], id[75], id[76]);
	fprintf(stderr,"CRC1: %02x%02x%02x%02x\n",
		id[77], id[78], id[79], id[80]);

	crc1 = &id[77];
	if (memcmp(crc1, &gp_device_crc1, 4 * sizeof(uint8_t))) {
		fprintf(stderr, "device is ED/HD (EMU/HS)\n");
		strcpy(proc_type, "EMU");
	} else {
		fprintf(stderr, "device is GP\n");
		strcpy(proc_type, "GP");
	}

	return proc_type;
}

int usb_boot(usb_handle *usb, int fastboot_mode,
	     void *data, unsigned sz,
	     void *data2, unsigned sz2)
{
	uint32_t msg_boot = 0xF0030002;
	uint32_t msg_size = sz;

	fprintf(stderr,"sending 2ndstage to target... %08x\n",msg_boot);
	usb_write(usb, &msg_boot, sizeof(msg_boot));
	usb_write(usb, &msg_size, sizeof(msg_size));
	usb_write(usb, data, sz);

	if ((data2) || (fastboot_mode)) {
		fprintf(stderr,"waiting for 2ndstage response...\n");
		usb_read(usb, &msg_size, sizeof(msg_size));
		if (msg_size != 0xaabbccdd) {
			fprintf(stderr,"unexpected 2ndstage response\n");
			return -1;
		}

	/* In fastboot mode, we stay in SRAM so don't
	download data2 to the target. Return back from here */
	if (fastboot_mode) {
		fprintf(stderr, "received 2ndstage response...\n");
		return 0;
	}

		msg_size = sz2;

		fprintf(stderr, "sending image to target...size "
				"(%d-B/%d-KB/%d-MB)\n", msg_size,
				msg_size/1024, msg_size/(1024 * 1024));

		usb_write(usb, &msg_size, sizeof(msg_size));
		usb_write(usb, data2, sz2);
	}
	
	return 0;
}

int match_omap4_bootloader(usb_ifc_info *ifc)
{
	if (ifc->dev_vendor != 0x0451)
		return -1;
	if ((ifc->dev_product != 0xd010) && (ifc->dev_product != 0xd00f) &&
		(ifc->dev_product != 0xd011) &&  (ifc->dev_product != 0xd012))
		return -1;
	return 0;
}

void *load_file(const char *file, unsigned *sz)
{
	void *data;
	struct stat s;
	int fd;
	
	fd = open(file, O_RDONLY);
	if (fd < 0)
		return 0;
	
	if (fstat(fd, &s))
		goto fail;
	
	data = malloc(s.st_size);
	if (!data)
		goto fail;
	
	if (read(fd, data, s.st_size) != s.st_size) {
		free(data);
		goto fail;
	}
	
	close(fd);
	*sz = s.st_size;
	return data;
	
fail:
	close(fd);
	return 0;
}

static int usage(void)
{
	fprintf(stderr, "\nusbboot syntax and options:\n\n");
	fprintf(stderr, "usbboot [ <2ndstage> ] <image>\n");
	fprintf(stderr, "=================================================\n");
	fprintf(stderr, "example: ./out/<board>/usbboot u-boot.bin\n");
	fprintf(stderr, "---- ---- ---- ---- OR ---- ---- ---- ----\n");
	fprintf(stderr, "example: ./out/<board>/usbboot out/<board>/aboot.bin "
			"u-boot.bin\n");
	fprintf(stderr, "==>this will download and execute aboot second \n"
			"stage in SRAM and then download and execute \n"
			"u-boot.bin in SDRAM\n");
	fprintf(stderr, "=================================================\n");
	fprintf(stderr, "example: ./out/<board>/usbboot -f \n");
	fprintf(stderr, "---- ---- ---- ---- OR ---- ---- ---- ----\n");
	fprintf(stderr, "example: ./out/<board>/usbboot -f "
						"out<board>/iboot.ift\n");
	fprintf(stderr, "==>this will download and execute iboot second \n"
			"stage in SRAM along with the configuration header\n"
			"(CH) and then enter into FASTBOOT mode\n");
	fprintf(stderr, "=================================================\n");

	return 0;
}

extern unsigned char aboot_data[];
extern unsigned aboot_size;

extern unsigned char iboot_data[];
extern unsigned iboot_size;

int main(int argc, char **argv)
{
	void *data, *data2;
	unsigned sz, sz2;
	usb_handle *usb = NULL;
	int once = 1;
	int fastboot_mode = 0;
	char proctype[8];

	if ((argc < 2) || (argc > 3)) {
		usage();
		return 0;
	}

	if ((argv[1][0] == '-') && ((argv[1][1] == 'f') ||
	(argv[1][1] == 'F'))) {
		fprintf(stderr, "usbboot -f:  starting in fastboot mode\n");
		for (;;) {
			usb = usb_open(match_omap4_bootloader);
			if (usb) {
				strcpy(proctype, usb_boot_read_chip_info(usb));
				if (!memcmp(proctype, "EMU", 3)) {
					data = load_file("iboot.ift", &sz);
					if (!data) {
						fprintf(stderr, "unable to "
						"load signed ED/HD (EMU/HS)"
						"iboot.ift\n");
						return -1;
					}
				 } else {
					fprintf(stderr, "using built-in GP "
						"iboot of size %d-KB\n",
							iboot_size/1024);
					data = iboot_data;
					sz = iboot_size;
				}

				break;
			}

			if (once) {
				once = 0;
				fprintf(stderr, "waiting for device...\n");
			}
			usleep(250);
		}

		once = 1;
		fastboot_mode = 1;
		data2 = 0;
		sz2 = 0;
	} else {
		if (argc < 3) {
			fprintf(stderr, "using built-in 2ndstage.bin of size"
						" %d-KB\n", aboot_size/1024);
			data = aboot_data;
			sz = aboot_size;
		} else {
			data = load_file(argv[1], &sz);
			if (data == 0) {
				fprintf(stderr, "cannot load '%s'\n", argv[1]);
				usage();
				return -1;
			}
			argc--;
			argv++;
		}

		data2 = load_file(argv[1], &sz2);
		if (data2 == 0) {
			fprintf(stderr, "cannot load '%s'\n", argv[1]);
			usage();
			return -1;
		}
	}

	for (;;) {
		if (usb == NULL)
			usb = usb_open(match_omap4_bootloader);
		if (usb)
			return usb_boot(usb, fastboot_mode,
					data, sz, data2, sz2);
		if (once) {
			once = 0;
			fprintf(stderr, "waiting for device...\n");
		}
		usleep(250);
	}
	
	return -1;    
}
