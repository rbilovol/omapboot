##
## Copyright (C) 2010 The Android Open Source Project
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##  * Redistributions of source code must retain the above copyright
##    notice, this list of conditions and the following disclaimer.
##  * Redistributions in binary form must reproduce the above copyright
##    notice, this list of conditions and the following disclaimer in
##    the documentation and/or other materials provided with the 
##    distribution.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
## FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
## COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
## INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
## BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
## OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
## AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
## OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
## OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
## SUCH DAMAGE.

VERSION = 1
PATCHLEVEL = 1
SUBLEVEL = 0
ORGANIZATION="Texas Instruments Inc"
VERSION_FILE = include/version.h
ABOOT_VERSION = $(VERSION).$(PATCHLEVEL).$(SUBLEVEL)

what_to_build:: all

-include local.mk

TOOLCHAIN ?= arm-eabi-

BOARD ?= panda
ARCH ?= omap4
EXTRAOPTS ?= -m32

TARGET_CC := $(TOOLCHAIN)gcc
TARGET_LD := $(TOOLCHAIN)ld
TARGET_OBJCOPY := $(TOOLCHAIN)objcopy
TARGET_OBJDUMP := $(TOOLCHAIN)objdump

$(shell cat arch/$(ARCH)/configs/config_$(BOARD).h > include/config.h)
TARGET_CFLAGS := -g -Os  -Wall
TARGET_CFLAGS +=  -march=armv7-a -fno-builtin -ffreestanding
TARGET_CFLAGS += -I. -Iinclude
TARGET_CFLAGS += -include include/config.h

TARGET_LIBGCC := $(shell $(TARGET_CC) $(TARGET_CFLAGS) -print-libgcc-file-name)

HOST_CFLAGS := -g -O2 -Wall $(EXTRAOPTS)
HOST_CFLAGS += -Itools
HOST_CFLAGS += -include include/config.h

OUT := out/$(BOARD)
OUT_HOST_OBJ := $(OUT)/host-obj
OUT_TARGET_OBJ := $(OUT)/target-obj

ALL :=

# Build the usbboot host tool
include build/rules.mk
include tools/host_usbboot.mk

# Build the target with it's dependencies
include arch/$(ARCH)/board/$(BOARD).mk

COMMON_OBJS := 	crc32.o \
		libc/utils.o \
		fastboot.o \
		fastboot_mmc.o \
		libc/printf.o \
		libc/raise.o \
		libc/string.o \
		trusted.o \
		boot.o \
		misc.o \

M_NAME := aboot
LDS :=  build/$(M_NAME).lds
M_BASE := $(ABOOT_TEXT_BASE)
M_OBJS := arch/common/start_reloc.o
M_OBJS += $(OMAP_COMMON_OBJS)
M_OBJS += $(COMMON_OBJS)
M_OBJS += $(PROC_COMMON_OBJS)
M_OBJS += $(BOARD_OBJS)
M_OBJS += booti.o
M_OBJS += aboot.o
M_LIBS := $(TARGET_LIBGCC)
include build/target-executable.mk

M_NAME := iboot
LDS :=  build/$(M_NAME).lds
M_BASE := $(IBOOT_TEXT_BASE)
M_OBJS := iboot/start_reloc.o
M_OBJS += $(OMAP_COMMON_OBJS)
M_OBJS += $(COMMON_OBJS)
M_OBJS += $(PROC_COMMON_OBJS)
M_OBJS += $(BOARD_OBJS)
M_OBJS += booti.o
M_OBJS += iboot/iboot.o
M_OBJS += pmic.o
M_LIBS := $(TARGET_LIBGCC)
include build/target-executable.mk

M_NAME := eboot
LDS :=  build/$(M_NAME).lds
M_BASE := $(EBOOT_TEXT_BASE)
M_OBJS := eboot/start_reloc.o
M_OBJS += $(OMAP_COMMON_OBJS)
M_OBJS += $(COMMON_OBJS)
M_OBJS += $(PROC_COMMON_OBJS)
M_OBJS += $(BOARD_OBJS)
M_OBJS += booti.o
M_OBJS += eboot/eboot.o
M_OBJS += pmic.o
M_LIBS := $(TARGET_LIBGCC)
include build/target-executable.mk

$(OUT)/eboot.ift: $(OUT)/eboot.bin $(OUT)/mkheader
	@echo generate $@
	@./$(OUT)/mkheader $(EBOOT_TEXT_BASE) `wc -c $(OUT)/eboot.bin` add_gp_hdr > $@
	@cat $(OUT)/eboot.bin >> $@
	@echo Renaming eboot.ift to MLO ...Done!
	@cp $(OUT)/eboot.ift $(OUT)/MLO

$(OUT)/iboot.ift: $(OUT)/iboot.bin $(OUT)/mkheader
	@echo generate $@
	@./$(OUT)/mkheader $(IBOOT_TEXT_BASE) `wc -c $(OUT)/iboot.bin` no_gp_hdr > $@
	@cat $(OUT)/iboot.bin >> $@

$(OUT)/aboot.ift: $(OUT)/aboot.bin $(OUT)/mkheader
	@echo generate $@
	@./$(OUT)/mkheader $(ABOOT_TEXT_BASE) `wc -c $(OUT)/aboot.bin` no_gp_hdr > $@
	@cat $(OUT)/aboot.bin >> $@

ALL += $(OUT)/aboot.ift $(OUT)/iboot.ift $(OUT)/eboot.ift

$(OUT_HOST_OBJ)/2ndstage.o: $(OUT)/aboot.bin $(OUT)/bin2c $(OUT)/mkheader
	@echo generate $@
	@./$(OUT)/mkheader $(ABOOT_TEXT_BASE) `wc -c $(OUT)/aboot.bin` no_gp_hdr > $@
	@cat $(OUT)/aboot.bin >> $@
	$(QUIET)./$(OUT)/bin2c aboot < $@ > $(OUT)/2ndstage.c
	gcc -c $(EXTRAOPTS) -o $@ $(OUT)/2ndstage.c

$(OUT_HOST_OBJ)/secondstage.o: $(OUT)/iboot.bin $(OUT)/bin2c $(OUT)/mkheader
	@echo generate $@
	@./$(OUT)/mkheader $(IBOOT_TEXT_BASE) `wc -c $(OUT)/iboot.bin` no_gp_hdr > $@
	@cat $(OUT)/iboot.bin >> $@
	$(QUIET)./$(OUT)/bin2c iboot < $@ > $(OUT)/secondstage.c
	gcc -c $(EXTRAOPTS) -o $@ $(OUT)/secondstage.c
clean::
	@echo clean
	@rm -f include/config.h
	@rm -f include/version.h
	@rm -rf $(OUT)

all:: version $(ALL)

version:
	@echo -n "#define ABOOT_VERSION \""$(ORGANIZATION)" Bootloader " > $(VERSION_FILE); \
	echo -n "$(ABOOT_VERSION)" >> $(VERSION_FILE); \
	echo -n $(shell $(CONFIG_SHELL) build/getgitinfo \
		 $(TOPDIR)) >> $(VERSION_FILE); \
		echo "\"" >> $(VERSION_FILE)

# we generate .d as a side-effect of compiling. override generic rule:
%.d:
-include $(DEPS)
