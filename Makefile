##
## Copyright (C) 2010 The Android Open Source Project
## All rights reserved.
##
## Copyright (C) 2012 Texas Instruments, Inc.
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
VERSION_FILE = include/common/version.h
ABOOT_VERSION = $(VERSION).$(PATCHLEVEL).$(SUBLEVEL)
USER_PARAMS_FILE = include/common/user_params.h

what_to_build:: all

-include local.mk

ifeq ("$(origin V)", "command line")
  VERBOSE = $(V)
endif
ifndef VERBOSE
  VERBOSE = 0
endif
QUIET := $(if $(VERBOSE:1=),@)

#if TOOLCHAIN is defined, override CROSS_COMPILE, else use CROSS_COMPILE
ifneq ("$(TOOLCHAIN)", "")
	CROSS_COMPILE := $(TOOLCHAIN)
else
	CROSS_COMPILE ?= arm-eabi-
endif

HYP_MODE := $(HYP_MODE)
MSHIELD := $(MSHIELD)
BOARD ?= panda
ARCH ?= arm
MACH ?= omap4
EXTRAOPTS ?= -m32
GENERATE_ULO = generate_ULO
ULO = ULO
BOARD_MLO = $(BOARD)
DUAL_STAGE :=
ifeq ("$(MACH)", "omap4")
	DUAL_STAGE := 1
	GENERATE_ULO := generate_2ND
	ULO := 2ND
# Need to specify right board name to generate MLO files with correct names
ifeq ("$(BOARD)", "blaze_tablet")
	BOARD_MLO := Blaze_Tablet
else ifeq ("$(BOARD)", "blaze")
	BOARD_MLO := Blaze
endif
else
ifeq ("$(SPLIT)", "1")
	DUAL_STAGE := 1
else
BOOT_DS_OBJS := booti.o \
		fastboot.o \
		fastboot_mmc.o \
		device_tree.o
endif
endif

TARGET_CC := $(CROSS_COMPILE)gcc
TARGET_LD := $(CROSS_COMPILE)ld
TARGET_OBJCOPY := $(CROSS_COMPILE)objcopy
TARGET_OBJDUMP := $(CROSS_COMPILE)objdump

$(shell cat arch/$(MACH)/configs/config_$(BOARD).h > include/common/config.h)
$(shell echo "" > $(USER_PARAMS_FILE))

TARGET_FLAGS := -g -Os  -Wall
TARGET_FLAGS += -fno-builtin -ffreestanding
TARGET_FLAGS += -I. -Iinclude/$(MACH)
TARGET_FLAGS += -I. -Iinclude/aboot
TARGET_FLAGS += -I. -Iinclude/libc
TARGET_FLAGS += -I. -Iinclude/common
TARGET_FLAGS += -include include/common/config.h

# Newer GCC require +sec postfix to build secure instructions
plus_sec := $(shell set -e;						\
		if (echo ".arch_extension sec" |			\
		$(TARGET_CC) -c -march=armv7-a -x assembler -o /dev/null -) \
		>/dev/null 2>/dev/null;	\
		then echo "+sec";					\
		else echo "";						\
		fi)
TARGET_AFLAGS := $(TARGET_FLAGS) -Wa,-march=armv7-a$(plus_sec)
TARGET_CFLAGS := $(TARGET_FLAGS) -march=armv7-a -mthumb -ffunction-sections -fdata-sections
TARGET_CC_LOC := $(shell $(TARGET_CC) $(TARGET_CFLAGS) -print-search-dirs|grep ^install|cut -d ':' -f2|tr -d ' ')

TARGET_LIBGCC := $(shell $(TARGET_CC) $(TARGET_CFLAGS) -print-libgcc-file-name)

HOST_CFLAGS := -g -O2 -Wall $(EXTRAOPTS)
HOST_CFLAGS += -Ihost/include/common
HOST_CFLAGS += -Ihost/include/$(MACH)
HOST_CFLAGS += -include include/common/config.h
HOST_CC_LOC := $(shell $(CC) $(HOST_CFLAGS) -print-search-dirs|grep ^install|cut -d ':' -f2|tr -d ' ')

# C=1 to enable check of modified source for target build only
# C=2 to enable check of modified source for Host build only
# C=3 to enable check of modified source for target and Host build
# C=0 or no define to not check (default)
ifeq ("$(origin C)", "command line")
	CHECKSRC := $(C)
endif
ifndef CHECKSRC
	TCHECKSRC := 0
	HCHECKSRC := 0
endif
ifeq ($(CHECKSRC), 1)
	TCHECKSRC = 1
	HCHECKSRC = 0
endif
ifeq ($(CHECKSRC), 2)
	TCHECKSRC = 0
	HCHECKSRC = 1
endif
ifeq ($(CHECKSRC), 3)
	TCHECKSRC = 1
	HCHECKSRC = 1
endif
CHECK ?= sparse
CHECK_FLAGS +=  -Wbitwise -Wno-return-void -D__STDC__
HOST_CHECK_FLAGS +=  $(CHECK_FLAGS) -I$(HOST_CC_LOC)/include
TARGET_CHECK_FLAGS +=  $(CHECK_FLAGS) -I$(TARGET_CC_LOC)/include

OUT := out/$(BOARD)
OUT_HOST_OBJ := $(OUT)/host-obj
OUT_TARGET_OBJ := $(OUT)/target-obj

COMMON_OBJS := 	crc32.o \
		libc/utils.o \
		boot_common.o \
		libc/printf.o \
		libc/raise.o \
		libc/string.o \
		trusted.o \
		arch/common/misc.o

target_dep:
# Build the target with it's dependencies
include arch/$(MACH)/$(MACH).mk

ifeq ($(DUAL_STAGE), 1)
ALL := two_stage

two_stage:
M_NAME := sboot
M_LDS :=  arch/$(MACH)/$(M_NAME).lds
M_MAP :=  $(OUT)/$(M_NAME).map
M_BASE := $(SBOOT_TEXT_BASE)
M_OBJS := sboot/start.o
M_OBJS += $(OMAP_COMMON_OBJS)
M_OBJS += $(COMMON_OBJS)
M_OBJS += $(PROC_COMMON_OBJS)
M_OBJS += $(BOARD_OBJS)
M_OBJS += sboot/sboot.o
M_OBJS += arch/common/rom_usb_stubs.o
M_OBJS += booti.o
M_OBJS += fastboot_common.o
M_OBJS += fastboot.o
M_OBJS += fastboot_mmc.o
M_OBJS += device_tree.o
M_LIBS := $(TARGET_LIBGCC)
include build/target-executable.mk

ALL += second_stage_object_size
else
ALL :=
endif

usbboot:
# Build the usbboot host tool
include build/rules.mk
include host/tools/host_usbboot.mk

boot_modules:
M_NAME := aboot
M_LDS :=  arch/$(MACH)/$(M_NAME).lds
M_MAP :=  $(OUT)/$(M_NAME).map
M_BASE := $(ABOOT_TEXT_BASE)
M_OBJS := arch/common/start.o
M_OBJS += $(OMAP_COMMON_OBJS)
M_OBJS += $(COMMON_OBJS)
M_OBJS += $(PROC_COMMON_OBJS)
M_OBJS += $(BOARD_OBJS)
M_OBJS += aboot.o
M_OBJS += arch/common/rom_usb.o
M_OBJS += booti.o
M_OBJS += fastboot_common.o
M_OBJS += fastboot.o
M_OBJS += fastboot_mmc.o
M_OBJS += device_tree.o
M_LIBS := $(TARGET_LIBGCC)
include build/target-executable.mk

M_NAME := iboot
M_LDS :=  arch/$(MACH)/$(M_NAME).lds
M_MAP :=  $(OUT)/$(M_NAME).map
M_BASE := $(IBOOT_TEXT_BASE)
M_OBJS := iboot/start.o
M_OBJS += $(OMAP_COMMON_OBJS)
M_OBJS += $(COMMON_OBJS)
M_OBJS += $(PROC_COMMON_OBJS)
M_OBJS += $(BOARD_OBJS)
M_OBJS += iboot/iboot.o
M_OBJS += arch/common/rom_usb.o
M_OBJS += fastboot_common.o
M_OBJS += $(BOOT_DS_OBJS)
M_LIBS := $(TARGET_LIBGCC)
include build/target-executable.mk

M_NAME := eboot
M_LDS :=  arch/$(MACH)/$(M_NAME).lds
M_MAP :=  $(OUT)/$(M_NAME).map
M_BASE := $(EBOOT_TEXT_BASE)
M_OBJS := eboot/start.o
M_OBJS += $(OMAP_COMMON_OBJS)
M_OBJS += $(COMMON_OBJS)
M_OBJS += $(PROC_COMMON_OBJS)
M_OBJS += $(BOARD_OBJS)
M_OBJS += eboot/eboot.o
M_OBJS += arch/common/rom_usb.o
M_OBJS += fastboot_common.o
M_OBJS += $(BOOT_DS_OBJS)
M_LIBS := $(TARGET_LIBGCC)
include build/target-executable.mk

# Override this with non 0 value to make MAKEALL continue build ignoring
# Failures - default behavior is to quit on detecting error
MAKEALL_LENIENT ?= 0

SIGN_MLO_OMAP4_FORMAT = \
echo "Signing eboot.bin to generate signed $(BOARD_MLO)_OMAP$1_HS_$2_MLO..." ;\
$(MSHIELD)/generate_MLO OMAP$1 $2 $(OUT)/eboot.bin ;\
cat MLO >> $(OUT)/$(BOARD_MLO)_OMAP$1_HS_$2_MLO ; rm MLO ; echo "Done!"

SIGN_MLO_OMAP5_FORMAT = \
echo "Signing eboot.bin to generate signed $(BOARD_MLO)_HS_$2_MLO..." ;\
$(MSHIELD)/generate_MLO OMAP$1 $2 $(OUT)/eboot.bin ;\
cat MLO >> $(OUT)/$(BOARD_MLO)_HS_$2_MLO ; rm MLO ; echo "Done!"

SIGN_ULO = \
echo "generate $@" ;\
echo "Signing iboot.bin to generate signed $(BOARD_MLO)_OMAP$1_HS_$2.$3_ULO..." ;\
$(MSHIELD)/$(GENERATE_ULO) OMAP$1 $2.$3 $(OUT)/iboot.bin ;\
./$(OUT)/mkheader $(IBOOT_TEXT_BASE) `wc -c $(ULO)` no_gp_hdr > $@ ;\
cat $(ULO) >> $@ ; rm $(ULO) ;\
./$(OUT)/bin2c iboot_hs_$1_$2 < $@ > $(OUT)/iboot_hs_$1_$2.c ;\
gcc -c $(EXTRAOPTS) -o $@ $(OUT)/iboot_hs_$1_$2.c ; echo "Done!"

.EXPORT_ALL_VARIABLES:

$(OUT)/$(BOARD)_GP_MLO:: $(OUT)/eboot.bin $(OUT)/mkheader
	@echo generate $@
	$(QUIET)./$(OUT)/mkheader $(EBOOT_TEXT_BASE) `wc -c $(OUT)/eboot.bin` add_gp_hdr > $@
	$(QUIET)cat $(OUT)/eboot.bin >> $@
	@echo ...Done!
ifeq ("$(MACH)", "omap4")
	cp $@ $(OUT)/$(BOARD_MLO)_OMAP4430_GP_ES2.2_MLO
	cp $@ $(OUT)/$(BOARD_MLO)_OMAP4430_GP_ES2.3_MLO
	cp $@ $(OUT)/$(BOARD_MLO)_OMAP4460_GP_ES1.0_MLO
	cp $@ $(OUT)/$(BOARD_MLO)_OMAP4460_GP_ES1.1_MLO
	cp $@ $(OUT)/$(BOARD_MLO)_OMAP4470_GP_ES1.0_MLO
	rm $@
endif
ifeq ("$(MACH)", "omap5")
	cp $@ $(OUT)/$(BOARD_MLO)_GP_ES1.0_MLO
	cp $@ $(OUT)/$(BOARD_MLO)_GP_ES2.0_MLO
	rm $@
endif

$(OUT)/$(BOARD)_HS_MLO: $(OUT)/eboot.bin $(OUT)/mkheader
ifneq ("$(MSHIELD)", "")
ifeq ("$(MACH)", "omap4")
	$(call SIGN_MLO_OMAP4_FORMAT,4430,ES2.2)
	$(call SIGN_MLO_OMAP4_FORMAT,4430,ES2.3)
	$(call SIGN_MLO_OMAP4_FORMAT,4460,ES1.0)
	$(call SIGN_MLO_OMAP4_FORMAT,4460,ES1.1)
	$(call SIGN_MLO_OMAP4_FORMAT,4470,ES1.0)
endif
ifeq ("$(MACH)", "omap5")
	$(call SIGN_MLO_OMAP5_FORMAT,5430,ES1.0)
	$(call SIGN_MLO_OMAP5_FORMAT,5430,ES2.0)
endif
endif

$(OUT)/aboot.ift: $(OUT)/aboot.bin $(OUT)/mkheader
	@echo generate $@
	$(QUIET)./$(OUT)/mkheader $(ABOOT_TEXT_BASE) `wc -c $(OUT)/aboot.bin` no_gp_hdr > $@
	$(QUIET)cat $(OUT)/aboot.bin >> $@

ALL += boot_modules $(OUT)/aboot.ift $(OUT)/$(BOARD)_GP_MLO $(OUT)/$(BOARD)_HS_MLO

$(OUT_HOST_OBJ)/2ndstage.o: $(OUT)/aboot.bin $(OUT)/bin2c $(OUT)/mkheader
	@echo generate $@
	$(QUIET)./$(OUT)/mkheader $(ABOOT_TEXT_BASE) `wc -c $(OUT)/aboot.bin` no_gp_hdr > $@
	$(QUIET)cat $(OUT)/aboot.bin >> $@
	$(QUIET)./$(OUT)/bin2c aboot < $@ > $(OUT)/2ndstage.c
	$(QUIET)gcc -c $(EXTRAOPTS) -o $@ $(OUT)/2ndstage.c

$(OUT_HOST_OBJ)/iboot_gp.o: $(OUT)/iboot.bin $(OUT)/bin2c $(OUT)/mkheader
	@echo generate $@
	$(QUIET)./$(OUT)/mkheader $(IBOOT_TEXT_BASE) `wc -c $(OUT)/iboot.bin` no_gp_hdr > $@
	$(QUIET)cat $(OUT)/iboot.bin >> $@
	$(QUIET)./$(OUT)/bin2c iboot_gp < $@ > $(OUT)/iboot_gp.c
	$(QUIET)gcc -c $(EXTRAOPTS) -o $@ $(OUT)/iboot_gp.c
	@echo ...Done!

ifneq ("$(MSHIELD)", "")
ifeq ("$(MACH)", "omap4")
$(OUT_HOST_OBJ)/iboot_hs_4430_ES2.o: $(OUT)/iboot.bin $(OUT)/bin2c $(OUT)/mkheader
	$(call SIGN_ULO,4430,ES2,3)
$(OUT_HOST_OBJ)/iboot_hs_4460_ES1.o: $(OUT)/iboot.bin $(OUT)/bin2c $(OUT)/mkheader
	$(call SIGN_ULO,4460,ES1,0)
$(OUT_HOST_OBJ)/iboot_hs_4470_ES1.o: $(OUT)/iboot.bin $(OUT)/bin2c $(OUT)/mkheader
	$(call SIGN_ULO,4470,ES1,0)
endif
ifeq ("$(MACH)", "omap5")
$(OUT_HOST_OBJ)/iboot_hs_5430_ES1.o: $(OUT)/iboot.bin $(OUT)/bin2c $(OUT)/mkheader
	$(call SIGN_ULO,5430,ES1,0)
$(OUT_HOST_OBJ)/iboot_hs_5430_ES2.o: $(OUT)/iboot.bin $(OUT)/bin2c $(OUT)/mkheader
	$(call SIGN_ULO,5430,ES2,0)
endif
endif

ifeq ($(DUAL_STAGE), 1)
$(OUT_HOST_OBJ)/sboot-bin.o: $(OUT)/sboot.bin $(OUT)/bin2c
	@echo generate $@
	$(QUIET)cat $(OUT)/sboot.bin >> $@
	$(QUIET)$(OUT)/bin2c sboot < $@ > $(OUT)/sboot-bin.c
	$(QUIET)gcc -c $(EXTRAOPTS) -o $@ $(OUT)/sboot-bin.c
endif

_clean_generic::
	$(QUIET)rm -f include/common/config.h
	$(QUIET)rm -f include/common/version.h
	$(QUIET)rm -f include/common/user_params.h
	$(QUIET)rm -rf out

clean::
	@echo clean
	$(QUIET)$(MAKE) _clean_generic

distclean::
	@echo distclean
	$(QUIET)$(MAKE) _clean_generic
	$(QUIET)find . -iname "*~" -o -iname "*.d" -o -iname "*.rej" -o \
		-iname "*.orig" |\
		xargs rm -f tags

.PHONY:	tags

all:: user_params usbboot version target_dep $(ALL)

version:
	$(QUIET)echo -n "#define ABOOT_VERSION \""$(ORGANIZATION)" Bootloader " > $(VERSION_FILE); \
	echo -n "$(ABOOT_VERSION)" >> $(VERSION_FILE); \
	echo -n $(shell $(CONFIG_SHELL) build/getgitinfo \
		 $(TOPDIR)) >> $(VERSION_FILE); \
		echo "\"" >> $(VERSION_FILE)

tags:
	@echo "Generating Tags"
	$(QUIET)find . -type f -iname \*.h -o -iname \*.c -o -iname \*.S | \
		xargs ctags

user_params:
	$(QUIET)rm -rf out/$(BOARD)
ifeq ($(DUAL_STAGE), 1)
	@echo "defining TWO_STAGE_OMAPBOOT"
	$(QUIET)echo -n "#define TWO_STAGE_OMAPBOOT  1" > $(USER_PARAMS_FILE)
	$(QUIET)echo "" >> $(USER_PARAMS_FILE)
endif
ifneq ("$(MSHIELD)", "")
	@echo "defining EMBED_IBOOT_HS"
	$(QUIET)echo -n "#define EMBED_IBOOT_HS 1" >> $(USER_PARAMS_FILE)
	$(QUIET)echo "" >> $(USER_PARAMS_FILE)
endif
ifneq ("$(HYP_MODE)", "")
	@echo "defining START_HYPERVISOR_MODE"
	$(QUIET)echo -n "#define START_HYPERVISOR_MODE 1" >> $(USER_PARAMS_FILE)
	$(QUIET)echo "" >> $(USER_PARAMS_FILE)
endif

second_stage_object_size:
ifeq ($(DUAL_STAGE), 1)
	@echo "defining SECOND_STAGE_OBJECT_SIZE"
	$(QUIET)echo -n "#define SECOND_STAGE_OBJECT_SIZE " >> $(USER_PARAMS_FILE)
	$(QUIET)cat $(OUT)/sboot.bin | wc -c >>  $(USER_PARAMS_FILE)
endif

MAKEALL:
	$(QUIET)rm -rf out
	@echo "checking available build platforms"
	$(QUIET)TMP_FILE="/tmp/make-all.$$$$.tmp";			\
		TMP_FILE2="/tmp/make-all-res.$$$$.tmp";			\
		ls arch/*/configs/*.h|cut -d '.' -f1 |			\
		sed -e "s/arch\//MACH=/g" |				\
		sed -e "s/\/configs\/config_/ BOARD=/g">"$$TMP_FILE" &&	\
		while read config; do 					\
		echo "Building $$config"; $(MAKE) $$config;R="$$?";	\
		if [ $$R -eq 0 ]; then					\
			echo "$$config : BUILD OK" >> "$$TMP_FILE2";	\
		else							\
			echo "$$config : BUILD FAIL">> "$$TMP_FILE2";	\
			if [ $(MAKEALL_LENIENT) -eq 0 ]; then		\
				cat "$$TMP_FILE2";			\
				echo "Further Builds stopped!";		\
				echo "use MAKEALL_LENIENT=1 to go on";	\
				rm -f "$$TMP_FILE" "$$TMP_FILE2";	\
				exit 1;					\
			fi;						\
		fi;							\
		done <"$$TMP_FILE";					\
		echo "Build with configs:";cat "$$TMP_FILE2";		\
		rm -f "$$TMP_FILE" "$$TMP_FILE2"

# we generate .d as a side-effect of compiling. override generic rule:
%.d:
-include $(DEPS)
