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
##

M_OBJS := $(addprefix $(OUT_TARGET_OBJ)/,$(M_OBJS))
DEPS += $(M_OBJS:%o=%d)

ALL += $(OUT)/$(M_NAME).bin
ALL += $(OUT)/$(M_NAME).lst

$(OUT)/$(M_NAME).bin: _SRC := $(OUT)/$(M_NAME)
$(OUT)/$(M_NAME).bin: $(OUT)/$(M_NAME)
	@echo create $@
	$(QUIET)$(TARGET_OBJCOPY) --gap-fill=0xee -O binary $(_SRC) $@

$(OUT)/$(M_NAME).lst: _SRC := $(OUT)/$(M_NAME)
$(OUT)/$(M_NAME).lst: $(OUT)/$(M_NAME)
	@echo create $@
	$(QUIET)$(TARGET_OBJDUMP) -D $(_SRC) > $@

$(OUT)/$(M_NAME): _OBJS := $(M_OBJS)
$(OUT)/$(M_NAME): _LIBS := $(M_LIBS)
$(OUT)/$(M_NAME): _BASE := $(M_BASE)
$(OUT)/$(M_NAME): _LDS := $(M_LDS)
$(OUT)/$(M_NAME): _MAP := $(M_MAP)
$(OUT)/$(M_NAME): $(M_OBJS)
	@echo link $@
	$(QUIET)$(TARGET_LD) -Bstatic -Map $(_MAP) -T $(_LDS) -Ttext $(_BASE) $(_OBJS) $(_LIBS) -o $@

M_OBJS :=
M_NAME :=
M_BASE :=
M_LIBS :=
M_LDS :=
M_MAP :=
