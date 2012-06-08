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

MKDIR = if [ ! -d $(dir $@) ]; then mkdir -p $(dir $@); fi

QUIET ?= @

$(OUT_HOST_OBJ)/%.o: %.c
	@$(MKDIR)
ifneq ($(HCHECKSRC),0)
	@echo check $<
	$(QUIET)$(CHECK) $(HOST_CHECK_FLAGS) $(HOST_CFLAGS) $<
endif
	@echo compile $<
	$(QUIET)$(CC) $(HOST_CFLAGS) -c $< -o $@ -MD -MT $@ -MF $(@:%o=%d)
$(OUT_HOST_OBJ)/%.o: %.S
	@$(MKDIR)
	@echo assemble $<
	$(QUIET)$(CC) $(HOST_CFLAGS) -c $< -o $@ -MD -MT $@ -MF $(@:%o=%d)

$(OUT_TARGET_OBJ)/%.o: %.c
	@$(MKDIR)
ifneq ($(TCHECKSRC),0)
	@echo check $<
	$(QUIET)$(CHECK) $(TARGET_CHECK_FLAGS) $(TARGET_CFLAGS) $<
endif
	@echo compile $<
	$(QUIET)$(TARGET_CC) $(TARGET_CFLAGS) -c $< -o $@ -MD -MT $@ -MF $(@:%o=%d)
$(OUT_TARGET_OBJ)/%.o: %.S
	@$(MKDIR)
	@echo assemble $<
	$(QUIET)$(TARGET_CC) $(TARGET_AFLAGS) -c $< -o $@ -MD -MT $@ -MF $(@:%o=%d)
