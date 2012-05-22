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
