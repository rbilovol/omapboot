/*
 * Copyright (C) 2011 The Android Open Source Project
 * All rights reserved.
 *
 * Copyright(c) 2012 Texas Instruments.
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

#include <aboot/types.h>
#include <aboot/io.h>
#include <omap4/hw.h>
#include <common/omap_rom.h>
#include <libc/string.h>

int pmic_enable(void)
{
	int ret = 0;

	hal_i2c i2c_id = HAL_I2C1;
	u32 clk32;
	u16 slave = 0x48;
	u16 reg_addr;
	u32 value;
	u16 cmd;

	ret = i2c_init(i2c_id);
	if (ret != 0) {
		printf("\nFailed to init I2C-%d\n", i2c_id);
		return ret;
	} else {
		printf("\nInitialized I2C-%d\n", i2c_id);
	}

	reg_addr = 0x98; value = 0x01;
	cmd = (reg_addr & 0xFF) | ((value & 0xFF) << 8);
	clk32 = readl(CLK32K_COUNTER_REGISTER);
	ret = i2c_write(i2c_id, slave, 2, &cmd, clk32, 0xFF);
	if (ret != 0) {
		printf("I2C write failed, ret = %d\n", ret);
		return ret;
	}

	reg_addr = 0x99; value = 0x03;
	cmd = (reg_addr & 0xFF) | ((value & 0xFF) << 8);
	clk32 = readl(CLK32K_COUNTER_REGISTER);
	ret = i2c_write(i2c_id, slave, 2, &cmd, clk32, 0xFF);
	if (ret != 0) {
		printf("I2C write failed, ret = %d\n", ret);
		return ret;
	}

	reg_addr = 0x9A; value = 0x21;
	cmd = (reg_addr & 0xFF) | ((value & 0xFF) << 8);
	clk32 = readl(CLK32K_COUNTER_REGISTER);
	ret = i2c_write(i2c_id, slave, 2, &cmd, clk32, 0xFF);
	if (ret != 0) {
		printf("I2C write failed, ret = %d\n", ret);
		return ret;
	}

	reg_addr = 0x9B; value = 0x15;
	cmd = (reg_addr & 0xFF) | ((value & 0xFF) << 8);
	clk32 = readl(CLK32K_COUNTER_REGISTER);
	ret = i2c_write(i2c_id, slave, 2, &cmd, clk32, 0xFF);
	if (ret != 0) {
		printf("I2C write failed, ret = %d\n", ret);
		return ret;
	}

	printf("pmic-enabled\n");
	return ret;
}

int pbias_config(void)
{
	int ret = 0;
	u32 value;

	value = readl(CONTROL_PBIAS_LITE);

	value = value | (1 << 22) | (1 << 26);

	writel(value, CONTROL_PBIAS_LITE);

	value = readl(CONTROL_CONF_MMC1);

	value = value | (1 << 31) | (1 << 30) | (1 << 27) | (1 << 26) |
								(1 << 25);

	writel(value, CONTROL_CONF_MMC1);

	printf("pbias-configured\n");
	return ret;
}

int pmic_disable(void)
{
	int ret = 0;
	hal_i2c i2c_id = HAL_I2C1;

	ret = i2c_close(i2c_id);
	if (ret != 0) {
		printf("i2c close for module %d failed, ret = %d\n",
							i2c_id, ret);
		return ret;
	}

	printf("pmic-disabled\n");
	return ret;
}
