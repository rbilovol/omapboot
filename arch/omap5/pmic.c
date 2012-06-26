/*
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
#include <omap5/hw.h>
#include <common/omap_rom.h>
#include <common/common_proc.h>
#include <libc/string.h>

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

int palmas_read_sw_revision(void)
{
	int ret = 0;

	hal_i2c i2c_id = HAL_I2C1;
	u32 clk32;
	u16 slave; u16 reg_addr; u16 cmd;

	ret = i2c_init(i2c_id);
	if (ret != 0) {
		printf("Failed to init I2C-%d\n", i2c_id);
		return ret;
	} else
		DBG("Initialized I2C-%d\n", i2c_id);

	slave = 0x48; reg_addr = 0x17;
	cmd = (reg_addr & 0xFF);
	clk32 = readl(CLK32K_COUNTER_REGISTER);

	ret = i2c_read(i2c_id, slave, 2, &cmd, clk32, 0xFF);
	if (ret != 0) {
		printf("I2C read failed, ret = %d\n", ret);
		return ret;
	} else
		printf("PMIC SW REVISION is = 0x%x\n", cmd);

	ret = i2c_close(i2c_id);
	if (ret != 0) {
		printf("i2c close for module %d failed, ret = %d\n",
							i2c_id, ret);
		return ret;
	} else
		DBG("I2C-%d has been disabled\n", i2c_id);

	return ret;
}

int palmas_configure_pwm_mode(void)
{
	int ret = 0;

	hal_i2c i2c_id = HAL_I2C1;
	u32 clk32; u32 value;
	u16 slave; u16 reg_addr; u16 cmd;

	ret = i2c_init(i2c_id);
	if (ret != 0) {
		printf("Failed to init I2C-%d\n", i2c_id);
		return ret;
	} else
		DBG("Initialized I2C-%d\n", i2c_id);

	slave = 0x48; reg_addr = 0x30; value = 0x0F;
	cmd = (reg_addr & 0xFF) | ((value & 0xFF) << 8);
	clk32 = readl(CLK32K_COUNTER_REGISTER);
	ret = i2c_write(i2c_id, slave, 2, &cmd, clk32, 0xFF);
	if (ret != 0) {
		printf("I2C write failed, ret = %d\n", ret);
		return ret;
	}

#ifdef DEBUG
	if (ret != 0) {
		printf("re-initialize the I2C\n");
		ret = rom_hal_i2c_initialize(i2c_id);
		if (ret != 0) {
			printf("\nFailed to re-init I2C-%d\n", i2c_id);
			return ret;
		} else
			printf("\nInitialized I2C-%d\n", i2c_id);
	}

	/* read back the value written */
	slave = 0x48; reg_addr = 0x30;
	cmd = (reg_addr & 0xFF);
	clk32 = readl(CLK32K_COUNTER_REGISTER);
	ret = i2c_read(i2c_id, slave, 2, &cmd, clk32, 0xFF);
	if (ret != 0) {
		printf("I2C read failed, ret = %d\n", ret);
		return ret;
	} else
		printf("SMPS7_CTRL = 0x%x\n", cmd);
#endif

	ret = i2c_close(i2c_id);
	if (ret != 0) {
		printf("i2c close for module %d failed, ret = %d\n",
							i2c_id, ret);
		return ret;
	} else
		DBG("I2C-%d has been disabled\n", i2c_id);

	return ret;
}

