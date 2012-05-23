
#include <aboot/aboot.h>
#include <aboot/io.h>

#if defined CONFIG_IS_OMAP4
#include <omap4/mux.h>
#include <omap4/hw.h>
#elif defined CONFIG_IS_OMAP5
#include <omap5/hw.h>
#endif

u32 wait_on_value(u32 read_bit_mask, u32 match_value, u32 read_addr, u32 bound)
{
	u32 i = 0, val;
	do {
		++i;
		val = readl(read_addr) & read_bit_mask;
		if (val == match_value)
			return (1);
		if (i == bound)
			return (0);
	} while (1);
}

void sdelay(unsigned long loops)
{
	__asm__ volatile ("1:\n" "subs %0, %1, #1\n"
			  "bne 1b":"=r" (loops):"0"(loops));
}

void set_modify(u32 reg, u32 mask, u32 value)
{
	u32 read = readl(reg);
	u32 reg_value = ((read & ~(mask)) | value);

	writel(reg_value, reg);
}
