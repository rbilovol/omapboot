MEMORY {
	sram (A): ORIGIN = 0x40300200, LENGTH = 48K
	mram (A): ORIGIN = 0x40310000, LENGTH = 1K
	sdram (Arw) : ORIGIN = 0xA0BFFFFF, LENGTH = 500m
}

ENTRY(_start)
SECTIONS {
	.text : { *(.text) } > sram

	. = ALIGN(4);
	.got : { *(.got) }

	. = ALIGN(4);
	.rodata : { *(.rodata) } > sram

	. = ALIGN(4);
	.data : { *(.data) } > sram

	. = ALIGN(4);
	.mram : { *(.mram) } > mram

	. = ALIGN(4);
	__bss_start = .;
	.bss : { *(.bss) } > sram
	. = ALIGN(4);
	__bss_end = .;

	. = ALIGN(4);
	.sdram (NOLOAD): { *(.sdram) } > sdram
}
