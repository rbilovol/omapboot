MEMORY
	{
	sram (A): ORIGIN = 0x40309000, LENGTH = 48K
	mram (Arw) : ORIGIN = 0x40315000, LENGTH = 10k
	}


ENTRY(_start)
SECTIONS {
	.text : { *(.text) } > sram

	. = ALIGN(4);
	.got : { *(.got) }

	. = ALIGN(4);
	.rodata : { *(.rodata) } > sram

	. = ALIGN(4);
	.data : { *(.data) } >sram

	. = ALIGN(4);
	__bss_start = .;
	.bss : { *(.bss) }>sram
	. = ALIGN(4);
	__bss_end = .;

	.mram : { *(.mram) } >mram
}
