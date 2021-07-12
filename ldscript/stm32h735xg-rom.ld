
flash_page_count = 8;

MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 1024K
	 ram : ORIGIN = 0x24000000, LENGTH = 320K
	 sysmem : ORIGIN = 0x20000000, LENGTH = 128K
	 backup : ORIGIN = 0x38800000, LENGTH = 4K
}

INCLUDE ldscripts/stm32h7xx.ld

PROVIDE ( _unique_id = 0x1FF0F420 );
