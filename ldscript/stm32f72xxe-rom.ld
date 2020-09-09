flash_page_count = 8;

MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 512K
	 ram : ORIGIN = 0x20010000, LENGTH = 176K
	 sysmem : ORIGIN = 0x20000000, LENGTH = 32K
	 backup : ORIGIN = 0x40024000, LENGTH = 4K
}

INCLUDE ldscripts/stm32f7xx.ld

PROVIDE ( _unique_id = 0x1FF07A10 );
