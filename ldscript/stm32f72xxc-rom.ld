flash_page_count = 6;

MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 256K
	 ram : ORIGIN = 0x20000000, LENGTH = 167K
	 sysmem : ORIGIN = ORIGIN(ram) + LENGTH(ram), LENGTH = 16K
	 backup : ORIGIN = 0x40024000, LENGTH = 4K
}

INCLUDE ldscripts/stm32f7xx.ld

PROVIDE ( _unique_id = 0x1FF07A10 );

