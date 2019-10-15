flash_page_count = 0;


MEMORY {
	 flash : ORIGIN = 0x24000000, LENGTH = 448K
	 ram : ORIGIN = ORIGIN(flash) + LENGTH(flash), LENGTH = 64K
	 sysmem : ORIGIN = 0x20000000, LENGTH = 64K
}

PROVIDE ( _flash_start = 0x08000000 );
INCLUDE ldscripts/stm32h7xx.ld


