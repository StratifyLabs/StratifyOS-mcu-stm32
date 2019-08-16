flash_page_count = 0;


MEMORY {
	 flash : ORIGIN = 0x24000000, LENGTH = 384K
	 ram : ORIGIN = ORIGIN(flash) + LENGTH(flash), LENGTH = 128K
	 sysmem : ORIGIN = 0x20000000, LENGTH = 64K
}

INCLUDE ldscripts/stm32h7xx.ld


