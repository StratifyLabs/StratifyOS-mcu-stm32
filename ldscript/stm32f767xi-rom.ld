flash_page_count = 12;


MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 2048K
	 ram : ORIGIN = 0x20020000, LENGTH = 368K
	 sysmem : ORIGIN = 0x20000000, LENGTH = 128K
}

INCLUDE ldscripts/stm32f7xx.ld

