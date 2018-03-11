flash_page_count = 8;

MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 512K
	 ram : ORIGIN = 0x20010000, LENGTH = 176K
	 sysmem : ORIGIN = 0x20000000, LENGTH = 64K
}

INCLUDE ldscripts/stm32f4xx.ld

