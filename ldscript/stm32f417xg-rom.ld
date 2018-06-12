flash_page_count = 12;

MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 1024K
	 ram : ORIGIN = 0x20000000, LENGTH = 128K
	 sysmem : ORIGIN = 0x10000000, LENGTH = 64K
}

INCLUDE ldscripts/stm32f4xx.ld
