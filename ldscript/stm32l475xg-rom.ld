flash_page_count = 512;

MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 1024K
	 ram : ORIGIN = 0x20000000, LENGTH = 96K
	 sysmem : ORIGIN = 0x10000000, LENGTH = 32K
}

INCLUDE ldscripts/stm32l4xx.ld
