flash_page_count = 128;

MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 256K
	 ram : ORIGIN = 0x20000000, LENGTH = 48K
	 sysmem : ORIGIN =  0x10000000, LENGTH = 16K
}

INCLUDE ldscripts/stm32l4xx.ld
