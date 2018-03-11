flash_page_count = 6;

MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 256K
	 ram : ORIGIN = 0x20000000, LENGTH = 56K
	 sysmem : ORIGIN = ORIGIN(ram) + LENGTH(ram), LENGTH = 8K
}

INCLUDE ldscripts/stm32f4xx.ld

