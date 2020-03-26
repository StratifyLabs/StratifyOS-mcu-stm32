flash_page_count = 16;

MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 1536K
	 ram : ORIGIN = 0x20000000, LENGTH = 256K
	 sysmem : ORIGIN = ORIGIN(ram) + LENGTH(ram), LENGTH = 64K
}

INCLUDE ldscripts/stm32f4xx.ld
