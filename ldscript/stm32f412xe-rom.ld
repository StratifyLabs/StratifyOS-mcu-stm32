flash_page_count = 8;

MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 512K
	 ram : ORIGIN = 0x20000000, LENGTH = 240K
	 sysmem : ORIGIN = ORIGIN(ram) + LENGTH(ram), LENGTH = 16K
	 backup : ORIGIN = 0x40002850, LENGTH = 80
}

INCLUDE ldscripts/stm32f4xx.ld
