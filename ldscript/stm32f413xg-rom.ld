flash_page_count = 12;

MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 1024K
	 ram : ORIGIN = 0x20000000, LENGTH = 256K
	 sysmem : ORIGIN = 0x20040000, LENGTH = 64K
	 backup : ORIGIN = 0x40002850, LENGTH = 80
}

INCLUDE ldscripts/stm32f4xx.ld
