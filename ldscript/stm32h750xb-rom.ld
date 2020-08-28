flash_page_count = 1;


MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 128K
	 ram : ORIGIN = 0x20000000, LENGTH = 64K
	 sysmem : ORIGIN = 0x20010000, LENGTH = 64K
	 ram1 : ORIGIN = 0x30000000, LENGTH = 288K
	 ram2 : ORIGIN = 0x38000000, LENGTH = 64K
	 backup : ORIGIN = 0x38800000, LENGTH = 4K
}

INCLUDE ldscripts/stm32h7xx.ld

