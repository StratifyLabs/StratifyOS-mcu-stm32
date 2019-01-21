flash_page_count = 16;


MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 2048K
	 ram : ORIGIN = 0x24000000, LENGTH = 512K
	 sysmem : ORIGIN = 0x20000000, LENGTH = 128K
	 ram1 : ORIGIN = 0x30000000, LENGTH = 288K
	 ram2 : ORIGIN = 0x38000000, LENGTH = 64K
}

INCLUDE ldscripts/stm32h7xx.ld

