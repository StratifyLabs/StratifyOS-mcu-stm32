
flash_page_count = 8;

MEMORY {
	 flash : ORIGIN = 0x00200000, LENGTH = 1024K
	 ram : ORIGIN = 0x20010000, LENGTH = 240K
	 sysmem : ORIGIN = 0x20000000, LENGTH = 64K
}

INCLUDE ldscripts/stm32f7xx.ld

PROVIDE ( _unique_id = 0x1FF0F420 );
