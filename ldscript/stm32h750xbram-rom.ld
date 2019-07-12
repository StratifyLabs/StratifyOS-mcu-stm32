flash_page_count = 1;


MEMORY {
	 ram : ORIGIN = 0x24000000, LENGTH = 512K
	 sysmem : ORIGIN = 0x20000000, LENGTH = 128K
	 ram1 : ORIGIN = 0x30000000, LENGTH = 288K
	 ram2 : ORIGIN = 0x38000000, LENGTH = 64K
}

SECTIONS {

	.sysmem : {
		_sys = . ;
		*(.sysmem)
		. = ALIGN(1024);
	} > sysmem
	_esys = .;

	. = 0;
	INCLUDE ldscripts/stm32-common-ram.ld
}

PROVIDE ( _end = . );
PROVIDE ( _top_of_stack = (ORIGIN(sysmem) + LENGTH(sysmem)) );
PROVIDE ( _estack = (ORIGIN(sysmem) + LENGTH(sysmem)) );
PROVIDE ( _flash_start = 0 );
PROVIDE ( _flash_pages = flash_page_count );
PROVIDE ( _flash_size = 0 );
PROVIDE ( _ram_pages = ( (LENGTH(ram) + LENGTH(ram1) + LENGTH(ram2)) / 1024 ) );
PROVIDE ( _sram_size = LENGTH(ram) );
PROVIDE ( _sram_start = ORIGIN(ram) );
PROVIDE ( _sram1_size = LENGTH(ram1) );
PROVIDE ( _sram1_start = ORIGIN(ram1) );
PROVIDE ( _sram2_size = LENGTH(ram2) );
PROVIDE ( _sram2_start = ORIGIN(ram2) );
PROVIDE ( _unique_id = 0x1FF0F420 );

