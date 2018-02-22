

MEMORY {
	 flash : ORIGIN = 0x08000000, LENGTH = 512K
	 ram : ORIGIN = 0x20000000, LENGTH = 120K
	 sysmem : ORIGIN = 0x2001E000, LENGTH = 8K
}

SECTIONS {

	.sysmem : {
		_sys = . ;
		*(.sysmem)
		. = ALIGN(1024);
	} >sysmem
	_esys = .;

  . = 0;
  INCLUDE ldscripts/stm32-common.ld
}

PROVIDE ( _end = . );
PROVIDE ( _top_of_stack = (0x2001E000 + 8K) );
PROVIDE ( _estack = (0x2001E000 + 8K) );
PROVIDE ( _flash_start = 0x08000000 );
PROVIDE ( _flash_pages = 8 );
PROVIDE ( _flash_size = 512K );
PROVIDE ( _ram_pages = 120 );
PROVIDE ( _sram_size = 120K );
PROVIDE ( _sram_start = 0x20000000 );
