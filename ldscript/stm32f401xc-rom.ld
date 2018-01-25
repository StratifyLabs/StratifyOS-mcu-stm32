

MEMORY {
   flash : ORIGIN = 0x08000000, LENGTH = 256K
   ram : ORIGIN = 0x20000000, LENGTH = 48K
   sysmem : ORIGIN = 0x2000C000, LENGTH = 16K
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
PROVIDE ( _top_of_stack = (0x2000C000 + 16384) );
PROVIDE ( _estack = (0x2000C000 + 16384) );
PROVIDE ( _flash_start = 0x08000000 );
PROVIDE ( _flash_pages = 8 );
PROVIDE ( _flash_size = 262144 );
PROVIDE ( _ram_pages = 48 );
PROVIDE ( _sram_size = 49152 );
PROVIDE ( _sram_start = 0x20000000 );
