
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
PROVIDE ( _top_of_stack = (ORIGIN(sysmem) + LENGTH(sysmem)) );
PROVIDE ( _estack = (ORIGIN(sysmem) + LENGTH(sysmem)) );
PROVIDE ( _flash_start = ORIGIN(flash) );
PROVIDE ( _flash_pages = flash_page_count );
PROVIDE ( _flash_size = LENGTH(flash) );
PROVIDE ( _ram_pages = ( LENGTH(ram) / 1024 ) );
PROVIDE ( _sram_size = LENGTH(ram) );
PROVIDE ( _sram_start = ORIGIN(ram) );
PROVIDE ( _unique_id = 0x1FFF7590 );
