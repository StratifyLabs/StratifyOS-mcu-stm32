flash_page_count = 0;


MEMORY {
	 flash : ORIGIN = 0x24000000, LENGTH = 512K
	 ram : ORIGIN = 0x38000000, LENGTH = 48K
	 sysmem : ORIGIN = 0x20000000, LENGTH = 64K
	 backup : ORIGIN = 0x38800000, LENGTH = 4K
	 dmamem : ORIGIN = 0x3800C000, LENGTH = 16K
}

PROVIDE ( _flash_start = 0x08000000 );

SECTIONS {

	.sysmem : {
		_sys = . ;
		*(.sysmem)
		. = ALIGN(1024);
	} > sysmem
	_esys = .;

	. = 0;
	.backup : {
		_backup = . ;
		*(.backup)
	} >backup
	_ebackup = .;

	. = 0;
	.dmamem : {
		_dmamem = . ;
		*(.dmamem)
		} >dmamem
	_edmamem = .;

	. = 0;
	.text : {
				 _text = .;
				 _tcim = .;
			 KEEP(*(.startup))
			 KEEP(*(.reset_vector))
			 KEEP(*(.isr_vector))
				 *(.text)
				 *(.init)
				 *(.fini)
				 *(.eh_frame)
				 _ctors = .;
				 KEEP(*(.init_array))
				 KEEP(*(.ctors))
				 _dtors = .;
				 KEEP(*(.dtors))
				 KEEP(*(.fini_array))
				 _dtors_end = .;
				 *(.jcr)
				 *(.rodata)
				 *(.rodata*)
				 *(.glue_7)
				 *(.glue_7t)

				 *(.text.*)
				 _priv_code = .;
				 *(.priv_code)
				 _priv_exec_code = .;
				 *(.priv_exec_code)
				 _epriv_code = .;
				 . = ALIGN(32);
	} >flash


	.ARM.extab :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} > flash
	__exidx_start = .;
	.ARM.exidx   : {
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} >flash

	. = ALIGN(4);
	__exidx_end = .;
	_etext = .;
	_etcim = .;
	.data : {
				 _data = .;
				 *(.data)
				 *(.data.*)
				 } >ram AT>flash
	_edata = .;
	.bss : {
			_bss = .;
				 *(.bss)
				 *(.bss.*)
				 } >ram
	. = ALIGN(4);
	_ebss = .;
}

PROVIDE ( _end = . );
PROVIDE ( _top_of_stack = (ORIGIN(sysmem) + LENGTH(sysmem)) );
PROVIDE ( _estack = (ORIGIN(sysmem) + LENGTH(sysmem)) );
PROVIDE ( _flash_pages = flash_page_count );
PROVIDE ( _flash_size = LENGTH(flash) );
PROVIDE ( _ram_pages = ( (LENGTH(ram) + LENGTH(ram1) + LENGTH(ram2)) / 1024 ) );
PROVIDE ( _sram_size = LENGTH(ram) );
PROVIDE ( _sram_start = ORIGIN(ram) );
PROVIDE ( _sram1_size = LENGTH(ram1) );
PROVIDE ( _sram1_start = ORIGIN(ram1) );
PROVIDE ( _sram2_size = LENGTH(ram2) );
PROVIDE ( _sram2_start = ORIGIN(ram2) );
PROVIDE ( _unique_id = 0x1FF0F420 );

