
EXTERN ( core_vector_table )

MEMORY {
   flash : ORIGIN = 0, LENGTH = 512K
	 ram : ORIGIN = 0x2007C000, LENGTH = 16K
	 sysmem : ORIGIN = 0x20080000, LENGTH = 16K
}

SECTIONS {

   .sysmem : {
		_sys = . ;
		*(.sysmem)
		. = ALIGN(1024);
	} >sysmem
	_esys = .;

  . = 0;
  INCLUDE ldscripts/lpc-common.ld
}

PROVIDE ( _end = . );
PROVIDE ( _iap = 0x1FFF1FF1 );
PROVIDE ( _top_of_stack = (0x20084000 - 32) );
PROVIDE ( _flash_pages = 30 );
PROVIDE ( _flash_size = 524288 );
PROVIDE ( _ram_pages = 60 );
PROVIDE ( _sram_size = 32768 );
PROVIDE ( _ahb_sram_size = 28672 );
