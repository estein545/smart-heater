ENTRY(_reset);

MEMORY {
    FLASH(rx)   : ORIGIN = 0x08000000, LENGTH = 64K
    SRAM(rwx)   : ORIGIN = 0x20000000, LENGTH = 20K
}
_estack = ORIGIN(SRAM) + LENGTH(SRAM);

SECTIONS {
    .vectors  : { KEEP(*(.vectors)) }   > FLASH
    .text     : { *(.text*) }              > FLASH
    .rodata   : { *(.rodata*) }            > FLASH

    .data   : {
        _sdata = .;
        *(.first_data)
        *(.data SORT(.data.*))
        _edata = .;
    } > SRAM AT > FLASH
    _sidata = LOADADDR(.data);

    .bss    : {
        _sbss = .;
        *(.bss SORT(.bss.*) COMMON) 
        _ebss = .;
    } > SRAM

    . = ALIGN(8);
    _end = .;
}