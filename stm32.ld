MEMORY
{
  ram1 (rwx)        : ORIGIN = 0x20020000, LENGTH = 368K
  ram2 (rwx)        : ORIGIN = 0x2007C000, LENGTH = 16K
  stack(rwx)        : ORIGIN = 0x2007BFF8, LENGTH = 0K

  rom (rx)         : ORIGIN = 0x00200000, LENGTH = 1024K
}

SECTIONS
{
    .nvic_vector : 
    {
        *(vectors)    /* Vector table */
    } >rom

    . = ALIGN(8);

    .text : 
    {
        *(.text)      /* Program code */
        *(.text.*)
        *(.rodata)    /* Read only data */
        *(.rodata.*)
    } >rom

    . = ALIGN(8);

    _DATAI_BEGIN = LOADADDR(.data);

    .data : 
    {
        _DATA_BEGIN = .;
        *(.data)
        *(.data.*)        
        _DATA_END = .;
    } >ram1 AT>rom

    . = ALIGN(8);

    _DATA2I_BEGIN = LOADADDR(.data2);

    .data2 : 
    {
        _DATA2_BEGIN = .;
        *(.data2)
        *(.data2.*)        
        _DATA2_END = .;
    } >ram2 AT>rom

    . = ALIGN(8);


    .bss :
    {
        _BSS_BEGIN = .;
        *(.bss)
        *(COMMON)        
        _BSS_END = .;
    } >ram1 

    . = ALIGN(8);

    .heap :
    {
        _HEAP = .;
    } >ram1

    .stack :
    {
        _STACKTOP = .;
    } >stack


    . = ALIGN(8);

}  
