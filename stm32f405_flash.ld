/* 独立应用直接从0x08000000启动；1MiB Flash /128KiB普通SRAM。
 * 64KiB CCM未使用，USB缓冲区均位于普通SRAM。 */
ENTRY(Reset_Handler)
_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size = 0x1000;
_Min_Stack_Size = 0x2000;
MEMORY {
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
}
SECTIONS {
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } > FLASH
  .text : { . = ALIGN(4); *(.text .text.*) *(.rodata .rodata.*) KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); _etext = .; } > FLASH
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH
  .preinit_array (READONLY) : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } > FLASH
  .init_array (READONLY) : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } > FLASH
  .fini_array (READONLY) : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } > FLASH
  _sidata = LOADADDR(.data);
  .data : { . = ALIGN(4); _sdata = .; *(.data .data.*) . = ALIGN(4); _edata = .; } > RAM AT> FLASH
  .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; __bss_start__ = .; *(.bss .bss.*) *(COMMON) . = ALIGN(4); _ebss = .; __bss_end__ = .; } > RAM
  .noinit (NOLOAD) : { . = ALIGN(4); *(.noinit) . = ALIGN(4); } > RAM
  ._user_heap_stack (NOLOAD) : { . = ALIGN(8); PROVIDE(end = .); PROVIDE(_end = .); . += _Min_Heap_Size; _heap_limit = .; . += _Min_Stack_Size; . = ALIGN(8); } > RAM
  /DISCARD/ : { *(.note.GNU-stack) }
  .ARM.attributes 0 : { *(.ARM.attributes) }
}
