/* Partition layout: docs/superpowers/specs/2026-10-16-usb-bootloader-design.md
   Must match bootloader/memory.x. The app links at ACTIVE. */
MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  BOOTLOADER        : ORIGIN = 0x08000000, LENGTH = 64K
  BOOTLOADER_STATE  : ORIGIN = 0x08010000, LENGTH = 8K
  FLASH             : ORIGIN = 0x08012000, LENGTH = 474K
  DFU               : ORIGIN = 0x08088800, LENGTH = 476K
  RAM               : ORIGIN = 0x20000000, LENGTH = 96K
  RAM2              : ORIGIN = 0x10000000, LENGTH = 32K
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* NOTE Do NOT modify `_stack_start` unless you know what you are doing */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(BOOTLOADER);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(BOOTLOADER);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(BOOTLOADER);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(BOOTLOADER);
