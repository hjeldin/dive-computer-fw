/* Partition layout: docs/superpowers/specs/2026-10-16-usb-bootloader-design.md
   Must match the app's memory.x. */
MEMORY
{
  FLASH             : ORIGIN = 0x08000000, LENGTH = 64K
  BOOTLOADER_STATE  : ORIGIN = 0x08010000, LENGTH = 8K
  ACTIVE            : ORIGIN = 0x08012000, LENGTH = 474K
  DFU               : ORIGIN = 0x08088800, LENGTH = 476K
  RAM               : ORIGIN = 0x20000000, LENGTH = 96K
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(FLASH);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(FLASH);

__bootloader_active_start = ORIGIN(ACTIVE) - ORIGIN(FLASH);
__bootloader_active_end = ORIGIN(ACTIVE) + LENGTH(ACTIVE) - ORIGIN(FLASH);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(FLASH);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(FLASH);
