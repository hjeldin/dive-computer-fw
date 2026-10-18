# USB-C firmware update via a bulk-transport bootloader

Date: 2026-10-16

## Goal

Flash new app firmware over the USB-C port, without a debug probe, with rollback if the new image
does not come up, and with a recovery path when the running app is broken.

## Why not the obvious options

- **STM32 ROM DFU bootloader:** enables OTG VBUS sensing on PA9. On mainboard v3, PA9 is
  `LCD_DC`, not VBUS, so the ROM never connects. Verified with a debugger: clearing
  `GCCFG.VBDEN` inside the ROM makes `0483:df11` enumerate immediately. Not fixable in firmware.
- **embassy-usb-dfu (standard USB DFU):** `embassy-usb-synopsys-otg` on the L476 stalls every
  SETUP that follows a control transfer with an OUT data stage for ~5 s (reproduced on upstream
  `main` `31dff5c`, see `docs/embassy-otg-ep0-stall-issue.md`). DFU sends every block as such a
  control transfer, so a download would take minutes.

Bulk endpoints are not affected, so the image travels over a vendor-class bulk interface.

## Decisions

| Topic | Decision |
|---|---|
| Who receives the image | The bootloader. The app only requests update mode and resets. |
| Recovery | Holding ENTER (PA4) at power-on forces update mode. |
| Image check | Host sends length and CRC32; bootloader verifies before marking for swap. No signing. |
| Confirmation | New app calls `mark_booted()` after all tasks are spawned and USB has enumerated, or 10 s have passed. |
| Crate location | `bootloader/` in this repo, own `Cargo.toml`, `memory.x`, `.cargo/config.toml`, using `../embassy` (path from the crate: `../../embassy`). |

## Flash layout

STM32L476RG: 1 MiB flash, 2 KiB pages, 8-byte write size.

```
BOOTLOADER        0x08000000   64K
BOOTLOADER_STATE  0x08010000    8K
ACTIVE (app)      0x08012000  474K
DFU               0x08088800  476K   (ACTIVE + 1 page, required by embassy-boot swap)
spare             0x080FF800    2K
```

The app links at `ACTIVE`. The bootloader `memory.x` and app `memory.x` must define the same
partition addresses. Both files carry a comment pointing at this spec.

## Bootloader

### Boot flow

1. After reset: enable GPIOA, configure PA4 as input with pull-up, read RTC backup register 31.
   Update mode is requested if BKP31 == `UPDATE_REQUESTED` (`0xB007_0001`, cleared on read) or PA4
   reads low on 5 samples taken 2 ms apart.
2. `embassy_stm32::init` with the USB clocks (update mode) or the reset defaults (boot).
3. Always run `BootLoader::prepare` first: it finishes a pending swap or reverts an unconfirmed
   image. An upload must never start on top of an unconfirmed swap, or embassy-boot would later
   "revert" it into ACTIVE and both images would be lost.
4. If update mode was requested: serve the update protocol (below).
5. If the first word of `ACTIVE` is not a stack pointer inside SRAM1
   (`0x2000_0000..=0x2001_8000`), e.g. erased flash: write `UPDATE_REQUESTED` to BKP31 and reset
   into update mode. Checked after `prepare` so that a first install (image in DFU, empty
   ACTIVE) boots after its swap instead of looping back into update mode.
6. Otherwise `BootLoader::load` at `ACTIVE`.

### Update mode

- Clocks as in the app: 8 MHz HSE, SYSCLK 80 MHz from PLL1 R, USB 48 MHz from PLLSAI1 Q.
- USB device `c0de:b007`, manufacturer "hjeldin", product "Dive Computer Bootloader".
  One vendor-class interface (class 0xFF) with one bulk OUT and one bulk IN endpoint, 64-byte
  packets. `vbus_detection = false`.
- No class or vendor control requests are used.
- Stays in update mode until `FINISH` succeeds or `RESET` is received. No timeout.

### Protocol

Host sends a request on bulk OUT, device answers with exactly one reply on bulk IN. The host
waits for each reply before the next request. Requests longer than 64 bytes are sent as several
packets; the device reads until it has the full request (length known from the header).
All integers little-endian.

Requests: `[cmd: u8][payload]`

| cmd | Name | Payload |
|---|---|---|
| 0x01 | `INFO` | none |
| 0x02 | `BEGIN` | `len: u32`, `crc32: u32` |
| 0x03 | `DATA` | `offset: u32`, `n: u16`, `n` bytes, `1 <= n <= 1024` |
| 0x04 | `FINISH` | none |
| 0x05 | `RESET` | none |

Replies: `[status: u8][payload]`, status `0x00` OK, otherwise an error code:

| status | Meaning |
|---|---|
| 0x01 | unknown command or malformed request |
| 0x02 | bad state (e.g. `DATA` before `BEGIN`, `FINISH` before all bytes arrived) |
| 0x03 | image too large for DFU partition |
| 0x04 | offset not equal to bytes received so far |
| 0x05 | flash erase/write error |
| 0x06 | CRC mismatch |

Reply payloads:

- `INFO` OK: `protocol_version: u8` (= 1), `dfu_size: u32`, `page_size: u32`.
- All other replies: none.

Behaviour:

- `BEGIN` erases the DFU pages needed for `len` (rounded up to a page), resets the write state.
- `DATA` appends bytes. Writes go through a buffer of the flash write size (8 bytes); a final
  partial block is padded with `0xFF` when `FINISH` flushes it.
- `FINISH` flushes, computes CRC32 (IEEE, as `zlib.crc32`) over `len` bytes of DFU, and on match
  replies OK, marks the image for swap (`FirmwareUpdater::mark_updated`) and resets after the
  reply has been sent. On mismatch replies `0x06`, marks nothing, and returns to the idle state;
  the host must start again with `BEGIN`.
- `RESET` replies OK and resets without marking anything.

## App changes

- `memory.x`: `FLASH` = `ACTIVE` (`0x08012000`, 474K); add `BOOTLOADER`, `BOOTLOADER_STATE`,
  `DFU` regions and the `__bootloader_state_*` / `__bootloader_dfu_*` symbols that
  `FirmwareUpdaterConfig::from_linkerfile_blocking` needs.
- `src/rom_dfu.rs` is replaced by `src/bootloader.rs`:
  - `reboot_to_bootloader()`: write `UPDATE_REQUESTED` to BKP31, `SCB::sys_reset()`.
  - `mark_booted()`: open `FLASH` blocking, `BlockingFirmwareState::mark_booted()`.
  - The ROM jump, memory remap and the early check in `async_main` are removed.
- USB task: on `DFU` command reply `OK DFU`, wait 50 ms, `reboot_to_bootloader()`.
- `async_main`: after spawning all tasks, wait until USB is configured or 10 s have passed, then
  `bootloader::mark_booted()`.
- Removed: the 2 s USB-power boot window (replaced by ENTER recovery), the reset-cause string in
  the USB serial number (back to a fixed serial).
- `Cargo.toml`: `embassy-usb-dfu` dependency removed; `embassy-boot-stm32` kept.

## Host tool: `tools/flash_usb.py`

Python 3, `pyusb`, `pyserial` (underscore in the name so tests can import it). Replaces `tools/flash-usb.sh`.

1. Input: ELF (default `target/thumbv7em-none-eabihf/release/stm-blink`) or `.bin`. ELF is
   converted with `arm-none-eabi-objcopy -O binary`.
2. If `c0de:b007` is present, use it. Else find the app's CDC port by VID:PID `c0de:cafe`, open it
   (allowing for the ~5 s open), send `\nDFU\n`, wait up to 15 s for `c0de:b007`.
3. `INFO`, check protocol version and size; `BEGIN`; `DATA` in 1024-byte chunks with progress;
   `FINISH`.
4. Wait up to 120 s for `c0de:cafe` to enumerate again (the swap of the 474K slot takes about a
   minute) and report.
   Any error reply aborts with the status name.

`flake.nix`: add `python3` with `pyusb` and `pyserial`; `dfu-util` removed.

## Development workflow

- Flash bootloader once: `cd bootloader && cargo run --release` (probe-rs).
- Flash app as before with `cargo run --release`; it now links at `0x08012000` and the bootloader
  boots it. A probe-flashed app was not swapped in, so its `mark_booted` call only rewrites the
  "booted" state and is harmless.
- Flash app over USB: `tools/flash_usb.py`.

## Error handling

- Transfer corruption or a truncated upload: CRC mismatch, nothing marked, old app still boots.
- Power loss during upload: DFU partition partially written, nothing marked, old app boots.
- Power loss during swap: embassy-boot resumes the swap on next boot (progress kept in
  `BOOTLOADER_STATE`).
- New app crashes or hangs before confirming: next reset reverts to the previous image.
- New app confirms but its USB is broken: hold ENTER at power-on.
- Erased or garbage `ACTIVE`: bootloader stays in update mode.

## Testing

All on hardware (no host-side test framework exists in this repo).

1. Bootloader with empty `ACTIVE` enumerates as `c0de:b007`.
2. `flash_usb.py` uploads the app; app enumerates as `c0de:cafe` afterwards.
3. From the running app: `flash_usb.py` with a second build (different serial or version string)
   swaps to it.
4. Corrupt-CRC upload (tool flag `--bad-crc` for testing) gets `CRC mismatch`, old app still runs.
5. Image that never calls `mark_booted` (feature flag in the app) rolls back after reset.
6. Holding ENTER at power-on enters update mode while a valid app is installed.
7. Interrupt an upload (unplug) mid-transfer; old app still boots.
