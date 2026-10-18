# Dive computer firmware

Firmware for a homemade wrist dive computer, written in Rust with
[embassy](https://embassy.dev) on an STM32L476RG.

> **Warning:** This is a hobby prototype. It is not tested or certified for real
> dives. Do not use it to plan or run a dive. See `TODO.md` for the missing
> safety features (alarms, tissue persistence across reboots, watchdog).

## Features

- Bühlmann ZH-L16C decompression with user-set gradient factors (air only),
  via the `dive-computer-deco` crate: NDL, ceiling, deco stops, TTS
- Automatic dive start and end from the pressure sensor
- Dive, tissues, settings and startup screens on a 320×240 ILI9341 LCD
  (rendered by `dive-computer-gfx`), with two buttons
- Tilt-compensated compass from the IMU and magnetometer
- Sensor log (time, pressure, temperature, acceleration) as CSV on the SD card
  during dives
- Low-power mode: sensors poll slowly at the surface, the screen turns off
  after inactivity and the MCU sleeps in STOP2 (about 1.1–1.5 mA)
- Battery voltage monitor and adjustable backlight
- USB bootloader with rollback, and optional defmt logs over USB (see below)

## Hardware

| Part | Use |
| --- | --- |
| STM32L476RG | MCU (Cortex-M4F, 1 MiB flash, 128 KiB RAM) |
| MS5837-30BA | Depth and water temperature |
| ISM330DHCX | Accelerometer and gyroscope (also wakes the device) |
| MMC5983MA | Magnetometer |
| ILI9341 | 320×240 SPI LCD with PWM backlight |
| SD card (SDMMC) | Dive sensor log |
| QSPI flash | Settings and timestamps (not working on the current board) |
| BlueNRG-M0A | BLE (not usable on the v3 board, disabled) |
| Buzzer, 2 buttons, battery ADC, USB-C | User interface and power |

The board design lives in the `dive-watch` repository.

## Repository layout

- `src/main.rs`: clock, peripheral and task setup
- `src/dive.rs`: feeds pressure into the deco tracker once a second
- `src/state.rs`: shared state between tasks and the screen
- `src/tasks/`: one embassy task per peripheral (buttons, buzzer, SD card,
  IMU, battery, USB, backlight)
- `src/ms5837.rs`, `src/ili9341.rs`, ...: sensor and display drivers
- `bootloader/`: USB bootloader and its update protocol crate
- `tools/`: host scripts to flash over USB and read USB logs
- `docs/`: design specs and implementation plans

## Building

The build expects these sibling checkouts next to this repository:

- `../embassy` (fork with STM32L4 low-power and USB fixes)
- `../dive-computer-deco` (decompression model)
- `../dive-computer-gfx` (UI scenes)
- `../dive-computer-proto` (message protocol)

The toolchain is pinned in `rust-toolchain.toml`. `nix develop` gives a shell
with the toolchain, `probe-rs` and the Python tools. Then:

    cargo build --release

Flash and run with a debug probe (logs over RTT):

    cargo run --release

### Cargo features

| Feature | Effect |
| --- | --- |
| `sim-dive` | Replace sensor pressure with a simulated deco dive at 10× speed |
| `raw-pressure` | Show raw pressure in mbar in place of TTS on the dive screen |
| `usb-log` | Send defmt logs over a second USB serial port instead of RTT |
| `no-confirm` | Never confirm a new image, so the bootloader rolls back on reset |
| `flash-selftest` | At boot, erase and program a spare QSPI sector and log the result |

## Flashing over USB-C

The board runs a small bootloader (`bootloader/`) that can receive app images over USB.
Design: `docs/superpowers/specs/2026-10-16-usb-bootloader-design.md`.

One-time setup with a debug probe:

    cd bootloader && cargo run --release

Then build and flash the app over USB:

    cargo build --release
    nix develop -c tools/flash_usb.py

The tool asks the running app to reboot into the bootloader, uploads the image, and waits
for the new app to come back. A new image that fails during startup is rolled back on the
next reset.

Recovery: if the app does not respond over USB, hold ENTER while resetting the board; the
bootloader stays in update mode (`lsusb` shows `c0de:b007`) and the tool works as usual.

Flashing the app with `cargo run --release` (probe-rs) still works and leaves the
bootloader in place.

## Logs over USB

Built with the `usb-log` feature, the app sends its defmt logs over a second USB serial
port instead of RTT. Read and decode them with:

    cargo build --release --features usb-log
    nix develop -c tools/usb_log.py

The tool reads the log endpoint through libusb rather than `/dev/ttyACM*`, so the
buffered boot logs survive (a freshly opened tty is in cooked mode and mangles them). It
needs `defmt-print` (`cargo install defmt-print`) and the ELF of the firmware on the
device, and reconnects after a reset or replug. Panics only reach RTT.
