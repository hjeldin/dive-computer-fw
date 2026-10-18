# TODO

Open work on the dive computer firmware, ordered by priority within each
section. Safety items come first: they matter in the water.

## 1. Safety

- [ ] **Alarms.** The buzzer only clicks on button presses. Add alarms for:
  - ascent rate too high (the tracker already computes `ascent_rate_m_min`;
    see the `// todo: check ascent speed` in `src/ms5837.rs`)
  - ceiling violation / missed deco stop
  - NDL running low
  - maximum depth
  - low battery during a dive
- [ ] **Persist tissue loading across reboots.** Tissues start clean at every
  boot, so a repetitive dive after a reset or a flat battery gets NDLs that
  are far too generous. Save the tissues and the time of the last dive to
  flash, and desaturate them over the surface interval at boot.
- [ ] **Surface interval and no-fly time.** Follows from the item above; not
  shown anywhere yet.
- [ ] **Watchdog.** There is no IWDG. A hung task underwater leaves a frozen
  screen with old data. Add an IWDG fed by a supervisor task only when the
  pressure sensor, dive tick and screen tasks have reported recently.
- [ ] **Safety stop.** 3 min at 5 m countdown.
- [ ] **Visible sensor failure.** The dive task marks stale pressure readings
  as invalid, but a sensor that stops answering raises no clear on-screen
  error or alarm.
- [ ] **Settings persistence.** The external QSPI flash (U5) ignores erase and
  program commands, so GF, SAC, backlight and screen-off settings fall back to
  defaults on every reboot. Move the settings slot log to the free last 2 KB
  page of internal flash (`0x080FF800`, right after the DFU partition; reserve
  it in both `memory.x` files). The same mechanism can hold the tissues.

## 2. Dive features

- [ ] **Gases.** Only air in practice. The deco library has a `Gas` struct,
  but there is no Nitrox setting, no ppO2 / MOD limit and no CNS / OTU
  tracking.
- [ ] **Logbook.** The SD card only gets a raw sensor CSV during dives. Add a
  dive log (number, date, max depth, duration, minimum temperature, profile)
  and a way to browse past dives on the device.
- [ ] **Download dives.** Export the logbook over the USB command interface,
  or over BLE (see the hardware section: BLE is unusable on the v3 board).
- [ ] **Compass calibration on the device.** Hard-iron calibration is a
  constant copied from the logs (`MAG_OFFSET` / `MAG_HALF_RANGE` in
  `src/tasks/ninedof.rs`). Add an on-device calibration and a bearing lock.
- [ ] **Surface pressure / altitude.** Check that the deco calculations use
  the surface pressure measured at boot, not a fixed 1.0 bar.

## 3. Reliability

- [ ] **Plausibility checks.** Reject pressure jumps and temperature values
  that point to a flooded or failing sensor.
- [ ] **Battery estimate.** The level is a linear 3.2–4.2 V scale with no load
  or temperature compensation. Add a "do not dive" warning below a threshold.
- [ ] **Firmware tests.** The deco library has unit tests; the firmware has
  none, and the `sim-dive` feature is not run automatically. A host test that
  runs simulated dive profiles through the tracker and checks the alarms
  would catch regressions.

## 4. Power

Current state (PPK2, 3500 mAh battery): about 34 mA bright, 26 mA dimmed,
about 1.1–1.5 mA with the screen off (STOP2). Good enough for now; possible
further steps:

- [ ] Measure again after the wake-up reductions (SD retry, battery monitor,
  screen and backlight fallbacks).
- [ ] Find the 0.44–0.72 mA floor between wake-ups: measure with the ST-Link
  disconnected, then with the LCD unplugged. Suspects: LCD module in sleep,
  the QSPI chip, the PB2 pull-up against the BlueNRG pull-down.
- [ ] Remaining 1 Hz wake-ups: dive tick and USB power poll.
- [ ] Optional: run from MSI 48 MHz in PLL mode with the LSE instead of
  HSE + PLL, so a wake from STOP does not wait about 2 ms for HSE.

## 5. Hardware (next board revision)

- [ ] **BlueNRG antenna.** The In1 GND and In2 +3V3 planes are solid under the
  whole module, including the antenna, in both the ordered v3 board and the
  current layout. Add an all-layer rule area (no pours, tracks or vias) over
  the antenna end plus 6 mm, and fix the keepouts in the
  `dive-watch:XCVR_BLUENRG-M0A` library footprint.
- [ ] **BlueNRG IRQ pull-down.** The datasheet asks for an external pull-down
  on `SPI1_BT_IRQ`; add 10–100 k to GND. Until then, use the internal
  pull-down on PB2.
- [ ] **BlueNRG CS on the v3 board.** Pad 10 (`SPI_CS`) is tied to GND on the
  ordered board (fixed in the current layout). Bodge: cut the trace to the
  GND via and wire pad 10 to PC4 (LQFP pin 24).
- [ ] **Current layout cleanup:** test point references (`CS`, `IRQ`, ...) do
  not match the schematic (`TPBTCS1`, ...); a dangling `SPI1_BT_CS` via at
  (68.5, 59.71); vias at the tips of the module pads; BT traces on F.Cu under
  the module; refill zones before generating gerbers.
- [ ] **QSPI flash U5.** Erase and program abort with no effect even after
  fitting C27. Check the U5 VCC/GND joints, reflow, or replace the chip.
- [ ] **Hall-effect or piezo buttons.** If buttons move to magnets, keep them
  at least 45–50 mm from the magnetometer (U4), or move U4 to the bottom
  right corner.

## 6. Code and repository

- [ ] Remove leftovers: `bmp280.rs`, `ens160.rs`, `air_quality_request.rs`,
  `usart_hello.rs`, `no_interaction.rs`, `ble_irq.rs`, the `LOW_POWER_MODE`
  paths and the commented-out BlueNRG code (about 140 compiler warnings).
- [x] Rewrite `README.md`: it is still the `cortex-m-quickstart` template
  with the USB sections appended.
- [ ] Commit the outstanding work in this repo, `dive-computer-gfx`,
  `dive-computer-deco` and the embassy fork.
- [ ] Document or upstream the embassy fork changes: ADC v3 `power_down`,
  restoring the L4 clocks after STOP, timer/PWM stop handling, and the EP0
  stall workaround (`docs/embassy-otg-ep0-stall-issue.md`).
