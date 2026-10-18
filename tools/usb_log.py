#!/usr/bin/env python3
"""Show the dive computer's defmt logs from its USB log port.

The app must be built with the `usb-log` feature. Decoding needs defmt-print
(`cargo install defmt-print`) and the ELF of the firmware on the device.
Run it in the dev shell for pyusb: `nix develop -c tools/usb_log.py`.

The log port is read through libusb, not /dev/ttyACM*: the tty starts in
cooked mode, which mangles the first bytes (the buffered boot logs) before it
can be switched to raw, and opening it stalls 5 s on the EP0 bug (see
docs/embassy-otg-ep0-stall-issue.md). While this runs, the log tty is gone.
The device is picked up again after a reset or replug.
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from flash_usb import APP_PID, DEFAULT_ELF, LOG_INTERFACE, USB_VID  # noqa: E402

# The log port's data interface, after its communication interface LOG_INTERFACE
LOG_DATA_INTERFACE = LOG_INTERFACE + 1
READ_TIMEOUT_MS = 1000


def find_defmt_print() -> str | None:
    """defmt-print on PATH, or where `cargo install` puts it."""
    found = shutil.which("defmt-print")
    if found:
        return found
    cargo_bin = os.path.expanduser("~/.cargo/bin/defmt-print")
    return cargo_bin if os.access(cargo_bin, os.X_OK) else None


def wait_log_device():
    import usb.core

    announced = False
    while True:
        dev = usb.core.find(idVendor=USB_VID, idProduct=APP_PID)
        if dev is not None and has_log_port(dev):
            return dev
        if not announced:
            print("Waiting for the log port (is the app built with --features usb-log?)", file=sys.stderr)
            announced = True
        time.sleep(0.5)


def has_log_port(dev) -> bool:
    try:
        return dev.get_active_configuration()[(LOG_DATA_INTERFACE, 0)] is not None
    except Exception:
        return False


def forward(dev, sink) -> None:
    """Copy bytes from the log port's bulk IN endpoint to `sink` until the device goes away."""
    import usb.core
    import usb.util

    # cdc_acm holds both interfaces of the port
    for interface in (LOG_INTERFACE, LOG_DATA_INTERFACE):
        if dev.is_kernel_driver_active(interface):
            dev.detach_kernel_driver(interface)
    usb.util.claim_interface(dev, LOG_DATA_INTERFACE)
    intf = dev.get_active_configuration()[(LOG_DATA_INTERFACE, 0)]
    ep_in = usb.util.find_descriptor(
        intf,
        custom_match=lambda ep: usb.util.endpoint_direction(ep.bEndpointAddress) == usb.util.ENDPOINT_IN,
    )
    try:
        while True:
            try:
                data = bytes(ep_in.read(4096, timeout=READ_TIMEOUT_MS))
            except usb.core.USBTimeoutError:
                continue
            sink.write(data)
            sink.flush()
    except usb.core.USBError:
        return
    finally:
        usb.util.dispose_resources(dev)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--elf", default=DEFAULT_ELF, help=f"firmware ELF (default: {DEFAULT_ELF})")
    parser.add_argument("--raw", action="store_true", help="write undecoded frames to stdout")
    args = parser.parse_args()

    if args.raw:
        decoder = None
        sink = sys.stdout.buffer
    else:
        if not os.path.exists(args.elf):
            print(f"No ELF at {args.elf}", file=sys.stderr)
            return 1
        defmt_print = find_defmt_print()
        if defmt_print is None:
            print("defmt-print not found: cargo install defmt-print", file=sys.stderr)
            return 1
        decoder = subprocess.Popen([defmt_print, "-e", args.elf, "stdin"], stdin=subprocess.PIPE)
        sink = decoder.stdin

    try:
        import usb.core  # noqa: F401
    except ImportError:
        print("pyusb missing: run in the dev shell, nix develop -c tools/usb_log.py", file=sys.stderr)
        return 1

    try:
        while True:
            dev = wait_log_device()
            print(f"Reading logs from bus {dev.bus} device {dev.address}", file=sys.stderr)
            forward(dev, sink)
            print("Log port gone", file=sys.stderr)
    except (KeyboardInterrupt, BrokenPipeError):
        return 0
    finally:
        if decoder is not None:
            decoder.stdin.close()
            decoder.wait()


if __name__ == "__main__":
    sys.exit(main())
