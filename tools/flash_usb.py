#!/usr/bin/env python3
"""Flash the dive computer app over USB through the bulk-transport bootloader.

Protocol: docs/superpowers/specs/2026-10-16-usb-bootloader-design.md
"""
import struct

USB_VID = 0xC0DE
APP_PID = 0xCAFE
BOOTLOADER_PID = 0xB007

CMD_INFO = 0x01
CMD_BEGIN = 0x02
CMD_DATA = 0x03
CMD_FINISH = 0x04
CMD_RESET = 0x05

MAX_DATA = 1024
PROTOCOL_VERSION = 1

STATUS_NAMES = {
    0x00: "OK",
    0x01: "bad request",
    0x02: "bad state",
    0x03: "image too large",
    0x04: "bad offset",
    0x05: "flash error",
    0x06: "CRC mismatch",
}


class UpdateError(Exception):
    pass


def encode_info() -> bytes:
    return bytes([CMD_INFO])


def encode_begin(length: int, crc32: int) -> bytes:
    return struct.pack("<BII", CMD_BEGIN, length, crc32)


def encode_data(offset: int, chunk: bytes) -> bytes:
    if not 1 <= len(chunk) <= MAX_DATA:
        raise ValueError(f"DATA chunk must be 1..{MAX_DATA} bytes, got {len(chunk)}")
    return struct.pack("<BIH", CMD_DATA, offset, len(chunk)) + chunk


def encode_finish() -> bytes:
    return bytes([CMD_FINISH])


def encode_reset() -> bytes:
    return bytes([CMD_RESET])


def check_reply(reply: bytes, what: str) -> bytes:
    if not reply:
        raise UpdateError(f"{what}: empty reply")
    if reply[0] != 0:
        name = STATUS_NAMES.get(reply[0], f"status 0x{reply[0]:02x}")
        raise UpdateError(f"{what} failed: {name}")
    return reply


def decode_info(reply: bytes) -> tuple[int, int, int]:
    _, version, dfu_size, page_size = struct.unpack("<BBII", reply[:10])
    return version, dfu_size, page_size


import argparse
import os
import subprocess
import sys
import tempfile
import termios
import time
import tty
import zlib

DEFAULT_ELF = "target/thumbv7em-none-eabihf/release/stm-blink"
# The bootloader swaps all 237 ACTIVE pages with DFU; measured at about a minute.
APP_RETURN_TIMEOUT_S = 120


def find_usb(pid: int):
    import usb.core

    return usb.core.find(idVendor=USB_VID, idProduct=pid)


def wait_usb(pid: int, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        dev = find_usb(pid)
        if dev is not None:
            return dev
        time.sleep(0.2)
    return None


def find_app_tty() -> str | None:
    for name in sorted(os.listdir("/sys/class/tty")):
        if not name.startswith("ttyACM"):
            continue
        dev = os.path.realpath(f"/sys/class/tty/{name}/device/..")
        try:
            vid = open(f"{dev}/idVendor").read().strip()
            pid = open(f"{dev}/idProduct").read().strip()
        except OSError:
            continue
        if int(vid, 16) == USB_VID and int(pid, 16) == APP_PID:
            return f"/dev/{name}"
    return None


def request_update_mode(port: str) -> None:
    # Opening/closing the CDC port takes ~5 s each (embassy OTG EP0 stall), so no tight timeouts.
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    try:
        tty.setraw(fd)
        termios.tcflush(fd, termios.TCIOFLUSH)
        os.write(fd, b"\nDFU\n")
    finally:
        os.close(fd)


class Bootloader:
    def __init__(self, dev):
        import usb.util

        import usb.core

        self.dev = dev
        # Only configure a fresh device: a repeated SET_CONFIGURATION leaves the bootloader's
        # bulk OUT endpoint unprimed (embassy OTG driver), so requests would time out.
        try:
            config = dev.get_active_configuration()
        except usb.core.USBError:
            config = None
        if config is None:
            dev.set_configuration()
            config = dev.get_active_configuration()
        intf = config[(0, 0)]
        self.ep_out = usb.util.find_descriptor(
            intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
        )
        self.ep_in = usb.util.find_descriptor(
            intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
        )
        # Drop replies left over from an interrupted run, so each reply matches its request.
        while True:
            try:
                self.ep_in.read(64, 50)
            except usb.core.USBTimeoutError:
                break

    def request(self, payload: bytes, what: str, reply_len: int = 1, timeout_ms: int = 5000) -> bytes:
        self.ep_out.write(payload, timeout_ms)
        reply = check_reply(bytes(self.ep_in.read(64, timeout_ms)), what)
        if len(reply) != reply_len:
            raise UpdateError(f"{what}: expected a {reply_len}-byte reply, got {len(reply)} bytes")
        return reply


def load_image(path: str) -> bytes:
    if path.endswith(".bin"):
        return open(path, "rb").read()
    with tempfile.NamedTemporaryFile(suffix=".bin") as out:
        subprocess.run(["arm-none-eabi-objcopy", "-O", "binary", path, out.name], check=True)
        return open(out.name, "rb").read()


def upload(bl: Bootloader, image: bytes, bad_crc: bool) -> None:
    version, dfu_size, _page = decode_info(bl.request(encode_info(), "INFO", reply_len=10))
    if version != PROTOCOL_VERSION:
        raise UpdateError(f"bootloader speaks protocol {version}, tool speaks {PROTOCOL_VERSION}")
    if len(image) > dfu_size:
        raise UpdateError(f"image is {len(image)} bytes, DFU partition holds {dfu_size}")
    crc = zlib.crc32(image) ^ (1 if bad_crc else 0)
    # Erasing up to 476 KiB of flash takes a few seconds.
    bl.request(encode_begin(len(image), crc), "BEGIN", timeout_ms=30000)
    for offset in range(0, len(image), MAX_DATA):
        bl.request(encode_data(offset, image[offset : offset + MAX_DATA]), "DATA")
        done = min(offset + MAX_DATA, len(image))
        print(f"\r  {done * 100 // len(image):3d}% ({done}/{len(image)} bytes)", end="", flush=True)
    print()
    bl.request(encode_finish(), "FINISH", timeout_ms=30000)


def main() -> int:
    import usb.core

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("firmware", nargs="?", default=DEFAULT_ELF, help="ELF or .bin (default: release app)")
    parser.add_argument("--bad-crc", action="store_true", help="send a wrong CRC (tests the bootloader check)")
    args = parser.parse_args()

    image = load_image(args.firmware)
    print(f"Image: {args.firmware} ({len(image)} bytes)")

    dev = find_usb(BOOTLOADER_PID)
    if dev is None:
        port = find_app_tty()
        if port is None:
            print("error: neither the app nor the bootloader is connected", file=sys.stderr)
            return 1
        print(f"Requesting update mode on {port}")
        request_update_mode(port)
        # If the running image was never confirmed, the bootloader first reverts it (~1 min).
        dev = wait_usb(BOOTLOADER_PID, APP_RETURN_TIMEOUT_S)
        if dev is None:
            print("error: bootloader did not appear", file=sys.stderr)
            return 1

    try:
        upload(Bootloader(dev), image, args.bad_crc)
    except (UpdateError, usb.core.USBError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    print("Upload OK, waiting for the new app (the swap takes about a minute)")
    if wait_usb(APP_PID, APP_RETURN_TIMEOUT_S) is None:
        print("error: app did not come back", file=sys.stderr)
        return 1
    print("Done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
