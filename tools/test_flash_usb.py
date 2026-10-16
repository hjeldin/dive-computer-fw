import struct
import zlib

import pytest

import flash_usb as f


def test_encode_simple_commands():
    assert f.encode_info() == bytes([0x01])
    assert f.encode_finish() == bytes([0x04])
    assert f.encode_reset() == bytes([0x05])


def test_encode_begin_layout():
    assert f.encode_begin(16, 0x12345678) == bytes([0x02, 0x10, 0, 0, 0, 0x78, 0x56, 0x34, 0x12])


def test_encode_data_layout():
    assert f.encode_data(8, b"\xaa\xbb\xcc") == bytes([0x03, 8, 0, 0, 0, 3, 0, 0xAA, 0xBB, 0xCC])


@pytest.mark.parametrize("chunk", [b"", bytes(1025)])
def test_encode_data_rejects_bad_sizes(chunk):
    with pytest.raises(ValueError):
        f.encode_data(0, chunk)


def test_decode_info():
    reply = bytes([0, 1]) + struct.pack("<II", 489472, 2048)
    assert f.decode_info(reply) == (1, 489472, 2048)


def test_check_reply_raises_with_status_name():
    with pytest.raises(f.UpdateError, match="CRC mismatch"):
        f.check_reply(bytes([0x06]), "FINISH")
    assert f.check_reply(bytes([0x00, 9]), "INFO") == bytes([0x00, 9])


def test_crc_matches_device_check_value():
    # Same value the Rust Crc32 test pins.
    assert zlib.crc32(b"123456789") == 0xCBF43926


class _FakeEndpoint:
    def __init__(self, address):
        self.bEndpointAddress = address

    def read(self, size, timeout):
        import usb.core

        raise usb.core.USBTimeoutError("timeout", -7, 110)


class _FakeConfig:
    bConfigurationValue = 1

    def __getitem__(self, key):
        assert key == (0, 0)
        return [_FakeEndpoint(0x01), _FakeEndpoint(0x81)]


class _FakeDevice:
    def __init__(self, configured):
        self.configured = configured
        self.set_configuration_calls = 0

    def get_active_configuration(self):
        import usb.core

        if not self.configured:
            raise usb.core.USBError("not configured")
        return _FakeConfig()

    def set_configuration(self):
        self.set_configuration_calls += 1
        self.configured = True


def test_bootloader_does_not_reconfigure_configured_device():
    # Re-sending SET_CONFIGURATION leaves the device's bulk OUT endpoint unprimed (embassy OTG),
    # so a second tool run after a failed upload would time out.
    dev = _FakeDevice(configured=True)
    bl = f.Bootloader(dev)
    assert dev.set_configuration_calls == 0
    assert (bl.ep_out.bEndpointAddress, bl.ep_in.bEndpointAddress) == (0x01, 0x81)


def test_bootloader_configures_fresh_device():
    dev = _FakeDevice(configured=False)
    f.Bootloader(dev)
    assert dev.set_configuration_calls == 1


class _QueueEndpoint(_FakeEndpoint):
    """IN endpoint returning queued replies, then timing out like libusb."""

    def __init__(self, address, replies):
        super().__init__(address)
        self.replies = list(replies)
        self.written = []

    def read(self, size, timeout):
        import usb.core

        if not self.replies:
            raise usb.core.USBTimeoutError("timeout", -7, 110)
        return self.replies.pop(0)

    def write(self, data, timeout):
        self.written.append(bytes(data))


def _bootloader_with(replies):
    bl = f.Bootloader(_FakeDevice(configured=True))
    bl.ep_in = _QueueEndpoint(0x81, replies)
    bl.ep_out = bl.ep_in
    return bl


def test_bootloader_drains_stale_replies_on_connect(monkeypatch):
    # A reply left over from an interrupted run (e.g. Ctrl-C during BEGIN) must not be read as
    # the answer to the next request.
    stale = _QueueEndpoint(0x81, [b"\x00", b"\x00"])
    monkeypatch.setattr(_FakeConfig, "__getitem__", lambda self, key: [_FakeEndpoint(0x01), stale])
    f.Bootloader(_FakeDevice(configured=True))
    assert stale.replies == []


def test_request_rejects_reply_of_wrong_length():
    bl = _bootloader_with([b"\x00"])
    with pytest.raises(f.UpdateError, match="INFO"):
        bl.request(f.encode_info(), "INFO", reply_len=10)


def test_main_reports_usb_errors_without_traceback(monkeypatch, capsys):
    import usb.core

    def broken_upload(*args):
        raise usb.core.USBError("No such device (it may have been disconnected)")

    monkeypatch.setattr(f, "load_image", lambda path: b"\x00" * 16)
    monkeypatch.setattr(f, "find_usb", lambda pid: object())
    monkeypatch.setattr(f, "Bootloader", lambda dev: None)
    monkeypatch.setattr(f, "upload", broken_upload)
    monkeypatch.setattr("sys.argv", ["flash_usb.py", "x.bin"])
    assert f.main() == 1
    assert "No such device" in capsys.readouterr().err
