//! USB firmware update protocol shared by the bootloader and the app.
//!
//! See `docs/superpowers/specs/2026-10-16-usb-bootloader-design.md`.
#![cfg_attr(not(test), no_std)]

/// USB vendor ID shared by the app and the bootloader.
pub const USB_VID: u16 = 0xc0de;
/// USB product ID of the app (CDC ACM).
pub const APP_PID: u16 = 0xcafe;
/// USB product ID of the bootloader in update mode.
pub const BOOTLOADER_PID: u16 = 0xb007;
/// RTC backup register the app writes to request update mode.
pub const UPDATE_REQUEST_BKP: usize = 31;
/// Value of [`UPDATE_REQUEST_BKP`] that requests update mode.
pub const UPDATE_REQUESTED: u32 = 0xB007_0001;

pub const PROTOCOL_VERSION: u8 = 1;
/// Largest payload of one DATA request.
pub const MAX_DATA: usize = 1024;
/// Largest request: DATA header (7 bytes) plus payload.
pub const MAX_REQUEST: usize = 7 + MAX_DATA;
/// Flash write granularity of the STM32L4.
pub const WRITE_SIZE: usize = 8;

pub mod cmd {
    pub const INFO: u8 = 0x01;
    pub const BEGIN: u8 = 0x02;
    pub const DATA: u8 = 0x03;
    pub const FINISH: u8 = 0x04;
    pub const RESET: u8 = 0x05;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Status {
    Ok = 0x00,
    BadRequest = 0x01,
    BadState = 0x02,
    TooLarge = 0x03,
    BadOffset = 0x04,
    FlashError = 0x05,
    CrcMismatch = 0x06,
}

#[derive(Debug, PartialEq, Eq)]
pub enum Request<'a> {
    Info,
    Begin { len: u32, crc32: u32 },
    Data { offset: u32, data: &'a [u8] },
    Finish,
    Reset,
}

/// Total length of the request starting with `prefix`, once enough header bytes are known.
pub fn expected_len(prefix: &[u8]) -> Result<Option<usize>, Status> {
    let Some(&command) = prefix.first() else {
        return Ok(None);
    };
    match command {
        cmd::INFO | cmd::FINISH | cmd::RESET => Ok(Some(1)),
        cmd::BEGIN => Ok(Some(9)),
        cmd::DATA => {
            if prefix.len() < 7 {
                return Ok(None);
            }
            let n = u16::from_le_bytes([prefix[5], prefix[6]]) as usize;
            if n == 0 || n > MAX_DATA { Err(Status::BadRequest) } else { Ok(Some(7 + n)) }
        }
        _ => Err(Status::BadRequest),
    }
}

/// Parses one complete request.
pub fn parse(buf: &[u8]) -> Result<Request<'_>, Status> {
    if expected_len(buf)? != Some(buf.len()) {
        return Err(Status::BadRequest);
    }
    let u32_at = |i: usize| u32::from_le_bytes([buf[i], buf[i + 1], buf[i + 2], buf[i + 3]]);
    match buf[0] {
        cmd::INFO => Ok(Request::Info),
        cmd::BEGIN => Ok(Request::Begin { len: u32_at(1), crc32: u32_at(5) }),
        cmd::DATA => Ok(Request::Data { offset: u32_at(1), data: &buf[7..] }),
        cmd::FINISH => Ok(Request::Finish),
        cmd::RESET => Ok(Request::Reset),
        _ => Err(Status::BadRequest),
    }
}

/// Collects USB packets until one complete request is buffered.
pub struct Assembler {
    buf: [u8; MAX_REQUEST],
    len: usize,
}

impl Assembler {
    pub const fn new() -> Self {
        Self { buf: [0; MAX_REQUEST], len: 0 }
    }

    /// Appends a packet. Returns `Ok(true)` once a complete request is buffered.
    /// On error the buffer is cleared.
    pub fn push(&mut self, packet: &[u8]) -> Result<bool, Status> {
        if self.len + packet.len() > MAX_REQUEST {
            self.len = 0;
            return Err(Status::BadRequest);
        }
        self.buf[self.len..self.len + packet.len()].copy_from_slice(packet);
        self.len += packet.len();
        match expected_len(&self.buf[..self.len]) {
            Ok(Some(n)) if self.len == n => Ok(true),
            Ok(Some(n)) if self.len > n => {
                self.len = 0;
                Err(Status::BadRequest)
            }
            Ok(_) => Ok(false),
            Err(e) => {
                self.len = 0;
                Err(e)
            }
        }
    }

    pub fn request(&self) -> Result<Request<'_>, Status> {
        parse(&self.buf[..self.len])
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }
}

/// CRC-32/ISO-HDLC (the zlib/IEEE CRC).
pub struct Crc32(u32);

impl Crc32 {
    pub const fn new() -> Self {
        Self(0xFFFF_FFFF)
    }

    pub fn update(&mut self, data: &[u8]) {
        for &byte in data {
            let mut c = (self.0 ^ byte as u32) & 0xFF;
            for _ in 0..8 {
                c = if c & 1 != 0 { (c >> 1) ^ 0xEDB8_8320 } else { c >> 1 };
            }
            self.0 = (self.0 >> 8) ^ c;
        }
    }

    pub fn finish(&self) -> u32 {
        !self.0
    }
}

/// Where an upload is written. Offsets are relative to the start of the DFU partition.
pub trait UpdateTarget {
    fn dfu_size(&self) -> u32;
    fn page_size(&self) -> u32;
    /// Erases `[from, to)`; both are multiples of `page_size()`.
    fn erase(&mut self, from: u32, to: u32) -> Result<(), ()>;
    /// Writes whole [`WRITE_SIZE`] blocks at a [`WRITE_SIZE`]-aligned offset.
    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), ()>;
    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), ()>;
    /// Marks the DFU partition to be swapped in on the next boot.
    fn mark_updated(&mut self) -> Result<(), ()>;
}

/// One reply: status byte plus optional payload.
pub struct Reply {
    buf: [u8; 10],
    len: usize,
}

impl Reply {
    pub fn status(status: Status) -> Self {
        let mut buf = [0; 10];
        buf[0] = status as u8;
        Self { buf, len: 1 }
    }

    pub fn info(dfu_size: u32, page_size: u32) -> Self {
        let mut buf = [0; 10];
        buf[0] = Status::Ok as u8;
        buf[1] = PROTOCOL_VERSION;
        buf[2..6].copy_from_slice(&dfu_size.to_le_bytes());
        buf[6..10].copy_from_slice(&page_size.to_le_bytes());
        Self { buf, len: 10 }
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

/// What the device does after sending a reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum After {
    Continue,
    Reset,
}

#[derive(Clone, Copy)]
enum Phase {
    Idle,
    Receiving { len: u32, crc32: u32, received: u32 },
}

/// Upload state machine.
pub struct Updater {
    phase: Phase,
    block: [u8; WRITE_SIZE],
    block_len: usize,
    written: u32,
}

impl Updater {
    pub const fn new() -> Self {
        Self { phase: Phase::Idle, block: [0xFF; WRITE_SIZE], block_len: 0, written: 0 }
    }

    pub fn handle<T: UpdateTarget>(&mut self, target: &mut T, request: Request<'_>) -> (Reply, After) {
        match request {
            Request::Info => (Reply::info(target.dfu_size(), target.page_size()), After::Continue),
            Request::Begin { len, crc32 } => (Reply::status(self.begin(target, len, crc32)), After::Continue),
            Request::Data { offset, data } => (Reply::status(self.data(target, offset, data)), After::Continue),
            Request::Finish => match self.finish(target) {
                Status::Ok => (Reply::status(Status::Ok), After::Reset),
                status => (Reply::status(status), After::Continue),
            },
            Request::Reset => {
                self.reset();
                (Reply::status(Status::Ok), After::Reset)
            }
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn begin<T: UpdateTarget>(&mut self, target: &mut T, len: u32, crc32: u32) -> Status {
        self.reset();
        if len == 0 {
            return Status::BadRequest;
        }
        if len > target.dfu_size() {
            return Status::TooLarge;
        }
        let page = target.page_size();
        if target.erase(0, len.div_ceil(page) * page).is_err() {
            return Status::FlashError;
        }
        self.phase = Phase::Receiving { len, crc32, received: 0 };
        Status::Ok
    }

    fn data<T: UpdateTarget>(&mut self, target: &mut T, offset: u32, data: &[u8]) -> Status {
        let Phase::Receiving { len, crc32, received } = self.phase else {
            return Status::BadState;
        };
        if offset != received {
            return Status::BadOffset;
        }
        if received as usize + data.len() > len as usize {
            return Status::BadRequest;
        }
        for &byte in data {
            self.block[self.block_len] = byte;
            self.block_len += 1;
            if self.block_len == WRITE_SIZE && self.flush_block(target).is_err() {
                self.reset();
                return Status::FlashError;
            }
        }
        self.phase = Phase::Receiving { len, crc32, received: received + data.len() as u32 };
        Status::Ok
    }

    fn flush_block<T: UpdateTarget>(&mut self, target: &mut T) -> Result<(), ()> {
        self.block[self.block_len..].fill(0xFF);
        target.write(self.written, &self.block)?;
        self.written += WRITE_SIZE as u32;
        self.block_len = 0;
        Ok(())
    }

    fn finish<T: UpdateTarget>(&mut self, target: &mut T) -> Status {
        let Phase::Receiving { len, crc32, received } = self.phase else {
            return Status::BadState;
        };
        if received != len {
            return Status::BadState;
        }
        if self.block_len > 0 && self.flush_block(target).is_err() {
            self.reset();
            return Status::FlashError;
        }
        self.reset();

        let mut crc = Crc32::new();
        let mut buf = [0u8; 256];
        let mut offset = 0;
        while offset < len {
            let n = (len - offset).min(buf.len() as u32) as usize;
            if target.read(offset, &mut buf[..n]).is_err() {
                return Status::FlashError;
            }
            crc.update(&buf[..n]);
            offset += n as u32;
        }
        if crc.finish() != crc32 {
            return Status::CrcMismatch;
        }
        if target.mark_updated().is_err() {
            return Status::FlashError;
        }
        Status::Ok
    }
}

/// Turns USB packets into replies: reassembles requests and runs the [`Updater`].
pub struct Server {
    assembler: Assembler,
    updater: Updater,
    session: u32,
}

impl Server {
    pub const fn new() -> Self {
        Self { assembler: Assembler::new(), updater: Updater::new(), session: 0 }
    }

    /// Handles one packet. `session` must change whenever the host resets or reconfigures the
    /// device; a partial request or upload from an older session is then dropped. Returns the
    /// reply to send, if the packet completed a request or was invalid.
    pub fn on_packet<T: UpdateTarget>(&mut self, target: &mut T, session: u32, packet: &[u8]) -> Option<(Reply, After)> {
        if session != self.session {
            self.session = session;
            self.assembler.clear();
            self.updater = Updater::new();
        }
        match self.assembler.push(packet) {
            Ok(false) => None,
            Ok(true) => {
                let result = match self.assembler.request() {
                    Ok(request) => self.updater.handle(target, request),
                    Err(status) => (Reply::status(status), After::Continue),
                };
                self.assembler.clear();
                Some(result)
            }
            Err(status) => Some((Reply::status(status), After::Continue)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PAGE: u32 = 1024;
    const SIZE: u32 = 4096;

    struct MockTarget {
        mem: Vec<u8>,
        erased: Vec<(u32, u32)>,
        marked: bool,
        fail_write: bool,
    }

    impl MockTarget {
        fn new() -> Self {
            Self { mem: vec![0u8; SIZE as usize], erased: Vec::new(), marked: false, fail_write: false }
        }
    }

    impl UpdateTarget for MockTarget {
        fn dfu_size(&self) -> u32 {
            SIZE
        }
        fn page_size(&self) -> u32 {
            PAGE
        }
        fn erase(&mut self, from: u32, to: u32) -> Result<(), ()> {
            self.erased.push((from, to));
            self.mem[from as usize..to as usize].fill(0xFF);
            Ok(())
        }
        fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), ()> {
            assert_eq!(offset as usize % WRITE_SIZE, 0, "unaligned write");
            assert_eq!(data.len() % WRITE_SIZE, 0, "partial block write");
            if self.fail_write {
                return Err(());
            }
            self.mem[offset as usize..offset as usize + data.len()].copy_from_slice(data);
            Ok(())
        }
        fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), ()> {
            buf.copy_from_slice(&self.mem[offset as usize..offset as usize + buf.len()]);
            Ok(())
        }
        fn mark_updated(&mut self) -> Result<(), ()> {
            self.marked = true;
            Ok(())
        }
    }

    fn crc(data: &[u8]) -> u32 {
        let mut c = Crc32::new();
        c.update(data);
        c.finish()
    }

    fn status(reply: &(Reply, After)) -> u8 {
        reply.0.as_bytes()[0]
    }

    #[test]
    fn crc32_matches_ieee_check_value() {
        assert_eq!(crc(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn crc32_is_incremental() {
        let mut c = Crc32::new();
        c.update(b"1234");
        c.update(b"56789");
        assert_eq!(c.finish(), 0xCBF4_3926);
    }

    #[test]
    fn expected_len_per_command() {
        assert_eq!(expected_len(&[]), Ok(None));
        assert_eq!(expected_len(&[cmd::INFO]), Ok(Some(1)));
        assert_eq!(expected_len(&[cmd::FINISH]), Ok(Some(1)));
        assert_eq!(expected_len(&[cmd::RESET]), Ok(Some(1)));
        assert_eq!(expected_len(&[cmd::BEGIN]), Ok(Some(9)));
        assert_eq!(expected_len(&[cmd::DATA, 0, 0, 0, 0, 0]), Ok(None));
        assert_eq!(expected_len(&[cmd::DATA, 0, 0, 0, 0, 0x10, 0]), Ok(Some(7 + 16)));
        assert_eq!(expected_len(&[0x7f]), Err(Status::BadRequest));
    }

    #[test]
    fn data_length_must_be_1_to_1024() {
        assert_eq!(expected_len(&[cmd::DATA, 0, 0, 0, 0, 0, 0]), Err(Status::BadRequest));
        assert_eq!(expected_len(&[cmd::DATA, 0, 0, 0, 0, 0x01, 0x04]), Err(Status::BadRequest)); // 1025
        assert_eq!(expected_len(&[cmd::DATA, 0, 0, 0, 0, 0x00, 0x04]), Ok(Some(7 + 1024)));
    }

    #[test]
    fn parse_begin_and_data() {
        let begin = [cmd::BEGIN, 0x10, 0, 0, 0, 0x78, 0x56, 0x34, 0x12];
        assert_eq!(parse(&begin), Ok(Request::Begin { len: 16, crc32: 0x1234_5678 }));
        let data = [cmd::DATA, 8, 0, 0, 0, 3, 0, 0xaa, 0xbb, 0xcc];
        assert_eq!(parse(&data), Ok(Request::Data { offset: 8, data: &[0xaa, 0xbb, 0xcc] }));
        assert_eq!(parse(&[cmd::BEGIN, 1, 2]), Err(Status::BadRequest));
    }

    #[test]
    fn assembler_reassembles_max_data_request_from_64_byte_packets() {
        let mut req = vec![cmd::DATA, 0, 0, 0, 0, 0x00, 0x04];
        req.extend((0..1024u32).map(|i| i as u8));
        let mut asm = Assembler::new();
        let packets: Vec<&[u8]> = req.chunks(64).collect();
        assert_eq!(packets.len(), 17);
        for p in &packets[..16] {
            assert_eq!(asm.push(p), Ok(false));
        }
        assert_eq!(asm.push(packets[16]), Ok(true));
        match asm.request() {
            Ok(Request::Data { offset: 0, data }) => assert_eq!(data, &req[7..]),
            other => panic!("unexpected {other:?}"),
        }
        asm.clear();
        assert_eq!(asm.push(&[cmd::INFO]), Ok(true));
        assert_eq!(asm.request(), Ok(Request::Info));
    }

    #[test]
    fn assembler_rejects_unknown_command_and_trailing_bytes() {
        let mut asm = Assembler::new();
        assert_eq!(asm.push(&[0x42]), Err(Status::BadRequest));
        assert_eq!(asm.push(&[cmd::INFO, cmd::INFO]), Err(Status::BadRequest));
        assert_eq!(asm.push(&[cmd::INFO]), Ok(true), "assembler resets after an error");
    }

    #[test]
    fn info_reports_version_and_sizes() {
        let mut t = MockTarget::new();
        let mut u = Updater::new();
        let (reply, after) = u.handle(&mut t, Request::Info);
        assert_eq!(after, After::Continue);
        let mut expected = vec![0, PROTOCOL_VERSION];
        expected.extend(SIZE.to_le_bytes());
        expected.extend(PAGE.to_le_bytes());
        assert_eq!(reply.as_bytes(), &expected[..]);
    }

    #[test]
    fn upload_with_unaligned_length_pads_and_marks() {
        let image: Vec<u8> = (1..=13).collect();
        let mut t = MockTarget::new();
        let mut u = Updater::new();
        let r = u.handle(&mut t, Request::Begin { len: 13, crc32: crc(&image) });
        assert_eq!(status(&r), Status::Ok as u8);
        assert_eq!(t.erased, vec![(0, PAGE)]);
        let r = u.handle(&mut t, Request::Data { offset: 0, data: &image[..5] });
        assert_eq!(status(&r), Status::Ok as u8);
        let r = u.handle(&mut t, Request::Data { offset: 5, data: &image[5..] });
        assert_eq!(status(&r), Status::Ok as u8);
        let r = u.handle(&mut t, Request::Finish);
        assert_eq!((status(&r), r.1), (Status::Ok as u8, After::Reset));
        assert!(t.marked);
        assert_eq!(&t.mem[..13], &image[..]);
        assert_eq!(&t.mem[13..16], &[0xFF; 3]);
    }

    #[test]
    fn crc_mismatch_marks_nothing_and_returns_to_idle() {
        let image = [7u8; 16];
        let mut t = MockTarget::new();
        let mut u = Updater::new();
        u.handle(&mut t, Request::Begin { len: 16, crc32: crc(&image) ^ 1 });
        u.handle(&mut t, Request::Data { offset: 0, data: &image });
        let r = u.handle(&mut t, Request::Finish);
        assert_eq!((status(&r), r.1), (Status::CrcMismatch as u8, After::Continue));
        assert!(!t.marked);
        let r = u.handle(&mut t, Request::Data { offset: 16, data: &image });
        assert_eq!(status(&r), Status::BadState as u8);
    }

    #[test]
    fn begin_validates_length_and_erases_whole_pages() {
        let mut t = MockTarget::new();
        let mut u = Updater::new();
        assert_eq!(status(&u.handle(&mut t, Request::Begin { len: 0, crc32: 0 })), Status::BadRequest as u8);
        assert_eq!(status(&u.handle(&mut t, Request::Begin { len: SIZE + 1, crc32: 0 })), Status::TooLarge as u8);
        assert_eq!(status(&u.handle(&mut t, Request::Begin { len: PAGE + 1, crc32: 0 })), Status::Ok as u8);
        assert_eq!(t.erased, vec![(0, 2 * PAGE)]);
    }

    #[test]
    fn data_errors() {
        let mut t = MockTarget::new();
        let mut u = Updater::new();
        let r = u.handle(&mut t, Request::Data { offset: 0, data: &[1] });
        assert_eq!(status(&r), Status::BadState as u8, "data before begin");
        u.handle(&mut t, Request::Begin { len: 4, crc32: 0 });
        let r = u.handle(&mut t, Request::Data { offset: 1, data: &[1] });
        assert_eq!(status(&r), Status::BadOffset as u8);
        let r = u.handle(&mut t, Request::Data { offset: 0, data: &[1, 2, 3, 4, 5] });
        assert_eq!(status(&r), Status::BadRequest as u8, "more data than announced");
        let r = u.handle(&mut t, Request::Finish);
        assert_eq!(status(&r), Status::BadState as u8, "finish before all bytes");
    }

    #[test]
    fn begin_again_restarts_upload() {
        let image = [9u8; 8];
        let mut t = MockTarget::new();
        let mut u = Updater::new();
        u.handle(&mut t, Request::Begin { len: 8, crc32: crc(&image) });
        u.handle(&mut t, Request::Data { offset: 0, data: &image[..3] });
        u.handle(&mut t, Request::Begin { len: 8, crc32: crc(&image) });
        let r = u.handle(&mut t, Request::Data { offset: 0, data: &image });
        assert_eq!(status(&r), Status::Ok as u8);
        let r = u.handle(&mut t, Request::Finish);
        assert_eq!(status(&r), Status::Ok as u8);
        assert_eq!(&t.mem[..8], &image);
    }

    #[test]
    fn flash_write_error_reports_and_resets() {
        let mut t = MockTarget::new();
        t.fail_write = true;
        let mut u = Updater::new();
        u.handle(&mut t, Request::Begin { len: 8, crc32: 0 });
        let r = u.handle(&mut t, Request::Data { offset: 0, data: &[0; 8] });
        assert_eq!(status(&r), Status::FlashError as u8);
        let r = u.handle(&mut t, Request::Finish);
        assert_eq!(status(&r), Status::BadState as u8);
    }

    #[test]
    fn new_session_discards_partial_request() {
        // Unplug between the packets of a DATA request: the next session's INFO must still be answered.
        let mut t = MockTarget::new();
        let mut s = Server::new();
        assert!(s.on_packet(&mut t, 1, &[cmd::DATA, 0, 0, 0, 0, 0x00, 0x04]).is_none());
        let (reply, after) = s.on_packet(&mut t, 2, &[cmd::INFO]).expect("INFO answered");
        assert_eq!(reply.as_bytes().len(), 10);
        assert_eq!(after, After::Continue);
    }

    #[test]
    fn new_session_resets_upload() {
        let mut t = MockTarget::new();
        let mut s = Server::new();
        s.on_packet(&mut t, 1, &[cmd::BEGIN, 8, 0, 0, 0, 0, 0, 0, 0]);
        let (reply, _) = s.on_packet(&mut t, 2, &[cmd::DATA, 0, 0, 0, 0, 1, 0, 0xaa]).unwrap();
        assert_eq!(reply.as_bytes(), &[Status::BadState as u8]);
    }

    #[test]
    fn same_session_keeps_assembling() {
        let mut t = MockTarget::new();
        let mut s = Server::new();
        assert!(s.on_packet(&mut t, 0, &[cmd::BEGIN, 8, 0, 0]).is_none());
        let (reply, _) = s.on_packet(&mut t, 0, &[0, 0, 0, 0, 0]).unwrap();
        assert_eq!(reply.as_bytes(), &[Status::Ok as u8]);
    }

    #[test]
    fn reset_request_resets() {
        let mut t = MockTarget::new();
        let mut u = Updater::new();
        let r = u.handle(&mut t, Request::Reset);
        assert_eq!((status(&r), r.1), (Status::Ok as u8, After::Reset));
        assert!(!t.marked);
    }
}
