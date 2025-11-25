use bitfield::bitfield;
use embedded_hal_async::{delay::DelayNs, i2c::I2c};
use defmt::info;

pub struct MMC5983<I2C, D>
where
    I2C: I2c,
    D: DelayNs,
{
    i2c: I2C,
    address: u8,
    delay: D,
    // This contains the prd and cmm bits which is a write-only register
    control_reg0b: ControlReg0b,
}

bitfield! {
    struct XYZOut2(u8);
    get_zout, _: 3, 2;
    get_yout, _: 5, 4;
    get_xout, _: 7, 6;
}

bitfield! {
    struct ControlReg08(u8);
    get_meas_m_done, set_meas_m_done: 0;
    get_meas_t_done, set_meas_t_done: 1;
    get_otp_read_done, set_otp_read_done: 4;
}

bitfield! {
    struct ControlReg09(u8);
    /// Take magnetic field measurement, set ‘1’ will initiate measurement. This bit will be
    /// automatically reset to 0 at the end of each measurement.
    get_tm_m, set_tm_m: 0;
    /// Take Temperature measurement, set ‘1’ will initiate measurement. This bit will be
    /// automatically reset to 0 at the end of each measurement. This bit and TM_M cannot be high
    /// at the same time.
    get_tm_t, set_tm_t: 1;
    /// Writing “1” will enable the interrupt for completed measurements. Once a measurement is
    /// finished, either magnetic field or temperature, an interrupt will be sent to the host.
    get_int_meas_done_en, set_int_meas_done_en: 2;
    /// Writing “1” will cause the chip to do the Set operation, which will allow large set current
    /// to flow through the sensor coils for 500ns. This bit is self-cleared at the end of Set
    /// operation.
    get_set, set_set: 3;
    /// Writing “1” will cause the chip to do the Reset operation, which will allow large reset
    /// current to flow through the sensor coils for 500ns. This bit is self-cleared at the end of
    /// Reset operation.
    get_reset, set_reset: 4;
    /// Writing “1” will enable the feature of automatic set/reset.
    get_auto_sr_en, set_auto_sr_en: 5;
    /// Writing “1” will let the device to read the OTP data again. This bit will be automatically
    /// reset to 0 after the shadow registers for OTP are refreshed
    get_otp_read, set_otp_read: 6;
}

// TODO
enum OutputResolution {
    Hz100 = 0b00,
    Hz200 = 0b01,
    Hz400 = 0b10,
    Hz800 = 0b11,
}

bitfield! {
    pub struct ControlReg0a(u8);
    /// "Output Resolution"
    /// 00: 8ms (100Hz)
    /// 01: 4ms (200Hz)
    /// 10: 2ms (400Hz)
    /// 11: 1ms (800Hz)
    get_bw, set_bw: 1, 0;
    /// 1: Disable channel
    /// 0: Enable channel
    get_x_inhibit, set_x_inhibit: 2;
    /// 11: Disable channels
    /// 00: Enable channels
    get_yz_inhibit, set_yz_inhibit: 4, 3;
    // Used in `reset_chip`
    get_sw_rst, set_sw_rst: 7;
}

// TODO
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum ContinuousMeasurementFreq {
    Off = 0b000,
    Hz1 = 0b001,
    Hz10 = 0b010,
    Hz20 = 0b011,
    Hz50 = 0b100,
    Hz100 = 0b101,
    Hz200 = 0b110,
    Hz1000 = 0b111,
}

// TODO
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum PeriodicSetInterval {
    Per1 = 0b000,
    Per25 = 0b001,
    Per75 = 0b010,
    Per100 = 0b011,
    Per250 = 0b100,
    Per500 = 0b101,
    Per1000 = 0b110,
    Per2000 = 0b111,
    Off,
}

bitfield! {
    pub struct ControlReg0b(u8);
    /// These bits determine how often the chip will take measurements in Continuous
    /// Measurement Mode. The frequency is based on the assumption that BW[1:0] = 00.
    ///
    /// 000: Continuous Measurement Mode is off.
    /// 001: 1 Hz
    /// 010: 10 Hz
    /// 011: 20 Hz
    /// 100: 50 Hz
    /// 101: 100 Hz
    /// 110: (BW=01) 200 Hz
    /// 111: (BW=11) 1000 Hz
    get_cm_freq, set_cm_freq: 2, 0;
    /// 1: Enable Continuous Measurement Mode
    /// 0: Disable Continuous Measurement Mode
    /// CM_FREQ must be set to a non-zero value for this bit to have any effect.
    get_cmm_en, set_cmm_en: 3;

    /// These bits determine how often the chip will do a set operation. The device will perform a
    /// SET automatically per the setting in below table.
    ///
    /// Prd_set [2:0] Times of measurement
    /// 000: 1
    /// 001: 25
    /// 010: 75
    /// 011: 100
    /// 100: 250
    /// 101: 500
    /// 110: 1000
    /// 111: 2000
    get_prd_set, set_prd_set: 6, 4;
    /// Writing “1” will enable the feature of periodic set. This feature needs to work with both
    /// Auto_SR_en and Cmm_en bits set to 1.
    get_en_prd_set, set_en_prd_set: 7;
}

bitfield! {
    pub struct ControlReg0c(u8);
    /// Writing “1” will apply an extra current flowing from the positive end to the negative end
    /// of an internal coil and result in an extra magnetic field. This feature can be used to
    /// check whether the sensor has been saturated
    get_st_enp, set_st_enp: 1;
    /// Writing “1” will apply an extra current flowing from the negative end to the positive end
    /// of an internal coil and result in an extra magnetic field. This feature can be used to
    /// check whether the sensor has been saturated
    get_st_enm, set_st_enm: 2;
    /// Writing a 1 into this location will put the device into 3-wire SPI mode.
    get_spi_3w_en, set_spi_3w_en: 6;
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MeasurementRaw<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

const MAX_LOOPS: usize = 100;

impl<I2C, D> MMC5983<I2C, D>
where
    I2C: I2c,
    D: DelayNs,
{
    pub fn new(i2c: I2C, delay_driver: D, address: u8) -> Self {
        MMC5983 {
            i2c,
            address,
            delay: delay_driver,
            control_reg0b: ControlReg0b(0),
        }
    }

    pub async fn read_register(&mut self, register: u8) -> Result<u8, I2C::Error> {
        let mut data = [0u8];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .await?;
        Ok(data[0])
    }

    pub async fn set_register(&mut self, register: u8, data: u8) -> Result<(), I2C::Error> {
        if register == 0x0b {
            self.control_reg0b = ControlReg0b(data);
        }

        self.i2c.write(self.address, &[register, data]).await?;
        Ok(())
    }

    pub async fn reset(&mut self) -> Result<(), I2C::Error> {
        let mut reg = ControlReg0a(0);
        reg.set_sw_rst(true);
        self.set_register(0x0a, reg.0).await?;
        self.delay.delay_ms(10).await;
        Ok(())
    }

    pub async fn set_cmm_mode(
        &mut self,
        freq: ContinuousMeasurementFreq,
        period: PeriodicSetInterval,
    ) -> Result<(), I2C::Error> {
        let mut reg = ControlReg0b(0);
        match freq {
            ContinuousMeasurementFreq::Off => {
                reg.set_cmm_en(false);
            }
            _ => {
                reg.set_cm_freq(freq as u8);
                reg.set_cmm_en(true);
            }
        }
        match period {
            PeriodicSetInterval::Off => {
                reg.set_en_prd_set(false);
            }
            _ => {
                reg.set_prd_set(period as u8);
                reg.set_en_prd_set(true);
            }
        }

        self.set_register(0x0b, reg.0).await?;
        Ok(())
    }

    pub async fn init(&mut self) -> Result<MeasurementRaw<u32>, I2C::Error> {
        // Read the device ID
        let device_id = self.read_register(0x2F).await?;
        if device_id != 0b00110000 {
            panic!(
                "Invalid device ID: 0x{:02X}, are you sure this is an mmc5983ma",
                device_id
            );
        }

        self.reset().await?;

        let mut reg = ControlReg0a(0);
        reg.set_bw(0b00);
        reg.set_x_inhibit(false);
        reg.set_yz_inhibit(0b00);
        self.set_register(0x0a, reg.0).await?;

        let mut reg = ControlReg0c(0);
        reg.set_st_enp(false);
        reg.set_st_enm(false);
        reg.set_spi_3w_en(false);
        self.set_register(0x0c, reg.0).await?;

        self.set_cmm_mode(ContinuousMeasurementFreq::Off, PeriodicSetInterval::Off)
            .await?;

        // 50hz = 20ms
        self.delay.delay_ms(21).await;

        self.do_measurement_raw().await
    }

    // Read: 16 bit
    pub async unsafe fn read_raw_measurement_16(
        &mut self,
    ) -> Result<MeasurementRaw<u16>, I2C::Error> {
        let mut data = [0u8; 6];
        self.i2c
            .write_read(self.address, &[0x00], &mut data)
            .await?;
        let x = (data[0] as u16) << 16 | (data[1] as u16) << 8;
        let y = (data[2] as u16) << 16 | (data[3] as u16) << 8;
        let z = (data[4] as u16) << 16 | (data[5] as u16) << 8;
        return Ok(MeasurementRaw { x, y, z });
    }

    // Read: 18 bit
    pub async unsafe fn read_raw_measurement_18(
        &mut self,
    ) -> Result<MeasurementRaw<u32>, I2C::Error> {
        let mut data = [0u8; 7];
        self.i2c
            .write_read(self.address, &[0x00], &mut data)
            .await?;
        let xyzout2 = XYZOut2(data[6]);
        // Equivalent to C++ code:
        // x = (data[0] << 8) | data[1]; x = (x << 2) | (data[6] >> 6);
        // Which expands to: (data[0] << 10) | (data[1] << 2) | (data[6] >> 6)
        let x = ((data[0] as u32) << 10) | ((data[1] as u32) << 2) | (xyzout2.get_xout() as u32);
        let y = ((data[2] as u32) << 10) | ((data[3] as u32) << 2) | (xyzout2.get_yout() as u32);
        let z = ((data[4] as u32) << 10) | ((data[5] as u32) << 2) | (xyzout2.get_zout() as u32);
        // info!("x: {}", x);
        // info!("y: {}", y);
        // info!("z: {}", z);
        return Ok(MeasurementRaw { x, y, z });
    }

    pub async fn do_measurement_raw(&mut self) -> Result<MeasurementRaw<u32>, I2C::Error> {
        // Request to read mag
        let mut reg = ControlReg09(self.read_register(0x09).await?);
        reg.set_tm_m(true);
        self.set_register(0x09, reg.0).await?;

        for _ in 0..MAX_LOOPS {
            let r = ControlReg08(self.read_register(0x08).await?);

            if r.get_meas_t_done() {
                // If we get a mag measurement ready signal, write a 0 to register 0x08 to clear it
                // We only care about the temperature measurement signal
                let mut r = ControlReg08(0);
                r.set_meas_t_done(true);
                self.set_register(0x08, r.0).await?;

                // Request to read mag
                let mut reg = ControlReg09(0);
                reg.set_tm_m(true);
                self.set_register(0x09, reg.0).await?;
            }

            if r.get_meas_m_done() {
                // SAFETY: we just checked r.get_meas_m_done()
                unsafe {
                    return self.read_raw_measurement_18().await;
                }
            }

            self.delay.delay_ms(15).await;
        }

        panic!();
    }

    pub async fn get_temp_raw(&mut self) -> Result<Option<u8>, I2C::Error> {
        // turn off continuous measurement mode so we can read temp
        let mut r = ControlReg0b(self.read_register(0x0B).await?);
        let cmm_en = self.control_reg0b.get_cmm_en();
        if cmm_en {
            r.set_cmm_en(false);
            self.set_register(0x0b, r.0).await?;
        }

        // Request to read temp
        let mut reg = ControlReg09(0);
        reg.set_tm_t(true);
        self.set_register(0x09, reg.0).await?;

        for _ in 0..100 {
            let r = ControlReg08(self.read_register(0x08).await?);

            if r.get_meas_m_done() {
                // If we get a mag measurement ready signal, write a 0 to register 0x08 to clear it
                // We only care about the temperature measurement signal
                let mut r = ControlReg08(0);
                r.set_meas_m_done(true);
                self.set_register(0x08, r.0).await?;

                // Request to read temp
                let mut reg = ControlReg09(0);
                reg.set_tm_t(true);
                self.set_register(0x09, reg.0).await?;
            }

            if r.get_meas_t_done() {
                let t = self.read_register(0x07).await?;

                // turn back on continuous measurement mode if it was enabled before
                if cmm_en {
                    let mut r = ControlReg0b(self.read_register(0x0B).await?);
                    r.set_cmm_en(cmm_en);
                    self.set_register(0x0b, r.0).await?;
                }

                return Ok(Some(t));
            }

            self.delay.delay_ms(15).await;
        }

        Ok(None)
    }

    pub async fn get_temp_c(&mut self) -> Result<Option<f32>, I2C::Error> {
        let temp_raw = self.get_temp_raw().await?;
        // range is -75c ti 125c, ~0.8c per bit. 0x00 = -75c
        let range = (125.0 - -75.0) / 256.0;
        Ok(temp_raw.map(|temp_raw| (temp_raw as f32) * range + -75.0))
    }
}
