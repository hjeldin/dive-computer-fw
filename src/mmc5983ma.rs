use bitfield::bitfield;
use embedded_hal_async::{delay::DelayNs, i2c::I2c};
use crate::i2cdriver::I2CDriver;

pub struct MMC5983<'a, D>
where
    D: DelayNs,
{
    driver: I2CDriver<'a>,
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
    /// Take magnetic field measurement, set '1' will initiate measurement. This bit will be
    /// automatically reset to 0 at the end of each measurement.
    get_tm_m, set_tm_m: 0;
    /// Take Temperature measurement, set '1' will initiate measurement. This bit will be
    /// automatically reset to 0 at the end of each measurement. This bit and TM_M cannot be high
    /// at the same time.
    get_tm_t, set_tm_t: 1;
    /// Writing "1" will enable the interrupt for completed measurements. Once a measurement is
    /// finished, either magnetic field or temperature, an interrupt will be sent to the host.
    get_int_meas_done_en, set_int_meas_done_en: 2;
    /// Writing "1" will cause the chip to do the Set operation, which will allow large set current
    /// to flow through the sensor coils for 500ns. This bit is self-cleared at the end of Set
    /// operation.
    get_set, set_set: 3;
    /// Writing "1" will cause the chip to do the Reset operation, which will allow large reset
    /// current to flow through the sensor coils for 500ns. This bit is self-cleared at the end of
    /// Reset operation.
    get_reset, set_reset: 4;
    /// Writing "1" will enable the feature of automatic set/reset.
    get_auto_sr_en, set_auto_sr_en: 5;
    /// Writing "1" will let the device to read the OTP data again. This bit will be automatically
    /// reset to 0 after the shadow registers for OTP are refreshed
    get_otp_read, set_otp_read: 6;
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
    /// Writing "1" will enable the feature of periodic set. This feature needs to work with both
    /// Auto_SR_en and Cmm_en bits set to 1.
    get_en_prd_set, set_en_prd_set: 7;
}

bitfield! {
    pub struct ControlReg0c(u8);
    /// Writing "1" will apply an extra current flowing from the positive end to the negative end
    /// of an internal coil and result in an extra magnetic field. This feature can be used to
    /// check whether the sensor has been saturated
    get_st_enp, set_st_enp: 1;
    /// Writing "1" will apply an extra current flowing from the negative end to the positive end
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
const DEVICE_ID_REG: u8 = 0x2F;
const EXPECTED_DEVICE_ID: u8 = 0b00110000;
const XOUT_REG: u8 = 0x00;
const TEMP_REG: u8 = 0x07;
const STATUS_REG: u8 = 0x08;
const CONTROL_REG_09: u8 = 0x09;
const CONTROL_REG_0A: u8 = 0x0A;
const CONTROL_REG_0B: u8 = 0x0B;
const CONTROL_REG_0C: u8 = 0x0C;

impl<'a, D> MMC5983<'a, D>
where
    D: DelayNs,
{
    pub fn new(driver: I2CDriver<'a>, delay_driver: D) -> Self {
        MMC5983 {
            driver,
            delay: delay_driver,
            control_reg0b: ControlReg0b(0),
        }
    }

    async fn read_register(&mut self, register: u8) -> Result<u8, embassy_stm32::i2c::Error> {
        let mut data = [0u8];
        self.driver
            .write_read_bytes(&[register], &mut data)
            .await
            .map_err(|_| embassy_stm32::i2c::Error::Timeout)?;
        Ok(data[0])
    }

    async fn set_register(&mut self, register: u8, data: u8) -> Result<(), embassy_stm32::i2c::Error> {
        if register == CONTROL_REG_0B {
            self.control_reg0b = ControlReg0b(data);
        }

        self.driver.write_bytes(&[register, data]).await
            .map_err(|_| embassy_stm32::i2c::Error::Timeout)?;
        Ok(())
    }

    pub async fn reset(&mut self) -> Result<(), embassy_stm32::i2c::Error> {
        let mut reg = ControlReg0a(0);
        reg.set_sw_rst(true);
        self.set_register(CONTROL_REG_0A, reg.0).await?;
        self.delay.delay_ms(10).await;
        Ok(())
    }

    pub async fn set_cmm_mode(
        &mut self,
        freq: ContinuousMeasurementFreq,
        period: PeriodicSetInterval,
    ) -> Result<(), embassy_stm32::i2c::Error> {
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

        self.set_register(CONTROL_REG_0B, reg.0).await?;
        Ok(())
    }

    pub async fn init(&mut self) -> Result<MeasurementRaw<u32>, embassy_stm32::i2c::Error> {
        // Read the device ID
        let device_id = self.read_register(DEVICE_ID_REG).await?;
        if device_id != EXPECTED_DEVICE_ID {
            defmt::panic!(
                "Invalid device ID: 0x{:02X}, are you sure this is an mmc5983ma",
                device_id
            );
        }

        self.reset().await?;

        let mut reg = ControlReg0a(0);
        reg.set_bw(0b00);
        reg.set_x_inhibit(false);
        reg.set_yz_inhibit(0b00);
        self.set_register(CONTROL_REG_0A, reg.0).await?;

        let mut reg = ControlReg0c(0);
        reg.set_st_enp(false);
        reg.set_st_enm(false);
        reg.set_spi_3w_en(false);
        self.set_register(CONTROL_REG_0C, reg.0).await?;

        self.set_cmm_mode(ContinuousMeasurementFreq::Off, PeriodicSetInterval::Off)
            .await?;

        // 50hz = 20ms
        self.delay.delay_ms(21).await;

        self.do_measurement_raw().await
    }

    // Read: 16 bit
    // Fixed: Correct bit shifting for 16-bit values
    pub async unsafe fn read_raw_measurement_16(
        &mut self,
    ) -> Result<MeasurementRaw<u16>, embassy_stm32::i2c::Error> {
        let mut data = [0u8; 6];
        self.driver
            .write_read_bytes(&[XOUT_REG], &mut data)
            .await
            .map_err(|_| embassy_stm32::i2c::Error::Timeout)?;
        // Fixed: Correct bit shifting - data[0] is MSB, data[1] is LSB for 16-bit
        let x = ((data[0] as u16) << 8) | (data[1] as u16);
        let y = ((data[2] as u16) << 8) | (data[3] as u16);
        let z = ((data[4] as u16) << 8) | (data[5] as u16);
        Ok(MeasurementRaw { x, y, z })
    }

    // Read: 18 bit
    // Register layout according to MMC5983MA datasheet:
    // XOUT[7:0] in register 0x00, XOUT[15:8] in register 0x01, XOUT[17:16] in register 0x06 bits [7:6]
    // YOUT[7:0] in register 0x02, YOUT[15:8] in register 0x03, YOUT[17:16] in register 0x06 bits [5:4]
    // ZOUT[7:0] in register 0x04, ZOUT[15:8] in register 0x05, ZOUT[17:16] in register 0x06 bits [3:2]
    pub async unsafe fn read_raw_measurement_18(
        &mut self,
    ) -> Result<MeasurementRaw<u32>, embassy_stm32::i2c::Error> {
        let mut data = [0u8; 7];
        self.driver
            .write_read_bytes(&[XOUT_REG], &mut data)
            .await
            .map_err(|_| embassy_stm32::i2c::Error::Timeout)?;
        
        let xyzout2 = XYZOut2(data[6]);
        
        // Reconstruct 18-bit values
        // data[0] = XOUT[7:0] (LSB), data[1] = XOUT[15:8] (middle), xyzout2.get_xout() = XOUT[17:16] (MSB 2 bits)
        let x = (data[0] as u32) | ((data[1] as u32) << 8) | ((xyzout2.get_xout() as u32) << 16);
        let y = (data[2] as u32) | ((data[3] as u32) << 8) | ((xyzout2.get_yout() as u32) << 16);
        let z = (data[4] as u32) | ((data[5] as u32) << 8) | ((xyzout2.get_zout() as u32) << 16);
        
        Ok(MeasurementRaw { x, y, z })
    }

    async fn trigger_measurement(&mut self) -> Result<(), embassy_stm32::i2c::Error> {
        let mut reg = ControlReg09(self.read_register(CONTROL_REG_09).await?);
        reg.set_tm_m(true);
        self.set_register(CONTROL_REG_09, reg.0).await?;
        Ok(())
    }

    async fn wait_for_measurement(&mut self) -> Result<(), embassy_stm32::i2c::Error> {
        for _ in 0..MAX_LOOPS {
            let r = ControlReg08(self.read_register(STATUS_REG).await?);

            if r.get_meas_m_done() {
                // Clear the mag measurement done flag
                let mut clear_reg = ControlReg08(0);
                clear_reg.set_meas_m_done(true);
                self.set_register(STATUS_REG, clear_reg.0).await?;
                return Ok(());
            }

            // If temperature measurement completed, clear it
            if r.get_meas_t_done() {
                let mut clear_reg = ControlReg08(0);
                clear_reg.set_meas_t_done(true);
                self.set_register(STATUS_REG, clear_reg.0).await?;
            }

            self.delay.delay_ms(1).await;
        }
        Err(embassy_stm32::i2c::Error::Timeout)
    }

    async fn perform_set(&mut self) -> Result<(), embassy_stm32::i2c::Error> {
        let mut reg = ControlReg09(self.read_register(CONTROL_REG_09).await?);
        reg.set_set(true);
        self.set_register(CONTROL_REG_09, reg.0).await?;
        // Wait for SET operation to complete (500ns + some margin)
        // Use delay_ns for microsecond delays (10us = 10000ns)
        self.delay.delay_ns(10000).await;
        Ok(())
    }

    async fn perform_reset(&mut self) -> Result<(), embassy_stm32::i2c::Error> {
        let mut reg = ControlReg09(self.read_register(CONTROL_REG_09).await?);
        reg.set_reset(true);
        self.set_register(CONTROL_REG_09, reg.0).await?;
        // Wait for RESET operation to complete (500ns + some margin)
        // Use delay_ns for microsecond delays (10us = 10000ns)
        self.delay.delay_ns(10000).await;
        Ok(())
    }

    pub async fn do_measurement_raw(&mut self) -> Result<MeasurementRaw<u32>, embassy_stm32::i2c::Error> {
        // Simple measurement without SET/RESET compensation
        self.trigger_measurement().await?;
        self.wait_for_measurement().await?;
        unsafe {
            self.read_raw_measurement_18().await
        }
    }

    /// Perform SET/RESET compensated measurement to eliminate offset errors
    /// According to datasheet: Magnetic field = (M_SET - M_RESET) / 2
    /// This eliminates offset errors and residual magnetization
    /// Note: The result is already offset-compensated and centered
    pub async fn do_measurement_compensated(&mut self) -> Result<MeasurementRaw<u32>, embassy_stm32::i2c::Error> {
        // Step 1: Perform SET operation and take measurement
        self.perform_set().await?;
        self.trigger_measurement().await?;
        self.wait_for_measurement().await?;
        let m_set = unsafe { self.read_raw_measurement_18().await? };

        // Step 2: Perform RESET operation and take measurement
        self.perform_reset().await?;
        self.trigger_measurement().await?;
        self.wait_for_measurement().await?;
        let m_reset = unsafe { self.read_raw_measurement_18().await? };

        // Step 3: Calculate compensated value: (M_SET - M_RESET) / 2
        // Convert to signed to handle subtraction properly
        // The formula gives the true magnetic field without offset
        let x_set = m_set.x as i64;
        let x_reset = m_reset.x as i64;
        let x_diff = x_set - x_reset;
        let x_compensated = ((x_diff / 2) + 131072) as u32; // Add midpoint to center around zero

        let y_set = m_set.y as i64;
        let y_reset = m_reset.y as i64;
        let y_diff = y_set - y_reset;
        let y_compensated = ((y_diff / 2) + 131072) as u32;

        let z_set = m_set.z as i64;
        let z_reset = m_reset.z as i64;
        let z_diff = z_set - z_reset;
        let z_compensated = ((z_diff / 2) + 131072) as u32;

        Ok(MeasurementRaw {
            x: x_compensated,
            y: y_compensated,
            z: z_compensated,
        })
    }

    /// Take multiple measurements and average them to reduce noise
    pub async fn do_measurement_averaged(&mut self, samples: usize) -> Result<MeasurementRaw<u32>, embassy_stm32::i2c::Error> {
        let mut sum_x: u64 = 0;
        let mut sum_y: u64 = 0;
        let mut sum_z: u64 = 0;

        for _ in 0..samples {
            let data = self.do_measurement_raw().await?;
            sum_x += data.x as u64;
            sum_y += data.y as u64;
            sum_z += data.z as u64;
        }

        Ok(MeasurementRaw {
            x: (sum_x / samples as u64) as u32,
            y: (sum_y / samples as u64) as u32,
            z: (sum_z / samples as u64) as u32,
        })
    }

    pub async fn get_temp_raw(&mut self) -> Result<Option<u8>, embassy_stm32::i2c::Error> {
        // turn off continuous measurement mode so we can read temp
        let mut r = ControlReg0b(self.read_register(CONTROL_REG_0B).await?);
        let cmm_en = self.control_reg0b.get_cmm_en();
        if cmm_en {
            r.set_cmm_en(false);
            self.set_register(CONTROL_REG_0B, r.0).await?;
        }

        // Request to read temp
        let mut reg = ControlReg09(0);
        reg.set_tm_t(true);
        self.set_register(CONTROL_REG_09, reg.0).await?;

        for _ in 0..100 {
            let r = ControlReg08(self.read_register(STATUS_REG).await?);

            // Fixed: If we get a mag measurement ready signal while waiting for temp, clear it
            if r.get_meas_m_done() {
                // Clear the mag measurement done flag
                let mut clear_reg = ControlReg08(0);
                clear_reg.set_meas_m_done(true);
                self.set_register(STATUS_REG, clear_reg.0).await?;

                // Request to read temp again
                let mut reg = ControlReg09(0);
                reg.set_tm_t(true);
                self.set_register(CONTROL_REG_09, reg.0).await?;
            }

            if r.get_meas_t_done() {
                let t = self.read_register(TEMP_REG).await?;

                // turn back on continuous measurement mode if it was enabled before
                if cmm_en {
                    let mut r = ControlReg0b(self.read_register(CONTROL_REG_0B).await?);
                    r.set_cmm_en(cmm_en);
                    self.set_register(CONTROL_REG_0B, r.0).await?;
                }

                return Ok(Some(t));
            }

            self.delay.delay_ms(15).await;
        }

        Ok(None)
    }

    #[cfg(feature = "float")]
    pub async fn get_temp_c(&mut self) -> Result<Option<f32>, embassy_stm32::i2c::Error> {
        let temp_raw = self.get_temp_raw().await?;
        // range is -75c to 125c, ~0.8c per bit. 0x00 = -75c
        let range = (125.0 - -75.0) / 256.0;
        Ok(temp_raw.map(|temp_raw| (temp_raw as f32) * range + -75.0))
    }
}

