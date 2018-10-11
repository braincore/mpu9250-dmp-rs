//! A library for the MPU-9250 9-axis IMU.
//! The MPU9250 is a combo package that includes the MPU-6500 and AK8963.
pub mod dmp;
mod dmp_constants;

extern crate ak8963;
use ak8963::Ak8963Reg;
extern crate byteorder;
use byteorder::{BigEndian, ByteOrder};
extern crate i2cdev;
use i2cdev::core::*;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};
#[macro_use]
extern crate ndarray;
use ndarray::prelude::*;
use std::convert::From;
use std::thread;
use std::time;

pub const G: f32 = 9.80665;

pub const MPU9250_SLAVE_ADDR: u16 = 0x68;

pub(crate) fn get_i2c_bus_path(i2c_bus: i32) -> String {
    format!("/dev/i2c-{}", i2c_bus)
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
enum Mpu9250Reg {
    SmplrtDiv = 0x019,
    Config = 0x1a,
    GyroConfig = 0x1b,
    AccelConfig = 0x1c,
    AccelConfig2 = 0x1d,
    FifoEn = 0x23,
    I2cMstCtrl = 0x24,
    I2cSlv0Addr = 0x25,
    I2cSlv0Reg = 0x26,
    I2cSlv0Ctrl = 0x27,
    #[allow(dead_code)]
    I2cSlv4Ctrl = 0x34,
    IntPinCfg = 0x37,
    IntEnable = 0x38,
    #[allow(dead_code)]
    AccelXoutH = 0x3b,
    #[allow(dead_code)]
    GyroConfigXoutH = 0x43,
    I2cMstDelayCtrl = 0x67,
    UserCtrl = 0x6a,
    PwrMgmt1 = 0x6b,
    FifoCountH = 0x72,
    FifoRw = 0x74,
    WhoAmI = 0x75,
}

impl Mpu9250Reg {
    pub fn addr(&self) -> u8 {
        *self as u8
    }
}

pub trait Register {
    fn mask(&self) -> u8;
}

enum RegI2cMstCtrl {
    /// Full speed i2c
    Speed400kHz,
    I2cMstPNsr,
    MultMstEn,
}

impl Register for RegI2cMstCtrl {
    fn mask(&self) -> u8 {
        match *self {
            RegI2cMstCtrl::Speed400kHz => 0x0d,
            RegI2cMstCtrl::I2cMstPNsr => 1 << 4,
            RegI2cMstCtrl::MultMstEn => 1 << 7,
        }
    }
}

enum RegI2cSlv0Addr {
    I2cSlv0Rnw,
}

impl Register for RegI2cSlv0Addr {
    fn mask(&self) -> u8 {
        match *self {
            RegI2cSlv0Addr::I2cSlv0Rnw => 1 << 7,
        }
    }
}

enum RegI2cSlv0Ctrl {
    I2cSlv0En,
}

impl Register for RegI2cSlv0Ctrl {
    fn mask(&self) -> u8 {
        match *self {
            RegI2cSlv0Ctrl::I2cSlv0En => 1 << 7,
        }
    }
}

enum RegPwrMgmt1 {
    Hreset,
    Sleep,
}

impl Register for RegPwrMgmt1 {
    fn mask(&self) -> u8 {
        match *self {
            RegPwrMgmt1::Sleep => 1 << 6,
            RegPwrMgmt1::Hreset => 1 << 7,
        }
    }
}

enum RegIntPinCfg {
    BypassEn,
    Actl, // NOTE: Active Low!
}

impl Register for RegIntPinCfg {
    fn mask(&self) -> u8 {
        match *self {
            RegIntPinCfg::BypassEn => 1 << 1,
            RegIntPinCfg::Actl => 1 << 7,
        }
    }
}

#[allow(dead_code)]
enum RegFifoEn {
    Slv0,
    Accel,
    GyroZ,
    GyroY,
    GyroX,
    Temp,
}

impl Register for RegFifoEn {
    fn mask(&self) -> u8 {
        match *self {
            RegFifoEn::Slv0 => 1,
            RegFifoEn::Accel => 1 << 3,
            RegFifoEn::GyroZ => 1 << 4,
            RegFifoEn::GyroY => 1 << 5,
            RegFifoEn::GyroX => 1 << 6,
            RegFifoEn::Temp => 1 << 7,
        }
    }
}

enum RegUserCtrl {
    FifoRst,
    DmpRst,
    I2cMstEn,
    FifoEn,
    DmpEn,
}

impl Register for RegUserCtrl {
    fn mask(&self) -> u8 {
        match *self {
            RegUserCtrl::FifoRst => 1 << 2,
            RegUserCtrl::DmpRst => 1 << 3,
            RegUserCtrl::I2cMstEn => 1 << 5,
            RegUserCtrl::FifoEn => 1 << 6,
            RegUserCtrl::DmpEn => 1 << 7,
        }
    }
}

enum RegIntEnable {
    DmpIntEn,
}

impl Register for RegIntEnable {
    fn mask(&self) -> u8 {
        match *self {
            RegIntEnable::DmpIntEn => 1 << 1,
        }
    }
}

#[allow(dead_code)]
enum RegI2cMstDelayCtrl {
    I2cSlv0DlyEn,
    I2cSlv1DlyEn,
    I2cSlv2DlyEn,
    I2cSlv3DlyEn,
    I2cSlv4DlyEn,
    DelayEsShadow,
}

impl Register for RegI2cMstDelayCtrl {
    fn mask(&self) -> u8 {
        match *self {
            RegI2cMstDelayCtrl::I2cSlv0DlyEn => 1,
            RegI2cMstDelayCtrl::I2cSlv1DlyEn => 1 << 1,
            RegI2cMstDelayCtrl::I2cSlv2DlyEn => 1 << 2,
            RegI2cMstDelayCtrl::I2cSlv3DlyEn => 1 << 3,
            RegI2cMstDelayCtrl::I2cSlv4DlyEn => 1 << 4,
            RegI2cMstDelayCtrl::DelayEsShadow => 1 << 7,
        }
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone, Debug)]
pub enum AccelFsr {
    Opt2G,
    Opt4G,
    Opt8G,
    Opt16G,
}

/// The full-scale range of the accelerometer.
/// 2G gives the highest sensitivity, but saturates at +-2G.
/// 16G gives the lowest sensitivity, but has the largest range.
impl AccelFsr {
    /// Sensitivity is the measurement of the LSB.
    pub fn get_sensitivity_g(&self) -> f32 {
        return match *self {
            AccelFsr::Opt2G => 2.0,
            AccelFsr::Opt4G => 4.0,
            AccelFsr::Opt8G => 8.0,
            AccelFsr::Opt16G => 16.0,
        } / 32768.0;
    }

    // Returns sensitivity in m/s/s.
    pub fn get_sensitivity_mss(&self) -> f32 {
        // Repeat code from get_sensitivity_g to reduce floating point
        // precision loss from floating point math.
        return match *self {
            AccelFsr::Opt2G => 2.0,
            AccelFsr::Opt4G => 4.0,
            AccelFsr::Opt8G => 8.0,
            AccelFsr::Opt16G => 16.0,
        } * G / 32768.0;
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone, Debug)]
pub enum AccelDlpf {
    OptOff,
    Opt184,
    Opt92,
    Opt41,
    Opt20,
    Opt10,
    Opt5,
}

/// The full-scale range of the gyroscope.
/// 250 gives the highest resolution, but saturates at +-250 degrees per second.
/// 2000 gives the lowest resolution, but saturates at +-2000dps.
#[allow(dead_code)]
#[derive(Copy, Clone, Debug)]
pub enum GyroFsr {
    Opt250,
    Opt500,
    Opt1000,
    Opt2000,
}

impl GyroFsr {
    pub fn get_scalar(&self) -> f32 {
        return match *self {
            GyroFsr::Opt250 => 250.0,
            GyroFsr::Opt500 => 500.0,
            GyroFsr::Opt1000 => 1000.0,
            GyroFsr::Opt2000 => 2000.0,
        } / 32768.0;
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone, Debug)]
pub enum GyroDlpf {
    OptOff,
    Opt184,
    Opt92,
    Opt41,
    Opt20,
    Opt10,
    Opt5,
}

pub struct Mpu9250 {
    i2c_dev: LinuxI2CDevice,
    pub accel_fsr: AccelFsr,
    pub gyro_fsr: GyroFsr,
    pub debug: bool,
    pub ak8963_i2c_addr: u16,
}

impl Mpu9250 {
    /// Connects to MPU-9250 and checks the WHOAMI.
    ///
    /// If i2c_addr is None, defaults to 0x68. From the datasheet, the chip can
    /// be configured to respond to 0x68 or 0x69.
    ///
    /// If whoami is None, defaults to 0x71 per datasheet, but it is
    /// configurable on the chip.
    ///
    /// If ak8963_i2c_addr is None, defaults to 0x0c.
    ///
    /// Configures the MPU-9250 to:
    /// * Acceleration full-scale range: 2G
    /// * Gyroscope full-scale range: 2000dps
    pub fn new(
        i2c_bus: i32,
        i2c_addr: Option<u16>,
        whoami: Option<u8>,
        ak8963_i2c_addr: Option<u16>,
    ) -> Result<Self, LinuxI2CError> {
        let mut i2c_dev = LinuxI2CDevice::new(get_i2c_bus_path(i2c_bus), i2c_addr.unwrap_or(0x68))?;

        let mut buf1 = [0u8; 1];
        i2c_dev.write(&[Mpu9250Reg::WhoAmI.addr()])?;
        i2c_dev.read(&mut buf1)?;
        if buf1[0] != whoami.unwrap_or(0x71) {
            panic!("Unexpected WHOAMI value: {:?}", buf1[0]);
        }

        let mut mpu = Self {
            i2c_dev,
            accel_fsr: AccelFsr::Opt2G,
            gyro_fsr: GyroFsr::Opt2000,
            debug: false,
            ak8963_i2c_addr: ak8963_i2c_addr.unwrap_or(0x0c),
        };
        mpu.write_config()?;
        Ok(mpu)
    }

    /// Debug mode prints various messages to stdout.
    pub fn set_debug(&mut self, debug: bool) {
        self.debug = debug;
    }

    /// Writes the accel and gyro FSRs to the device.
    pub fn write_config(&mut self) -> Result<(), LinuxI2CError> {
        let accel_fsr = self.accel_fsr.clone();
        self.write_accel_fsr(accel_fsr)?;
        let gyro_fsr = self.gyro_fsr.clone();
        self.write_gyro_fsr(gyro_fsr)?;
        Ok(())
    }

    pub fn write_accel_fsr(&mut self, accel_fsr: AccelFsr) -> Result<(), LinuxI2CError> {
        let accel_config_byte: u8 = match accel_fsr {
            AccelFsr::Opt2G => 0,
            AccelFsr::Opt4G => 1,
            AccelFsr::Opt8G => 2,
            AccelFsr::Opt16G => 3,
        } << 3;
        self.i2c_dev
            .write(&[Mpu9250Reg::AccelConfig.addr(), accel_config_byte])?;
        self.accel_fsr = accel_fsr;
        Ok(())
    }

    pub fn write_accel_dlpf(&mut self, accel_dlpf: AccelDlpf) -> Result<(), LinuxI2CError> {
        let accel_config_2_byte: u8 = match accel_dlpf {
            AccelDlpf::OptOff => 7,
            AccelDlpf::Opt184 => 1,
            AccelDlpf::Opt92 => 2,
            AccelDlpf::Opt41 => 3,
            AccelDlpf::Opt20 => 4,
            AccelDlpf::Opt10 => 5,
            AccelDlpf::Opt5 => 6,
        };
        self.i2c_dev
            .write(&[Mpu9250Reg::AccelConfig2.addr(), accel_config_2_byte])?;
        Ok(())
    }

    pub fn write_gyro_fsr(&mut self, gyro_fsr: GyroFsr) -> Result<(), LinuxI2CError> {
        let gyro_config_byte: u8 = match gyro_fsr {
            GyroFsr::Opt250 => 0,
            GyroFsr::Opt500 => 1,
            GyroFsr::Opt1000 => 2,
            GyroFsr::Opt2000 => 3,
        } << 3;
        self.i2c_dev
            .write(&[Mpu9250Reg::GyroConfig.addr(), gyro_config_byte])?;
        self.gyro_fsr = gyro_fsr;
        Ok(())
    }

    pub fn write_gyro_dlpf(&mut self, gyro_dlpf: GyroDlpf) -> Result<(), LinuxI2CError> {
        let config_byte: u8 = match gyro_dlpf {
            GyroDlpf::OptOff => 7,
            GyroDlpf::Opt184 => 1,
            GyroDlpf::Opt92 => 2,
            GyroDlpf::Opt41 => 3,
            GyroDlpf::Opt20 => 4,
            GyroDlpf::Opt10 => 5,
            GyroDlpf::Opt5 => 6,
        };
        self.i2c_dev
            .write(&[Mpu9250Reg::Config.addr(), config_byte])?;
        // Empirically, it takes around ~100-120ms for the gyro registers to
        // contain non-zero values.
        self.wait_for_nonzero_gyro()?;
        Ok(())
    }

    /// When the gyro isn't initialized, the register values read zero. This
    /// makes it easy to wait for the gyro to initialize. TODO: Is there a
    /// ready bit we should be checking instead?
    pub fn wait_for_nonzero_gyro(&mut self) -> Result<(), LinuxI2CError> {
        loop {
            let sample = self.read_sample()?;
            if sample.gyro == array![0.0, 0.0, 0.0] {
                thread::sleep(time::Duration::from_millis(10));
            } else {
                return Ok(());
            }
        }
    }

    /// Reads from dedicated accel & gyro registers. (Not from FIFO)
    pub fn read_sample(&mut self) -> Result<Mpu9250Sample, LinuxI2CError> {
        let mut buf: [u8; 14] = [0u8; 14];
        self.i2c_dev.write(&[Mpu9250Reg::AccelXoutH.addr()])?;
        self.i2c_dev.read(&mut buf)?;

        Ok(Self::parse_accel_and_gyro_and_temp(
            &buf,
            self.accel_fsr.get_sensitivity_mss(),
            self.gyro_fsr.get_scalar(),
        ))
    }

    /// data should be a 12-byte slice.
    /// accel: [0,5]
    /// gyro: [6,11]
    pub(crate) fn parse_accel_and_gyro(
        data: &[u8],
        accel_fsr_scalar: f32,
        gyro_fsr_scalar: f32,
    ) -> Mpu9250Sample {
        let accel_v_raw = Self::buffer_to_array1x3_i16(&data[0..6]);
        let gyro_v_raw = Self::buffer_to_array1x3_i16(&data[6..12]);

        let accel_v = accel_fsr_scalar * accel_v_raw.map(|e| *e as f32);
        let gyro_v = gyro_fsr_scalar * gyro_v_raw.map(|e| *e as f32);

        Mpu9250Sample {
            accel: accel_v,
            accel_raw: accel_v_raw,
            gyro: gyro_v,
            gyro_raw: gyro_v_raw,
        }
    }

    /// data should be a 14-byte slice.
    /// accel: [0,5]
    /// temp: [6,7]
    /// gyro: [5,13]
    pub(crate) fn parse_accel_and_gyro_and_temp(
        data: &[u8],
        accel_fsr_scalar: f32,
        gyro_fsr_scalar: f32,
    ) -> Mpu9250Sample {
        // TODO: Does not parse temp right now.
        let accel_v_raw = Self::buffer_to_array1x3_i16(&data[0..6]);
        let gyro_v_raw = Self::buffer_to_array1x3_i16(&data[8..14]);

        let accel_v = accel_fsr_scalar * accel_v_raw.map(|e| *e as f32);
        let gyro_v = gyro_fsr_scalar * gyro_v_raw.map(|e| *e as f32);

        Mpu9250Sample {
            accel: accel_v,
            accel_raw: accel_v_raw,
            gyro: gyro_v,
            gyro_raw: gyro_v_raw,
        }
    }

    /// By enabling bypass, the magnetometer will be directly accessible over
    /// the i2c bus.
    pub fn enable_bypass(&mut self) -> Result<(), LinuxI2CError> {
        self.i2c_dev.write(&[Mpu9250Reg::UserCtrl.addr(), 0])?;
        self.i2c_dev.write(&[
            Mpu9250Reg::IntPinCfg.addr(),
            RegIntPinCfg::Actl.mask() | RegIntPinCfg::BypassEn.mask(),
        ])?;
        return Ok(());
    }

    pub fn disable_bypass(&mut self) -> Result<(), LinuxI2CError> {
        self.i2c_dev
            .write(&[Mpu9250Reg::UserCtrl.addr(), RegUserCtrl::I2cMstEn.mask()])?;
        self.i2c_dev
            .write(&[Mpu9250Reg::IntPinCfg.addr(), RegIntPinCfg::Actl.mask()])?;
        Ok(())
    }

    fn buffer_to_array1x3_i16(buf: &[u8]) -> Array1<i16> {
        array![
            BigEndian::read_i16(&buf[0..2]),
            BigEndian::read_i16(&buf[2..4]),
            BigEndian::read_i16(&buf[4..6])
        ]
    }

    /// After resetting the device, the FSR and DLPF values will need to be
    /// set again.
    pub fn reset(&mut self) -> Result<(), LinuxI2CError> {
        // Write the reset bit
        self.i2c_dev
            .write(&[Mpu9250Reg::PwrMgmt1.addr(), RegPwrMgmt1::Hreset.mask()])?;

        // Turn off all other power mgmt features
        self.i2c_dev.write(&[Mpu9250Reg::PwrMgmt1.addr(), 0u8])?;

        thread::sleep(time::Duration::from_millis(100));
        Ok(())
    }

    pub fn power_off(&mut self) -> Result<(), LinuxI2CError> {
        // Write the reset bit
        self.i2c_dev
            .write(&[Mpu9250Reg::PwrMgmt1.addr(), RegPwrMgmt1::Hreset.mask()])?;
        // Write the sleep bit
        self.i2c_dev
            .write(&[Mpu9250Reg::PwrMgmt1.addr(), RegPwrMgmt1::Sleep.mask()])?;
        Ok(())
    }

    pub fn dmp_enable(&mut self) -> Result<(), LinuxI2CError> {
        self.dmp_interrupt_disable()?;

        self.disable_bypass()?;

        // TODO: Test if this really purges FIFO items
        self.i2c_dev.write(&[Mpu9250Reg::FifoEn.addr(), 0])?;

        self.dmp_interrupt_enable()?;

        self.reset_fifo()?;
        Ok(())
    }

    pub fn dmp_disable(&mut self) -> Result<(), LinuxI2CError> {
        self.dmp_interrupt_disable()?;
        self.i2c_dev.write(&[Mpu9250Reg::FifoEn.addr(), 0])?;
        self.reset_fifo()?;
        Ok(())
    }

    pub fn dmp_interrupt_enable(&mut self) -> Result<(), LinuxI2CError> {
        self.i2c_dev
            .write(&[Mpu9250Reg::IntEnable.addr(), RegIntEnable::DmpIntEn.mask()])?;
        // Disable all other FIFO features (except for the DMP)
        self.i2c_dev.write(&[Mpu9250Reg::FifoEn.addr(), 0])?;
        Ok(())
    }

    pub fn dmp_interrupt_disable(&mut self) -> Result<(), LinuxI2CError> {
        self.i2c_dev.write(&[Mpu9250Reg::IntEnable.addr(), 0])?;
        // Disable all other FIFO features (except for the DMP)
        self.i2c_dev.write(&[Mpu9250Reg::FifoEn.addr(), 0])?;
        Ok(())
    }

    /// Sets up mag interface on Slv0 interface.
    ///
    /// Does not enable fifo. You must still call enable_fifo() to begin
    /// populating the fifo with mag data.
    ///
    /// Assumes mag is connected to Slv0.
    pub fn setup_mag_in_fifo(&mut self) -> Result<(), LinuxI2CError> {
        // Enable master and clock speed
        self.i2c_dev.write(&[
            Mpu9250Reg::I2cMstCtrl.addr(),
            RegI2cMstCtrl::Speed400kHz.mask()
                | RegI2cMstCtrl::MultMstEn.mask()
                | RegI2cMstCtrl::I2cMstPNsr.mask(),
        ])?;

        // Set slave 0 address to magnetometer i2c address
        self.i2c_dev.write(&[
            Mpu9250Reg::I2cSlv0Addr.addr(),
            RegI2cSlv0Addr::I2cSlv0Rnw.mask() | self.ak8963_i2c_addr as u8,
        ])?;

        // Set mag data register to read from
        self.i2c_dev
            .write(&[Mpu9250Reg::I2cSlv0Reg.addr(), Ak8963Reg::Hxl.addr()])?;

        // Read 7 bytes from slave 0.
        self.i2c_dev.write(&[
            Mpu9250Reg::I2cSlv0Ctrl.addr(),
            RegI2cSlv0Ctrl::I2cSlv0En.mask() | 0x07,
        ])?;

        // TEST: Couldn't get this to work. It's supposed to cause slaves to
        // be sampled less frequently than the sample rate. Don't need it.
        //self.i2c_dev.write(&[Mpu9250Reg::I2cMstDelayCtrl.addr(), RegI2cMstDelayCtrl::I2cSlv0DlyEn.mask()])?;
        //self.i2c_dev.write(&[Mpu9250Reg::I2cSlv4Ctrl.addr(), 100u8])?;

        Ok(())
    }

    /// rate is specified as the number of samples per second (Hz).
    /// It controls the rate the FIFO is populated by the MPU. It also sets
    /// the upper bound rate the FIFO is populated by the DMP.
    /// Note: Per the docs, this only works if internal sampling is set to
    /// 1kHz.
    pub fn set_sample_rate(&mut self, rate: u16) -> Result<(), LinuxI2CError> {
        if rate > 1000 || rate < 4 {
            panic!("Error: Invalid sample rate.");
        }

        // Derived from: Sample Rate = Internal Sample Rate / (1 + SMPLRT_DIV)
        let smplrt_div = (1000 / rate) as u8 - 1;
        self.i2c_dev
            .write(&[Mpu9250Reg::SmplrtDiv.addr(), smplrt_div])?;
        Ok(())
    }

    /// Disables and then re-enables the fifo.
    /// This clears the contents of the fifo.
    pub fn reset_fifo(&mut self) -> Result<(), LinuxI2CError> {
        self.disable_fifo()?;
        self.enable_fifo()?;
        Ok(())
    }

    /// Enables interrupt and fifo.
    pub fn enable_fifo(&mut self) -> Result<(), LinuxI2CError> {
        self.i2c_dev
            .write(&[Mpu9250Reg::IntEnable.addr(), RegIntEnable::DmpIntEn.mask()])?;
        self.i2c_dev
            .write(&[Mpu9250Reg::FifoEn.addr(), RegFifoEn::Slv0.mask()])?;
        self.i2c_dev.write(&[
            Mpu9250Reg::UserCtrl.addr(),
            RegUserCtrl::I2cMstEn.mask() | RegUserCtrl::FifoEn.mask() | RegUserCtrl::DmpEn.mask(),
        ])?;

        Ok(())
    }

    /// Disables interrupt and fifo.
    /// Also resets the fifo and dmp.
    pub fn disable_fifo(&mut self) -> Result<(), LinuxI2CError> {
        self.i2c_dev.write(&[Mpu9250Reg::IntEnable.addr(), 0])?;
        self.i2c_dev.write(&[Mpu9250Reg::FifoEn.addr(), 0])?;
        self.i2c_dev.write(&[
            Mpu9250Reg::UserCtrl.addr(),
            RegUserCtrl::FifoRst.mask() | RegUserCtrl::DmpRst.mask(),
        ])?;
        Ok(())
    }

    /// Returns the number of bytes in the fifo.
    fn read_fifo_byte_count(&mut self) -> Result<u16, LinuxI2CError> {
        let mut data = [0u8; 2];
        self.i2c_dev.write(&[Mpu9250Reg::FifoCountH.addr()])?;
        self.i2c_dev.read(&mut data)?;

        // Technically only bits 0-12 are valid, so let's be paranoid and
        // zero out higher bits.
        data[0] = data[0] & 0x1f;
        let length: u16 = BigEndian::read_u16(&data);
        if self.debug {
            println!("FIFO byte count: {:?}", length);
        }
        Ok(length)
    }

    /// Reads exactly count bytes from the fifo.
    pub fn read_fifo_strict(&mut self, count: u16) -> Result<Vec<u8>, FifoReadError> {
        let available_bytes = self.read_fifo_byte_count()?;
        if available_bytes < count {
            return Err(FifoReadError::InsufficientData(available_bytes));
        }

        let mut data = vec![0; count as usize];
        self.i2c_dev.write(&[Mpu9250Reg::FifoRw.addr()])?;
        self.i2c_dev.read(&mut data)?;
        Ok(data)
    }
}

#[derive(Clone, Debug)]
pub struct Mpu9250Sample {
    pub accel: Array1<f32>,
    pub accel_raw: Array1<i16>,
    pub gyro: Array1<f32>,
    pub gyro_raw: Array1<i16>,
}

#[derive(Debug)]
pub enum FifoReadError {
    InsufficientData(u16),
    I2c(LinuxI2CError),
}

impl From<LinuxI2CError> for FifoReadError {
    fn from(e: LinuxI2CError) -> Self {
        FifoReadError::I2c(e)
    }
}

#[cfg(test)]
mod tests {
    use super::Mpu9250;
    use std::env;

    fn get_i2c_bus() -> i32 {
        match env::var("MPU9250_I2C_BUS") {
            Ok(bus_string) => bus_string
                .parse()
                .expect("Could not convert MPU9250_I2C_BUS env var to i32."),
            Err(_) => 1,
        }
    }

    fn get_i2c_addr() -> Option<u16> {
        match env::var("MPU9250_I2C_ADDR") {
            Ok(addr_string) => Some(
                addr_string
                    .parse()
                    .expect("Could not convert MPU9250_I2C_ADDR env var to u16."),
            ),
            Err(_) => None,
        }
    }

    fn get_whoami() -> Option<u8> {
        match env::var("MPU9250_WHOAMI") {
            Ok(whoami_string) => Some(
                whoami_string
                    .parse()
                    .expect("Could not convert MPU9250_WHOAMI env var to u8."),
            ),
            Err(_) => None,
        }
    }

    fn get_ak8963_i2c_addr() -> Option<u16> {
        match env::var("AK8963_I2C_ADDR") {
            Ok(addr_string) => Some(
                addr_string
                    .parse()
                    .expect("Could not convert AK8963_I2C_ADDR env var to u16."),
            ),
            Err(_) => None,
        }
    }

    #[test]
    fn basic() {
        let mut mpu9250 = Mpu9250::new(
            get_i2c_bus(),
            get_i2c_addr(),
            get_whoami(),
            get_ak8963_i2c_addr(),
        ).expect("Could not connect to MPU9250");
        mpu9250.read_sample().unwrap();
    }
}
