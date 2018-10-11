use super::dmp_constants::*;
use super::get_i2c_bus_path;
use super::{
    AccelDlpf, AccelFsr, FifoReadError, GyroDlpf, GyroFsr, MPU9250_SLAVE_ADDR, Mpu9250,
    Mpu9250Sample,
};
use ak8963::{
    Ak8963, Ak8963Sample, SampleRate as Ak8963SampleRate, Sensitivity as Ak8963Sensitivity,
};
use byteorder::{BigEndian, ByteOrder, WriteBytesExt};
use i2cdev::core::*;
use i2cdev::linux::{LinuxI2CDevice, LinuxI2CError};
use ndarray::prelude::*;
use std::cmp;

const GYRO_SF: u32 = 46850825;

const QUAT_ERROR_THRESH: i32 = (1 << 16);
const QUAT_MAG_SQ_NORMALIZED: i32 = (1 << 28);
const QUAT_MAG_SQ_MIN: i32 = (QUAT_MAG_SQ_NORMALIZED - QUAT_ERROR_THRESH);
const QUAT_MAG_SQ_MAX: i32 = (QUAT_MAG_SQ_NORMALIZED + QUAT_ERROR_THRESH);

#[derive(Clone, Copy)]
enum Mpu6500Reg {
    BankSel = 0x6d,
    MemRw = 0x6f,
    PrgmStartH = 0x70,
}

impl Mpu6500Reg {
    pub fn addr(&self) -> u8 {
        *self as u8
    }
}

const MPU6500_BANK_SIZE: usize = 256;

pub enum DmpInterruptMode {
    /// Interrupts after one FIFO Period (set by sample rate)
    Continuous,
    /// Interrupts on a tap event
    Gesture,
}

pub struct DigitalMotionProcessor {
    i2c_dev: LinuxI2CDevice,
    pub mpu: Mpu9250,
    pub mag: Ak8963,
    pub debug: bool,
}

impl DigitalMotionProcessor {
    pub fn new(
        i2c_bus: i32,
        mpu_addr: Option<u16>,
        mpu_whoami: Option<u8>,
        ak_addr: Option<u16>,
    ) -> Result<DigitalMotionProcessor, LinuxI2CError> {
        let i2c_dev = LinuxI2CDevice::new(get_i2c_bus_path(i2c_bus), MPU9250_SLAVE_ADDR)?;

        let mut mpu = Mpu9250::new(i2c_bus, mpu_addr, mpu_whoami, None)?;

        mpu.enable_bypass()?;

        let mag = Ak8963::new(
            i2c_bus,
            ak_addr,
            Ak8963Sensitivity::Opt16bit,
            Ak8963SampleRate::Opt100Hz,
        ).unwrap();

        mpu.disable_bypass()?;

        Ok(DigitalMotionProcessor {
            i2c_dev,
            mpu,
            mag,
            debug: false,
        })
    }

    pub fn set_debug(&mut self, debug: bool) {
        self.debug = debug;
        self.mpu.set_debug(debug);
    }

    pub fn initialize(&mut self, sample_rate: u16) -> Result<(), LinuxI2CError> {
        self.mpu.reset()?;
        self.mpu.set_sample_rate(sample_rate)?;
        self.mpu.write_accel_fsr(AccelFsr::Opt2G)?;
        self.mpu.write_gyro_fsr(GyroFsr::Opt250)?;
        self.mpu.write_accel_dlpf(AccelDlpf::Opt41)?;
        self.mpu.write_gyro_dlpf(GyroDlpf::Opt41)?;

        self.load_motion_driver_firmware()?;

        // Match MPU's sample rate.
        self.write_fifo_rate(200)?;

        self.enable_quaternion_accel_gyro_in_fifo()?;

        self.set_interrupt_mode(DmpInterruptMode::Continuous)?;

        self.mpu.dmp_enable()?;

        self.mpu.setup_mag_in_fifo()?;
        Ok(())
    }

    pub fn write_memory(&mut self, mem_addr: u16, data: &[u8]) -> Result<(), LinuxI2CError> {
        let length = data.len();
        if (((mem_addr & 0xFF) as usize) + length) > MPU6500_BANK_SIZE {
            panic!("Exceeds bank size");
        }

        // First, write the mem address to bank select.
        // Note: Address written as little-endian
        self.i2c_dev.write(&[
            Mpu6500Reg::BankSel.addr(),
            (mem_addr >> 8) as u8,
            (mem_addr & 0xFF) as u8,
        ])?;

        // Second, write contents.
        let mut data2 = vec![Mpu6500Reg::MemRw.addr()];
        for x in data.iter() {
            data2.push(*x);
        }
        self.i2c_dev.write(&data2)?;
        Ok(())
    }

    pub fn read_memory(&mut self, mem_addr: u16, length: u8) -> Result<Vec<u8>, LinuxI2CError> {
        if (((mem_addr & 0xFF) + (length as u16)) as usize) > MPU6500_BANK_SIZE {
            panic!("Exceeds bank size");
        }

        // First, write the mem address to bank select.
        // Note: Address written as little-endian
        self.i2c_dev.write(&[
            Mpu6500Reg::BankSel.addr(),
            (mem_addr >> 8) as u8,
            (mem_addr & 0xFF) as u8,
        ])?;

        let mut data = vec![0; length as usize];
        self.i2c_dev.write(&[Mpu6500Reg::MemRw.addr()])?;
        self.i2c_dev.read(&mut data)?;
        Ok(data)
    }

    fn load_motion_driver_firmware(&mut self) -> Result<(), LinuxI2CError> {
        let mut write_offset: u16 = 0; // equivalent to bytes written
        while write_offset < DMP_CODE_SIZE {
            let bytes_to_write = cmp::min(DMP_LOAD_CHUNK, DMP_CODE_SIZE - write_offset);
            let firmware_slice =
                &DMP_FIRMWARE[write_offset as usize..(write_offset + bytes_to_write) as usize];
            self.write_memory(write_offset, firmware_slice)?;
            write_offset += bytes_to_write;
            // TODO: Read DMP memory to verify correctness of write.
        }

        self.i2c_dev
            .write(&[Mpu6500Reg::PrgmStartH.addr(), 0x04, 0x00])?;
        Ok(())
    }

    fn set_interrupt_mode(&mut self, mode: DmpInterruptMode) -> Result<(), LinuxI2CError> {
        match mode {
            DmpInterruptMode::Continuous => {
                self.write_memory(
                    CFG_FIFO_ON_EVENT,
                    &[
                        0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9,
                    ],
                )?;
            }
            DmpInterruptMode::Gesture => {
                self.write_memory(
                    CFG_FIFO_ON_EVENT,
                    &[
                        0xda, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0xda, 0xb4, 0xda,
                    ],
                )?;
            }
        };
        Ok(())
    }

    fn write_fifo_rate(&mut self, rate: u16) -> Result<(), LinuxI2CError> {
        // Sets the rate the DMP adds data to the FIFO relative to the sample
        // rate. Setting rate=200 puts DMP on par with the sample rate.
        // rate=100 causes the DMP to enqueue at half the sample rate.
        let regs_end: [u8; 12] = [
            DINAFE, DINAF2, DINAAB, 0xc4, DINAAA, DINAF1, DINADF, DINADF, 0xbb, 0xaf, DINADF,
            DINADF,
        ];
        let div = ((DMP_MAX_RATE / rate) - 1) as u16;

        self.write_memory(D_0_22, &[(div >> 8) as u8, (div & 0xff) as u8])?;
        self.write_memory(CFG_6, &regs_end)?;
        Ok(())
    }

    pub fn enable_6x_lp_quat(&mut self) -> Result<(), LinuxI2CError> {
        self.write_memory(CFG_8, &[DINA20, DINA28, DINA30, DINA38])?;
        Ok(())
    }

    pub fn disable_6x_lp_quat(&mut self) -> Result<(), LinuxI2CError> {
        self.write_memory(CFG_8, &[0xA3; 4])?;
        Ok(())
    }

    fn enable_quaternion_accel_gyro_in_fifo(&mut self) -> Result<(), LinuxI2CError> {
        // Set integration scale factor.
        let mut tmp = vec![];
        tmp.write_u32::<BigEndian>(GYRO_SF).unwrap();
        self.write_memory(D_0_104, &tmp)?;

        // Send sensor data to the FIFO.
        let mut buf10: [u8; 10] = [0; 10];

        buf10[0] = 0xA3;
        // Raw accel
        buf10[1] = 0xC0;
        buf10[2] = 0xC8;
        buf10[3] = 0xC2;
        // Raw gyro
        buf10[4] = 0xC4;
        buf10[5] = 0xCC;
        buf10[6] = 0xC6;
        // Unknown
        buf10[7] = 0xA3;
        buf10[8] = 0xA3;
        buf10[9] = 0xA3;
        self.write_memory(CFG_15, &buf10)?;

        // No tap or android orient
        self.write_memory(CFG_27, &[0xD8])?;
        // Disable tap feature
        self.write_memory(CFG_20, &[0xD8])?;
        // Disable orientation feature
        self.write_memory(CFG_ANDROID_ORIENT_INT, &[0xD8])?;

        self.enable_6x_lp_quat()?;
        self.mpu.reset_fifo()?;

        Ok(())
    }

    pub fn read_fifo(&mut self) -> Result<DmpSample, DmpFifoReadError> {
        let data = self
            .mpu
            .read_fifo_strict(35)
            .map_err(|e| DmpFifoReadError::Read(e))?;

        if self.debug {
            println!("FIFO packet data (mag+mpu): {:?}", &data[..]);
        }
        let mag_output = self.mag.parse_sample_data(&data[0..7]);
        let quaternion = Self::parse_quaternion(&data[7..23]).ok_or(DmpFifoReadError::Parse)?;
        let taitbryan = Self::quaternion_to_taitbryan(&quaternion);
        let mpu_output = Mpu9250::parse_accel_and_gyro(
            &data[23..35],
            self.mpu.accel_fsr.get_sensitivity_mss(),
            self.mpu.gyro_fsr.get_scalar(),
        );

        Ok(DmpSample {
            mag: mag_output,
            mpu: mpu_output,
            quaternion,
            taitbryan,
        })
    }

    fn parse_quaternion(data: &[u8]) -> Option<Array1<f32>> {
        let q_data = array![
            BigEndian::read_i32(&data[0..4]),
            BigEndian::read_i32(&data[4..8]),
            BigEndian::read_i32(&data[8..12]),
            BigEndian::read_i32(&data[12..16]),
        ];

        let mut q: Array1<f64> = array![
            q_data[0] as f64,
            q_data[1] as f64,
            q_data[2] as f64,
            q_data[3] as f64,
        ];
        let norm = (q[0].powi(2) + q[1].powi(2) + q[2].powi(2) + q[3].powi(2)).sqrt();
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;

        // Check that the magnitude normalizes to one.
        // We scale down the numbers to avoid 64-bit int math.
        let mut q_verify: Vec<i32> = vec![
            q_data[0] >> 16,
            q_data[1] >> 16,
            q_data[2] >> 16,
            q_data[3] >> 16,
        ];
        for v in &mut q_verify {
            match (*v).checked_mul(*v) {
                None => return None,
                Some(sq_v) => *v = sq_v,
            }
        }
        // Check for integer overflow.
        let quat_mag_sq = q_verify[0].checked_add(q_verify[1]).and_then(|v| {
            v.checked_add(q_verify[2])
                .and_then(|v2| v2.checked_add(q_verify[3]))
        });
        match quat_mag_sq {
            None => return None,
            Some(quat_mag_sq) => {
                if (quat_mag_sq < QUAT_MAG_SQ_MIN) || (quat_mag_sq > QUAT_MAG_SQ_MAX) {
                    return None;
                }
            }
        }
        Some(array![q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32])
    }

    pub fn quaternion_to_taitbryan(q: &Array1<f32>) -> Array1<f32> {
        let tb1 = 2.0 * (q[0] * q[2] - q[1] * q[3]).asin();
        let tb0 =
            (2.0 * (q[2] * q[3] + q[0] * q[1])).atan2(1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
        let tb2 =
            (2.0 * (q[1] * q[2] + q[0] * q[3])).atan2(1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
        array![tb0, tb1, tb2]
    }
}

#[derive(Clone, Debug)]
pub struct DmpSample {
    pub mag: Option<Ak8963Sample>,
    pub mpu: Mpu9250Sample,
    pub quaternion: Array1<f32>,
    pub taitbryan: Array1<f32>,
}

#[derive(Debug)]
pub enum DmpFifoReadError {
    Read(FifoReadError),
    Parse,
}
