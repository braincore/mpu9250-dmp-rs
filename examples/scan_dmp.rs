//! Receive interrupts from the DMP at 100Hz.
//! Use the interrupts as triggers to read from the fifo.
#[macro_use]
extern crate clap;
use clap::{App, Arg};
extern crate mpu9250_dmp;
use mpu9250_dmp::dmp::{DigitalMotionProcessor, DmpFifoReadError};
extern crate sysfs_gpio;
use sysfs_gpio::{Direction, Edge, Pin};

pub fn main() {
    let matches = App::new("MPU-9250 DMP FIFO Scanner")
        .arg(
            Arg::with_name("bus")
                .short("b")
                .long("bus")
                .value_name("BUS")
                .help("The i2c bus for the MPU-9250.")
                .takes_value(true),
        )
        .arg(
            Arg::with_name("addr")
                .short("a")
                .long("addr")
                .value_name("ADDR")
                .help("The i2c addr for the MPU-9250.")
                .takes_value(true),
        )
        .arg(
            Arg::with_name("whoami")
                .short("w")
                .long("whoami")
                .value_name("WHOAMI")
                .help("The WHOAMI register value for the MPU-9250.")
                .takes_value(true),
        )
        .arg(
            Arg::with_name("interrupt-pin")
                .short("i")
                .long("interrupt-pin")
                .value_name("INTERRUPT-PIN")
                .help("The interrupt pin connected to the MPU-9250.")
                .takes_value(true),
        )
        .arg(
            Arg::with_name("rate")
                .short("r")
                .long("rate")
                .value_name("RATE")
                .help("The rate the DMP populates the FIFO & triggers interrupts.")
                .takes_value(true),
        )
        .get_matches();
    let i2c_bus = value_t!(matches, "bus", i32).unwrap_or(1);
    let i2c_addr = value_t!(matches, "addr", u16).ok();
    let whoami = value_t!(matches, "whoami", u8).ok();
    let int_pin = value_t!(matches, "interrupt-pin", u64).unwrap_or_else(|e| e.exit());
    let rate = value_t!(matches, "rate", u16).unwrap_or(100);

    let mut dmp = DigitalMotionProcessor::new(i2c_bus, i2c_addr, whoami, None).unwrap();
    dmp.mpu.dmp_interrupt_disable().unwrap();
    dmp.initialize(rate).unwrap();
    dmp.mpu.reset_fifo().unwrap();

    let input = Pin::new(int_pin);
    input
        .with_exported(|| {
            input.set_direction(Direction::In)?;
            input.set_edge(Edge::FallingEdge)?;
            let mut poller = input.get_poller()?;
            loop {
                match poller.poll(1000)? {
                    Some(_) => match dmp.read_fifo() {
                        Ok(mut dmp_sample) => {
                            println!("{:#?}", dmp_sample);
                        }
                        Err(e) => match e {
                            DmpFifoReadError::Read(e) => {
                                println!("Read error: {:?}", e);
                            }
                            DmpFifoReadError::Parse => {
                                dmp.mpu.reset_fifo().unwrap();
                                println!("Parsing failed. Resetting fifo.");
                            }
                        },
                    },
                    None => {}
                }
            }
        })
        .unwrap();
}
