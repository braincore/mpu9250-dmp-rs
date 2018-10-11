//! Poll the MPU9250 at ~100Hz.
#[macro_use]
extern crate clap;
use clap::{App, Arg};
extern crate mpu9250_dmp;
use mpu9250_dmp::Mpu9250;
use std::thread;
use std::time::Duration;

pub fn main() {
    let matches = App::new("MPU-9250 Sample Scanner")
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
        .get_matches();
    let i2c_bus = value_t!(matches, "bus", i32).unwrap_or(1);
    let i2c_addr = value_t!(matches, "addr", u16).ok();
    let whoami = value_t!(matches, "whoami", u8).ok();

    let mut mpu =
        Mpu9250::new(i2c_bus, i2c_addr, whoami, None).expect("Could not connect to MPU-9250");
    loop {
        println!("{:#?}", mpu.read_sample().unwrap());
        thread::sleep(Duration::from_millis(10));
    }
}
