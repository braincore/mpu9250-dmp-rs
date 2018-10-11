# MPU-9250 Library for Rust [![Latest Version]][crates.io] [![Documentation]][docs.rs]

[Latest Version]: https://img.shields.io/crates/v/mpu9250-dmp.svg
[crates.io]: https://crates.io/crates/mpu9250-dmp
[Documentation]: https://docs.rs/mpu9250-dmp/badge.svg
[docs.rs]: https://docs.rs/mpu9250-dmp

A library for the MPU-9250 9-axis IMU. Only supports the i2c interface (no
SPI). Includes a minimal interface to the onboard Digital Motion Processor.

## Features

* Adjustable full scale range (2G..16G) and low-pass-filter bandwidth.
* Adjustable sample rate.
* Adjustable fifo population rate.
* Setup of DMP interrupt and fifo.
* Quaternion output from the DMP.

## Note: Heading Estimation

Because the quaternions are computed from a 6-axis measurement (accelerometer
and gyroscope, no magnetometer), it suffers from significant yaw drift. If you
require accurate heading, it's best to ignore the quaternion output and instead
use a separate 9-axis filter such as Madgwick.

## Note: Crate Quality

This crate was built to do performance testing on the MPU-9250 after which
usage of this crate was halted. The code isn't under active development and
hasn't been cleaned up. Because there are few examples of using the DMP out
there I thought it would still be valuable to share this with others.

## Usage

See `examples/scan.rs` and `examples/scan_dmp.rs`. Use `-h` to see parameters
for i2c, sample rate, and interrupt pin.

## Testing

By default, uses i2c `bus=1` and `addr=0x68`, and expects a `WHOAMI` register
value of `0x71`. To override, use these environment variables:

```
MPU9250_I2C_BUS=1 MPU9250_I2C_ADDR=104 MPU9250_WHOAMI=115 cargo test
```

## Resources

* [Product Specification v1.1](https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)
* [Datasheet v1.5](https://www.invensense.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf)

## Todo

There are no plans to do these.

- [ ] Expose temperature readings in `Mpu9250Sample`.
- [ ] DMP configurations such as orientation and tap detection.
