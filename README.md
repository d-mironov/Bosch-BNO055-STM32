# Library for Bosch BNO055 IMU Sensor Unit written in and for the STM32 HAL.

## Installation
Just throw the `bno055.c`, `bno055.h` and `bno_config.h` into your project, and you are good to go.

## Usage
You first need to initialize the I2C yourself and enable the clock.  
There is also an [`examples`](`/examples`) folder where you can find the I2C init code and all the other 
necessary stuff to understand how to use this libary.  

Simple Init and Gyroscope and Accelerometer Read:
```c

// `bno` is a global variable
bno = (bno055_t){
    .i2c = &hi2c1, .addr = BNO_ADDR, .mode = BNO_MODE_IMU,
};

// Initialize the sensor
bno055_init(&bno);

bno055_vec3_t gyroscope;
bno055_vec3_t accelerometer;

while (true) {
    // Read Gyroscope data on all axis
    bno055_gyro(&bno, &gyroscope);
    // OR
    bno.gyro(&bno, &gyroscope);

    // Read Accelerometer data on all axis
    bno055_acc(&bno, &accelerometer);
    // OR
    bno.acc(&bno, &accelerometer);
}
```

You can also read every axis separately. For any sensor.
```c
f32 gyro_x, gyro_y, gyro_z;
while (true) {
    bno055_gyro_x(&bno, &gyro_x);
    bno055_gyro_y(&bno, &gyro_y);
    bno055_gyro_z(&bno, &gyro_z);
}
```

You can also set the units for each sensor:
```c
// Temperature sensor in °C, Gyroscope in Deg/s, Accelerometer in m/s^2, Euler in Deg
bno055_set_unit(&bno, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_DPS,
                      BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_DEG);
```

You can read the Quaternion and Euler data:
```c
bno055_euler_t euler;
bno055_euler(&bno, &euler);
// OR
bno.euler(&bno, &euler);
printf("%2.2f | %2.2f | %2.2f\n", euler.yaw, euler.pitch, euler.roll);

bno055_vec4_t qua;
bno055_quaternion(&bno, &qua);
// OR
bno.quaternion(&bno, &qua);
printf("%2.2f | %2.2f | %2.2f | %2.2f\n", qua.w, qua.x, qua.y, qua.z);
```
They can also be read separately.  

For all the other features, take a look at the Wiki Page of this repository.  

## Planned features
- Axis remap
- Interrupt based approach
- DMA based approach
- Low memory footprint config
- Self-test
- Calibration

All of the planned features listed here will be implemented once I have the time to do so. For now just stay tuned.

## Disclaimer

This is not really production ready since it is not stress-tested and there is no interrupt- or DMA-based read/write cycle implemented. More of this will come in the future.


## Contact
Discord: moonxraccoon#4788
E-Mail: [danielmironow@pm.me](mailto:danielmironow@pm.me)
