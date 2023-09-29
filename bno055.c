#include <stdint.h>
#include <stdio.h>
#include <stm32f4xx.h>

#include "bno055.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"

/**
 * Initialize the Bosch BNO055 Sensor
 * ---------------------------------------------------------------
 *
 *  Initializes the Bosch BNO055 Sensor with
 *  the specified operation mode.
 * ---------------------------------------------------------------
 *
 *  Args:
 *  > `imu[bno055_t*]`: pointer to the structure to initialize
 * ---------------------------------------------------------------
 *
 *  Return:
 *  > `bool`: `true` if none of the init steps fail, `false` else
 * ---------------------------------------------------------------
 */
error_bno bno055_init(bno055_t* imu) {
    u8 id = 0;
    error_bno err;

    imu->addr = (imu->addr << 1);
    err = bno055_read_regs(*imu, BNO_CHIP_ID, &id, 1);
    if (err != BNO_OK) {
        return err;
    }
    if (id != BNO_DEF_CHIP_ID) {
        return BNO_ERR_WRONG_CHIP_ID;
    }
    if ((err = bno055_set_opmode(imu, BNO_MODE_CONFIG)) != BNO_OK) {
        return err;
    }
    HAL_Delay(2);
    bno055_reset(imu);
    HAL_Delay(5000);
    if ((err = bno055_set_pwr_mode(imu, BNO_PWR_NORMAL)) != BNO_OK) {
        return err;
    }
    HAL_Delay(10);
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
    HAL_Delay(BNO_CONFIG_TIME_DELAY + 5);
    bno055_on(imu);
    if ((err = bno055_set_opmode(imu, imu->mode)) != BNO_OK) {
        return err;
    }
    HAL_Delay(BNO_ANY_TIME_DELAY + 5);

    imu->temperature = &bno055_temperature;
    imu->acc_x = &bno055_acc_x;
    imu->acc_y = &bno055_acc_y;
    imu->acc_z = &bno055_acc_z;
    imu->acc = &bno055_acc;
    imu->linear_acc_x = &bno055_linear_acc_x;
    imu->linear_acc_y = &bno055_linear_acc_y;
    imu->linear_acc_z = &bno055_linear_acc_z;
    imu->linear_acc = &bno055_linear_acc;
    imu->gyro_x = &bno055_gyro_x;
    imu->gyro_y = &bno055_gyro_y;
    imu->gyro_z = &bno055_gyro_z;
    imu->gyro = &bno055_gyro;
    imu->mag_x = &bno055_mag_x;
    imu->mag_y = &bno055_mag_y;
    imu->mag_z = &bno055_mag_z;
    imu->mag = &bno055_mag;
    imu->gravity_x = &bno055_gravity_x;
    imu->gravity_y = &bno055_gravity_y;
    imu->gravity_z = &bno055_gravity_z;
    imu->gravity = &bno055_gravity;
    imu->euler_yaw = &bno055_euler_yaw;
    imu->euler_roll = &bno055_euler_roll;
    imu->euler_pitch = &bno055_euler_pitch;
    imu->euler = &bno055_euler;
    imu->quaternion_w = &bno055_quaternion_w;
    imu->quaternion_x = &bno055_quaternion_x;
    imu->quaternion_y = &bno055_quaternion_y;
    imu->quaternion_z = &bno055_quaternion_z;
    imu->quaternion = &bno055_quaternion;

    imu->acc_config = &bno055_acc_conf;
    imu->gyr_config = &bno055_gyr_conf;
    imu->mag_config = &bno055_mag_conf;
    return BNO_OK;
}

/**
 * Bosch BNO055 read temperature function
 * ----------------------------------------------------------------------
 *
 * Read the temperature of the Bosch BNO055 Sensor and store it in a
 * specified buffer.
 * Unit can be specified with `bno055_set_unit(...)`. (Default: Celsius)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the temperature from
 *     > `buf[s8*|int8_t*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, `BNO_ERR_X` else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_temperature(bno055_t* imu, s8* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data = 0;
    if ((err = bno055_read_regs(*imu, BNO_TEMP, &data, 1)) != BNO_OK) {
        return err;
    }
    *buf = (imu->_temp_unit) ? data * 2 : data;
    return BNO_OK;
}

/**
 * Bosch BNO055 read accelerometer data on X-axis
 * ----------------------------------------------------------------------
 *
 * Reads the accelerometer data and stores the value in the specified buffer.
 * Unit can be specified with `bno055_set_unit(...)`. (Default: m/s^2)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_acc_x(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_ACC_DATA_X_LSB, data, 2)) != BNO_OK) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) /
           ((imu->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
                                                     : BNO_ACC_SCALE_MG);
    return BNO_OK;
};

/**
 * Bosch BNO055 read accelerometer data on Y-axis
 * ----------------------------------------------------------------------
 *
 * Reads the accelerometer data and stores the value in the specified buffer.
 * Unit can be specified with `bno055_set_unit(...)`. (Default: m/s^2)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_acc_y(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_ACC_DATA_Y_LSB, data, 2)) != BNO_OK) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) /
           ((imu->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
                                                     : BNO_ACC_SCALE_MG);
    return BNO_OK;
};

/**
 * Bosch BNO055 read accelerometer data on Z-axis
 * ----------------------------------------------------------------------
 *
 * Reads the accelerometer data and stores the value in the specified buffer.
 * Unit can be specified with `bno055_set_unit(...)`. (Default: m/s^2)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_acc_z(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_ACC_DATA_Z_LSB, data, 2)) != BNO_OK) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) /
           ((imu->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
                                                     : BNO_ACC_SCALE_MG);
    return BNO_OK;
};

/**
 * Bosch BNO055 read acceleration data on all axis
 * ----------------------------------------------------------------------
 *
 * Reads the acceleration data and stores the value in the specified
 * vector buffer. Unit can be specified with `bno055_set_unit(...)`. (Default:
 * m/s^2)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[bno055_vec3_t*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_acc(bno055_t* imu, bno055_vec3_t* xyz) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[6];
    if ((err = bno055_read_regs(*imu, BNO_ACC_DATA_X_LSB, data, 6)) != BNO_OK) {
        return err;
    }
    float scale = (imu->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
                                                           : BNO_ACC_SCALE_MG;
    xyz->x = (s16)((data[1] << 8) | data[0]) / scale;
    xyz->y = (s16)((data[3] << 8) | data[2]) / scale;
    xyz->z = (s16)((data[5] << 8) | data[4]) / scale;

    return BNO_OK;
};

/**
 * Bosch BNO055 read linear acceleration data on X-axis
 * ----------------------------------------------------------------------
 *
 * Reads the linear acceleration data and stores the value in the specified
 * buffer. Unit can be specified with `bno055_set_unit(...)`. (Default: m/s^2)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_linear_acc_x(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_LIA_DATA_X_LSB, data, 2)) != BNO_OK) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) /
           ((imu->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
                                                     : BNO_ACC_SCALE_MG);
    return BNO_OK;
};

/**
 * Bosch BNO055 read linear acceleration data on Y-axis
 * ----------------------------------------------------------------------
 *
 * Reads the linear acceleration data and stores the value in the specified
 * buffer. Unit can be specified with `bno055_set_unit(...)`. (Default: m/s^2)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_linear_acc_y(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_LIA_DATA_Y_LSB, data, 2)) != BNO_OK) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) /
           ((imu->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
                                                     : BNO_ACC_SCALE_MG);
    return BNO_OK;
};

/**
 * Bosch BNO055 read linear acceleration data on Z-axis
 * ----------------------------------------------------------------------
 *
 * Reads the linear acceleration data and stores the value in the specified
 * buffer. Unit can be specified with `bno055_set_unit(...)`. (Default: m/s^2)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_linear_acc_z(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_LIA_DATA_Z_LSB, data, 2)) != BNO_OK) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) /
           ((imu->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
                                                     : BNO_ACC_SCALE_MG);
    return BNO_OK;
};

/**
 * Bosch BNO055 read linear acceleration data on all axis
 * ----------------------------------------------------------------------
 *
 * Reads the linear acceleration data and stores the value in the specified
 * vector buffer. Unit can be specified with `bno055_set_unit(...)`. (Default:
 * m/s^2)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[bno055_vec3_t*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_linear_acc(bno055_t* imu, bno055_vec3_t* xyz) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[6];
    if ((err = bno055_read_regs(*imu, BNO_LIA_DATA_X_LSB, data, 6)) != BNO_OK) {
        return err;
    }
    float scale = (imu->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
                                                           : BNO_ACC_SCALE_MG;
    xyz->x = (s16)((data[1] << 8) | data[0]) / scale;
    xyz->y = (s16)((data[3] << 8) | data[2]) / scale;
    xyz->z = (s16)((data[5] << 8) | data[4]) / scale;

    return BNO_OK;
};

/**
 * Bosch BNO055 read gyroscope data on X-axis
 * ----------------------------------------------------------------------
 *
 * Reads the gyroscope data and stores the value in the specified
 * buffer. Unit can be specified with `bno055_set_unit(...)`. (Default: m/s^2)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_gyro_x(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_GYR_DATA_X_LSB, data, 2)) != BNO_OK) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) /
           ((imu->_gyr_unit == BNO_GYR_UNIT_DPS) ? BNO_GYR_SCALE_DPS
                                                 : BNO_GYR_SCALE_RPS);
    return BNO_OK;
};

/**
 * Bosch BNO055 read gyroscope data on Y-axis
 * ----------------------------------------------------------------------
 *
 * Reads the gyroscope data and stores the value in the specified
 * buffer. Unit can be specified with `bno055_set_unit(...)`. (Default: m/s^2)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_gyro_y(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_GYR_DATA_Y_LSB, data, 2)) != BNO_OK) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) /
           ((imu->_gyr_unit == BNO_GYR_UNIT_DPS) ? BNO_GYR_SCALE_DPS
                                                 : BNO_GYR_SCALE_RPS);
    return BNO_OK;
};

/**
 * Bosch BNO055 read gyroscope data on Z-axis
 * ----------------------------------------------------------------------
 *
 * Reads the gyroscope data and stores the value in the specified
 * buffer. Unit can be specified with `bno055_set_unit(...)`. (Default: m/s^2)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_gyro_z(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_GYR_DATA_Z_LSB, data, 2)) != BNO_OK) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) /
           ((imu->_gyr_unit == BNO_GYR_UNIT_DPS) ? BNO_GYR_SCALE_DPS
                                                 : BNO_GYR_SCALE_RPS);
    return BNO_OK;
};

/**
 * Bosch BNO055 read gyroscope data on all axis
 * ----------------------------------------------------------------------
 *
 * Reads the gyroscope data and stores the value in the specified
 * vector buffer. Unit can be specified with `bno055_set_unit(...)`. (Default:
 * m/s^2)
 *
 * FIX: MCU restart when moving
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[bno055_vec3_t*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_gyro(bno055_t* imu, bno055_vec3_t* xyz) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[6];
    if ((err = bno055_read_regs(*imu, BNO_GYR_DATA_X_LSB, data, 6)) != BNO_OK) {
        return err;
    }

    f32 scale = (imu->_gyr_unit == BNO_GYR_UNIT_DPS) ? BNO_GYR_SCALE_DPS
                                                     : BNO_GYR_SCALE_RPS;
    xyz->x = (s16)((data[1] << 8) | data[0]) / scale;
    xyz->y = (s16)((data[3] << 8) | data[2]) / scale;
    xyz->z = (s16)((data[5] << 8) | data[4]) / scale;

    return BNO_OK;
}

/**
 * Bosch BNO055 read magnetometer data on X-axis
 * ----------------------------------------------------------------------
 *
 * Reads the magnetometer data and stores the value in the specified
 * buffer. Unit is `[uT]`.

 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_mag_x(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_MAG_DATA_X_LSB, data, 2)) != BNO_OK) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) / BNO_MAG_SCALE;
    return BNO_OK;
}

/**
 * Bosch BNO055 read magnetometer data on Y-axis
 * ----------------------------------------------------------------------
 *
 * Reads the magnetometer data and stores the value in the specified
 * buffer. Unit is `[uT]`.

 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_mag_y(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_MAG_DATA_Y_LSB, data, 2)) != BNO_OK) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) / BNO_MAG_SCALE;
    return BNO_OK;
}

/**
 * Bosch BNO055 read magnetometer data on Z-axis
 * ----------------------------------------------------------------------
 *
 * Reads the magnetometer data and stores the value in the specified
 * buffer. Unit is `[uT]`.
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_mag_z(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_MAG_DATA_Z_LSB, data, 2)) != BNO_OK) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) / BNO_MAG_SCALE;
    return BNO_OK;
}

/**
 * Bosch BNO055 read magnetometer data on all axis
 * ----------------------------------------------------------------------
 *
 * Reads the magnetometer data and stores the value in the specified vector
 * buffer. Unit is `[uT]`.
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[bno055_vec3_t*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_mag(bno055_t* imu, bno055_vec3_t* xyz) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[6];
    if ((err = bno055_read_regs(*imu, BNO_MAG_DATA_X_LSB, data, 6)) != BNO_OK) {
        return err;
    }

    xyz->x = (s16)((data[1] << 8) | data[0]) / BNO_MAG_SCALE;
    xyz->y = (s16)((data[3] << 8) | data[2]) / BNO_MAG_SCALE;
    xyz->z = (s16)((data[5] << 8) | data[4]) / BNO_MAG_SCALE;

    return BNO_OK;
};

/**
 * Bosch BNO055 read gravity data on X-axis
 * ----------------------------------------------------------------------
 *
 * Reads the gravity data and stores the value in the specified
 * buffer. Unit can be specified with `bno055_set_unit(...)`. (Default: m/s^2)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_gravity_x(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_GRV_DATA_X_LSB, data, 2)) != BNO_OK) {
        return err;
    }
    f32 scale = (imu->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
                                                         : BNO_ACC_SCALE_MG;
    *buf = (s16)((data[1] << 8) | data[0]) / scale;
    return BNO_OK;
}

/**
 * Bosch BNO055 read gravity data on Y-axis
 * ----------------------------------------------------------------------
 *
 * Reads the gravity data and stores the value in the specified
 * buffer. Unit can be specified with `bno055_set_unit(...)`. (Default: m/s^2)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_gravity_y(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_GRV_DATA_Y_LSB, data, 2)) != BNO_OK) {
        return err;
    }
    f32 scale = (imu->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
                                                         : BNO_ACC_SCALE_MG;
    *buf = (s16)((data[1] << 8) | data[0]) / scale;
    return BNO_OK;
}

/**
 * Bosch BNO055 read gravity data on Z-axis
 * ----------------------------------------------------------------------
 *
 * Reads the gravity data and stores the value in the specified
 * buffer. Unit can be specified with `bno055_set_unit(...)`. (Default: m/s^2)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_gravity_z(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_GRV_DATA_Z_LSB, data, 2)) != BNO_OK) {
        return err;
    }
    f32 scale = (imu->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
                                                         : BNO_ACC_SCALE_MG;
    *buf = (s16)((data[1] << 8) | data[0]) / scale;
    return BNO_OK;
}

/**
 * Bosch BNO055 read gravity data on all axis
 * ----------------------------------------------------------------------
 *
 * Reads the gravity data and stores the value in the specified
 * vector buffer. Unit can be selected with `bno055_set_unit(...)` (default:
 * m/s^2).
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[bno055_vec3_t*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_gravity(bno055_t* imu, bno055_vec3_t* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[6];
    if ((err = bno055_read_regs(*imu, BNO_GRV_DATA_X_LSB, data, 6)) != BNO_OK) {
        return err;
    }
    f32 scale = (imu->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
                                                         : BNO_ACC_SCALE_MG;
    buf->x = (s16)((data[1] << 8) | data[0]) / scale;
    buf->y = (s16)((data[3] << 8) | data[2]) / scale;
    buf->x = (s16)((data[5] << 8) | data[4]) / scale;
    return BNO_OK;
}

/**
 * Bosch BNO055 read euler yaw data
 * ----------------------------------------------------------------------
 *
 * Reads the yaw data and stores the value in the specified
 * buffer. Unit can be specified with `bno055_set_unit(...)`. (Default: deg)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_euler_yaw(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_EUL_HEADING_LSB, data, 2)) !=
        BNO_OK) {
        return err;
    }
    f32 scale = (imu->_eul_unit == BNO_EUL_UNIT_DEG) ? BNO_EUL_SCALE_DEG
                                                     : BNO_EUL_SCALE_RAD;
    *buf = (s16)((data[1] << 8) | data[0]) / scale;
    return BNO_OK;
}

/**
 * Bosch BNO055 read euler roll data
 * ----------------------------------------------------------------------
 *
 * Reads the roll data and stores the value in the specified
 * buffer. Unit can be specified with `bno055_set_unit(...)`. (Default: deg)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_euler_roll(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_EUL_ROLL_LSB, data, 2)) != BNO_OK) {
        return err;
    }
    f32 scale = (imu->_eul_unit == BNO_EUL_UNIT_DEG) ? BNO_EUL_SCALE_DEG
                                                     : BNO_EUL_SCALE_RAD;
    *buf = (s16)((data[1] << 8) | data[0]) / scale;
    return BNO_OK;
}

/**
 * Bosch BNO055 read euler pitch data
 * ----------------------------------------------------------------------
 *
 * Reads the pitch data and stores the value in the specified
 * buffer. Unit can be specified with `bno055_set_unit(...)`. (Default: deg)
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_euler_pitch(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_EUL_PITCH_LSB, data, 2)) != BNO_OK) {
        return err;
    }
    f32 scale = (imu->_eul_unit == BNO_EUL_UNIT_DEG) ? BNO_EUL_SCALE_DEG
                                                     : BNO_EUL_SCALE_RAD;
    *buf = (s16)((data[1] << 8) | data[0]) / scale;
    return BNO_OK;
}

/**
 * Bosch BNO055 read euler data on all axis
 * ----------------------------------------------------------------------
 *
 * Reads the euler data and stores the value in the specified
 * vector buffer. Unit can be selected with `bno055_set_unit(...)` (default:
 * deg).
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[bno055_euler_t*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_euler(bno055_t* imu, bno055_euler_t* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[6];
    if ((err = bno055_read_regs(*imu, BNO_EUL_HEADING_LSB, data, 6)) !=
        BNO_OK) {
        return err;
    }
    f32 scale = (imu->_eul_unit == BNO_EUL_UNIT_DEG) ? BNO_EUL_SCALE_DEG
                                                     : BNO_EUL_SCALE_RAD;
    buf->yaw = (s16)((data[1] << 8) | data[0]) / scale;
    buf->roll = (s16)((data[3] << 8) | data[2]) / scale;
    buf->pitch = (s16)((data[5] << 8) | data[4]) / scale;
    return BNO_OK;
}

/**
 * Bosch BNO055 read quaternion W-axis data
 * ----------------------------------------------------------------------
 *
 * Reads the quaternion data and stores the value in the specified
 * buffer.
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_quaternion_w(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_QUA_DATA_W_LSB, data, 2)) != BNO_OK) {
        return err;
    }
    *buf = (s16)((data[1] << 8) | data[0]) / (f32)BNO_QUA_SCALE;
    return BNO_OK;
}

/**
 * Bosch BNO055 read quaternion X-axis data
 * ----------------------------------------------------------------------
 *
 * Reads the quaternion data and stores the value in the specified
 * buffer.
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_quaternion_x(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_QUA_DATA_X_LSB, data, 2)) != BNO_OK) {
        return err;
    }
    *buf = (s16)((data[1] << 8) | data[0]) / (f32)BNO_QUA_SCALE;
    return BNO_OK;
}

/**
 * Bosch BNO055 read quaternion Y-axis data
 * ----------------------------------------------------------------------
 *
 * Reads the quaternion data and stores the value in the specified
 * buffer.
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_quaternion_y(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_QUA_DATA_Y_LSB, data, 2)) != BNO_OK) {
        return err;
    }
    *buf = (s16)((data[1] << 8) | data[0]) / (f32)BNO_QUA_SCALE;
    return BNO_OK;
}

/**
 * Bosch BNO055 read quaternion Z-axis data
 * ----------------------------------------------------------------------
 *
 * Reads the quaternion data and stores the value in the specified
 * buffer.
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[f32*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_quaternion_z(bno055_t* imu, f32* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = bno055_read_regs(*imu, BNO_QUA_DATA_Z_LSB, data, 2)) != BNO_OK) {
        return err;
    }
    *buf = (s16)((data[1] << 8) | data[0]) / (f32)BNO_QUA_SCALE;
    return BNO_OK;
}

/**
 * Bosch BNO055 read quaternion data on all axis
 * ----------------------------------------------------------------------
 *
 * Reads the euler data and stores the value in the specified
 * vector buffer. Unit can be selected with `bno055_set_unit(...)` (default:
 * deg).
 * ----------------------------------------------------------------------
 *
 * Args:
 *     > `imu[bno055_t*]`: BNO055 to read the data from
 *     > `buf[bno055_euler_t*]`: Buffer to store the read value in
 * ----------------------------------------------------------------------
 *
 * Return:
 *     > `error_bno`: `BNO_OK` on success, errorcode else.
 * ----------------------------------------------------------------------
 */
error_bno bno055_quaternion(bno055_t* imu, bno055_vec4_t* buf) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[8];
    if ((err = bno055_read_regs(*imu, BNO_QUA_DATA_W_LSB, data, 8)) != BNO_OK) {
        return err;
    }
    buf->w = (s16)((data[1] << 8) | data[0]) / (f32)BNO_QUA_SCALE;
    buf->x = (s16)((data[3] << 8) | data[2]) / (f32)BNO_QUA_SCALE;
    buf->y = (s16)((data[5] << 8) | data[4]) / (f32)BNO_QUA_SCALE;
    buf->z = (s16)((data[7] << 8) | data[6]) / (f32)BNO_QUA_SCALE;
    return BNO_OK;
}

error_bno bno055_acc_conf(bno055_t* bno, const bno055_acc_range_t range,
                          const bno055_acc_band_t bandwidth,
                          const bno055_acc_mode_t mode) {
    error_bno err;
    if ((err = bno055_set_page(bno, BNO_PAGE_1)) != BNO_OK) {
        return err;
    }
    if ((err = bno055_set_opmode(bno, BNO_MODE_CONFIG)) != BNO_OK) {
        return err;
    }
    HAL_Delay(BNO_CONFIG_TIME_DELAY + 5);
    u8 config = range | bandwidth | mode;
    if ((err = bno055_write_regs(*bno, BNO_ACC_CONFIG, &config, 1)) != BNO_OK) {
        return err;
    }
    if ((err = bno055_set_opmode(bno, bno->mode)) != BNO_OK) {
        return err;
    }
    HAL_Delay(BNO_ANY_TIME_DELAY + 5);
    if ((err = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
    return BNO_OK;
}

error_bno bno055_gyr_conf(bno055_t* bno, const bno055_gyr_range_t range,
                          const bno055_gyr_band_t bandwidth,
                          const bno055_gyr_mode_t mode) {
    error_bno err;
    if ((err = bno055_set_page(bno, BNO_PAGE_1)) != BNO_OK) {
        return err;
    }
    if ((err = bno055_set_opmode(bno, BNO_MODE_CONFIG)) != BNO_OK) {
        return err;
    }
    HAL_Delay(BNO_CONFIG_TIME_DELAY + 5);
    u8 config[2] = {range | bandwidth, mode};
    if ((err = bno055_write_regs(*bno, BNO_GYR_CONFIG_0, config, 2)) !=
        BNO_OK) {
        return err;
    }
    if ((err = bno055_set_opmode(bno, bno->mode)) != BNO_OK) {
        return err;
    }
    HAL_Delay(BNO_ANY_TIME_DELAY + 5);
    if ((err = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
    return BNO_OK;
}
error_bno bno055_mag_conf(bno055_t* bno, const bno055_mag_rate_t out_rate,
                          const bno055_mag_pwr_t pwr_mode,
                          const bno055_mag_mode_t mode) {
    error_bno err;
    if ((err = bno055_set_page(bno, BNO_PAGE_1)) != BNO_OK) {
        return err;
    }
    if ((err = bno055_set_opmode(bno, BNO_MODE_CONFIG)) != BNO_OK) {
        return err;
    }
    HAL_Delay(BNO_CONFIG_TIME_DELAY + 5);
    u8 config = out_rate | pwr_mode | mode;
    if ((err = bno055_write_regs(*bno, BNO_MAG_CONFIG, &config, 1)) != BNO_OK) {
        return err;
    }
    if ((err = bno055_set_opmode(bno, bno->mode)) != BNO_OK) {
        return err;
    }
    HAL_Delay(BNO_ANY_TIME_DELAY + 5);
    if ((err = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
    return BNO_OK;
}

error_bno bno055_set_opmode(bno055_t* imu, const bno055_opmode_t opmode) {
    error_bno err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    if ((err = bno055_write_regs(*imu, BNO_OPR_MODE, (u8*)&opmode, 1)) !=
        BNO_OK) {
        return err;
    }
    HAL_Delay(BNO_ANY_TIME_DELAY + 5);
    return BNO_OK;
}

error_bno bno055_set_unit(bno055_t* bno, const bno055_temp_unitsel_t t_unit,
                          const bno055_gyr_unitsel_t g_unit,
                          const bno055_acc_unitsel_t a_unit,
                          const bno055_eul_unitsel_t e_unit) {
    error_bno err;
    if ((err = bno055_set_opmode(bno, BNO_MODE_CONFIG)) != BNO_OK) {
        return err;
    }
    if ((err = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
    uint8_t data = t_unit | g_unit | a_unit | e_unit;
    if ((err = bno055_write_regs(*bno, BNO_UNIT_SEL, &data, 1)) != BNO_OK) {
        return err;
    }
    bno->_gyr_unit = g_unit;
    bno->_acc_unit = a_unit;
    bno->_eul_unit = e_unit;
    bno->_temp_unit = t_unit;

    if ((err = bno055_set_opmode(bno, bno->mode)) != BNO_OK) {
        return err;
    }
    return BNO_OK;
}

error_bno bno055_set_pwr_mode(bno055_t* imu, bno055_pwr_t pwr_mode) {
    if (imu == NULL) {
        return BNO_ERR_NULL_PTR;
    }
    error_bno err;
    if ((err = bno055_set_opmode(imu, BNO_MODE_CONFIG)) != BNO_OK) {
        return err;
    }
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
    if ((err = bno055_write_regs(*imu, BNO_PWR_MODE, (u8*)&pwr_mode, 1)) !=
        BNO_OK) {
        return err;
    }
    imu->_pwr_mode = pwr_mode;
    if ((err = bno055_set_page(imu, BNO_PAGE_0)) != BNO_OK) {
        return err;
    }
    if ((err = bno055_set_opmode(imu, imu->mode)) != BNO_OK) {
        return err;
    }
    HAL_Delay(2);
    return BNO_OK;
}

error_bno bno055_reset(bno055_t* imu) {
    u8 data = 0x20U;
    if (bno055_write_regs(*imu, BNO_SYS_TRIGGER, &data, 1) != BNO_OK) {
        return BNO_ERR_I2C;
    }
    return BNO_OK;
}

error_bno bno055_on(bno055_t* imu) {
    u8 data = 0x00U;
    if (bno055_write_regs(*imu, BNO_SYS_TRIGGER, &data, 1) != BNO_OK) {
        return BNO_ERR_I2C;
    }
    return BNO_OK;
}

error_bno bno055_read_regs(bno055_t imu, u8 addr, uint8_t* buf,
                           uint32_t buf_size) {
    HAL_StatusTypeDef err;
    // err = HAL_I2C_Mem_Read(imu.i2c, imu.addr, addr, I2C_MEMADD_SIZE_8BIT,
    // buf,
    //                        buf_size, HAL_MAX_DELAY);
    err = HAL_I2C_Master_Transmit(imu.i2c, imu.addr, &addr, 1, HAL_MAX_DELAY);
    if (err != HAL_OK) {
        return BNO_ERR_I2C;
    }
    err =
        HAL_I2C_Master_Receive(imu.i2c, imu.addr, buf, buf_size, HAL_MAX_DELAY);
    if (err != HAL_OK) {
        return BNO_ERR_I2C;
    }
    return BNO_OK;
}

error_bno bno055_write_regs(bno055_t imu, uint32_t addr, uint8_t* buf,
                            uint32_t buf_size) {
    HAL_StatusTypeDef err;
    err = HAL_I2C_Mem_Write(imu.i2c, imu.addr, addr, buf_size, buf, buf_size,
                            HAL_MAX_DELAY);
    if (err != HAL_OK) {
        return BNO_ERR_I2C;
    }
    return BNO_OK;
}

error_bno bno055_set_page(bno055_t* imu, const bno055_page_t page) {
    if (imu->_page != page) {
        return BNO_OK;
    }
    if (page > 0x01) {
        return BNO_ERR_PAGE_TOO_HIGH;
    }
    error_bno err;
    err = bno055_write_regs(*imu, BNO_PAGE_ID, (u8*)&page, 1);
    if (err != BNO_OK) {
        return err;
    }
    imu->_page = page;
    HAL_Delay(2);
    return BNO_OK;
}

char* bno055_err_str(const error_bno err) {
    switch (err) {
        case BNO_OK:
            return "[BNO] Ok!";
        case BNO_ERR_I2C:
            return "[BNO] I2C error!";
        case BNO_ERR_PAGE_TOO_HIGH:
            return "[BNO] Page setting to high.";
        case BNO_ERR_NULL_PTR:
            return "[BNO] BNO struct is nullpointer.";
        case BNO_ERR_AXIS_REMAP:
            return "[BNO] Axis remap error!";
        case BNO_ERR_SETTING_PAGE:
            return "[BNO] TODO";
        case BNO_ERR_WRONG_CHIP_ID:
            return "[BNO] Wrong Chip ID.";
    }
    return "[BNO] Ok!";
}
