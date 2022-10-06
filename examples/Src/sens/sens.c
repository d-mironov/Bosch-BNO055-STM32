#include "sens.h"

/**
 * TODO: add I2C error codes from HAL
 */
sensor_err read_block(I2C_HandleTypeDef* i2c, u8 addr, u8 reg, u8 size, u8* buf,
                      u8 bufsize) {
    HAL_StatusTypeDef err;
    err = HAL_I2C_Master_Transmit(i2c, addr, &reg, 1, HAL_MAX_DELAY);
    if (err != HAL_OK) {
        return SENSOR_ERR_I2C;
    }
    err = HAL_I2C_Master_Receive(i2c, addr, buf, bufsize, HAL_MAX_DELAY);
    if (err != HAL_OK) {
        return SENSOR_ERR_I2C;
    }
    return SENSOR_OK;
}

sensor_err read_byte(I2C_HandleTypeDef* i2c, u8 addr, u8 reg, u8 size,
                     u8* buf) {
    HAL_StatusTypeDef err;
    err = HAL_I2C_Master_Transmit(i2c, addr, &reg, 1, HAL_MAX_DELAY);
    if (err != HAL_OK) {
        return SENSOR_ERR_I2C;
    }
    err = HAL_I2C_Master_Receive(i2c, addr, buf, 1, HAL_MAX_DELAY);
    if (err != HAL_OK) {
        return SENSOR_ERR_I2C;
    }
    return SENSOR_OK;
}
