#pragma once

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

enum sensor_err {
    SENSOR_OK,
    SENSOR_ERR_I2C,
};

typedef enum sensor_err sensor_err;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

typedef float f32;
typedef double f64;

sensor_err read_block(I2C_HandleTypeDef* i2c, u8 addr, u8 reg, u8 size, u8* buf,
                      u8 bufsize);
sensor_err read_byte(I2C_HandleTypeDef* i2c, u8 addr, u8 reg, u8 size, u8* buf);
