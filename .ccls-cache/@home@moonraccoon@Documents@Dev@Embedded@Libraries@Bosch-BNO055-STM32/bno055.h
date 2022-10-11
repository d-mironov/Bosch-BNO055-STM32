/**
 * Copyright<2022> Daniel Mironow
 *
 * @file bno055.h
 *
 * @brief Bosch BNO055 driver written for the STM32 HAL
 * @author Daniel Mironow
 * Contact: danielmironow@pm.me
 */
#ifndef DRV_BNO055_H_
#define DRV_BNO055_H_

#include <stm32f4xx.h>

#include "stdbool.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_i2c_ex.h"

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

#define BNO_ADDR           (0x29)
#define BNO_ADDR_ALT       (0x28)
#define BNO_DEF_CHIP_ID    (0xA0)
//========================| Page 0 |========================
#define BNO_MAG_RADIUS_MSB (0x6A)
#define BNO_MAG_RADIUS_LSB (0x69)
#define BNO_ACC_RADIUS_MSB (0x68)
#define BNO_ACC_RADIUS_LSB (0x67)

// Gyroscope offset data
#define BNO_GYR_OFFSET_Z_MSB (0x66)
#define BNO_GYR_OFFSET_Z_LSB (0x65)
#define BNO_GYR_OFFSET_Y_MSB (0x64)
#define BNO_GYR_OFFSET_Y_LSB (0x63)
#define BNO_GYR_OFFSET_X_MSB (0x62)
#define BNO_GYR_OFFSET_X_LSB (0x61)

// magnetometer offset data
#define BNO_MAG_OFFSET_Z_MSB (0x60)
#define BNO_MAG_OFFSET_Z_LSB (0x5F)
#define BNO_MAG_OFFSET_Y_MSB (0x5E)
#define BNO_MAG_OFFSET_Y_LSB (0x5D)
#define BNO_MAG_OFFSET_X_MSB (0x5C)
#define BNO_MAG_OFFSET_X_LSB (0x5B)

// Accelerometer offset data
#define BNO_ACC_OFFSET_Z_MSB (0x5A)
#define BNO_ACC_OFFSET_Z_LSB (0x59)
#define BNO_ACC_OFFSET_Y_MSB (0x58)
#define BNO_ACC_OFFSET_Y_LSB (0x57)
#define BNO_ACC_OFFSET_X_MSB (0x56)
#define BNO_ACC_OFFSET_X_LSB (0x55)

// Axis remap
#define BNO_AXIS_MAP_SIGN   (0x42)
#define BNO_AXIS_MAP_CONFIG (0x41)

#define BNO_TEMP_SOURCE (0x40)
#define BNO_SYS_TRIGGER (0x3F)

// Power mode registers
#define BNO_PWR_MODE (0x3E)
#define BNO_OPR_MODE (0x3D)
#define BNO_UNIT_SEL (0x3B)

// status registers
#define BNO_SYS_ERR        (0x3A)
#define BNO_SYS_STATUS     (0x39)
#define BNO_SYS_CLK_STATUS (0x38)

#define BNO_INT_STA    (0x37)
#define BNO_ST_RESULT  (0x36)
#define BNO_CALIB_STAT (0x35)
#define BNO_TEMP       (0x34)

// Gravity Data
#define BNO_GRV_DATA_Z_MSB (0x33)
#define BNO_GRV_DATA_Z_LSB (0x32)
#define BNO_GRV_DATA_Y_MSB (0x31)
#define BNO_GRV_DATA_Y_LSB (0x30)
#define BNO_GRV_DATA_X_MSB (0x2F)
#define BNO_GRV_DATA_X_LSB (0x2E)

// Linear acceleration data
#define BNO_LIA_DATA_Z_MSB (0x2D)
#define BNO_LIA_DATA_Z_LSB (0x2C)
#define BNO_LIA_DATA_Y_MSB (0x2B)
#define BNO_LIA_DATA_Y_LSB (0x2A)
#define BNO_LIA_DATA_X_MSB (0x29)
#define BNO_LIA_DATA_X_LSB (0x28)

// Quaternion data
#define BNO_QUA_DATA_Z_MSB (0x27)
#define BNO_QUA_DATA_Z_LSB (0x26)
#define BNO_QUA_DATA_Y_MSB (0x25)
#define BNO_QUA_DATA_Y_LSB (0x24)
#define BNO_QUA_DATA_X_MSB (0x23)
#define BNO_QUA_DATA_X_LSB (0x22)
#define BNO_QUA_DATA_W_MSB (0x21)
#define BNO_QUA_DATA_W_LSB (0x20)

// Euler Angle data
#define BNO_EUL_PITCH_MSB   (0x1F)
#define BNO_EUL_PITCH_LSB   (0x1E)
#define BNO_EUL_ROLL_MSB    (0x1D)
#define BNO_EUL_ROLL_LSB    (0x1C)
#define BNO_EUL_HEADING_MSB (0x1B)
#define BNO_EUL_HEADING_LSB (0x1A)

// Gyroscope data
#define BNO_GYR_DATA_Z_MSB (0x19)
#define BNO_GYR_DATA_Z_LSB (0x18)
#define BNO_GYR_DATA_Y_MSB (0x17)
#define BNO_GYR_DATA_Y_LSB (0x16)
#define BNO_GYR_DATA_X_MSB (0x15)
#define BNO_GYR_DATA_X_LSB (0x14)

// Magnetometer data
#define BNO_MAG_DATA_Z_MSB (0x13)
#define BNO_MAG_DATA_Z_LSB (0x12)
#define BNO_MAG_DATA_Y_MSB (0x11)
#define BNO_MAG_DATA_Y_LSB (0x10)
#define BNO_MAG_DATA_X_MSB (0x0F)
#define BNO_MAG_DATA_X_LSB (0x0E)

// Accelerometer data
#define BNO_ACC_DATA_Z_MSB (0x0D)
#define BNO_ACC_DATA_Z_LSB (0x0C)
#define BNO_ACC_DATA_Y_MSB (0x0B)
#define BNO_ACC_DATA_Y_LSB (0x0A)
#define BNO_ACC_DATA_X_MSB (0x09)
#define BNO_ACC_DATA_X_LSB (0x08)

// Config Page ID
#define BNO_PAGE_ID       (0x07)
#define BNO_BL_REV_ID     (0x06)
#define BNO_SW_REV_ID_MSB (0x05)
#define BNO_SW_REV_ID_LSB (0x04)
#define BNO_GYR_ID        (0x03)
#define BNO_MAG_ID        (0x02)
#define BNO_ACC_ID        (0x01)
// Sensor ID
#define BNO_CHIP_ID       (0x00)
//==========================================================

//========================| Page 1 |========================
#define BNO_ACC_CONFIG       (0x08)
#define BNO_MAG_CONFIG       (0x09)
#define BNO_GYR_CONFIG_0     (0x0A)
#define BNO_GYR_CONFIG_1     (0x0B)
#define BNO_ACC_SLEEP_CONFIG (0x0C)
#define BNO_GYR_SLEEP_CONFIG (0x0D)
//==========================================================

#define BNO_CONFIG_TIME_DELAY 7   // ms
#define BNO_ANY_TIME_DELAY    19  // ms

// mag, acc, gyr, config offsets
#define BNO_ACC_BAND_OFFSET     (0x02)
#define BNO_ACC_MODE_OFFSET     (0x05)
#define BNO_GYR_BAND_OFFSET     (0x03)
#define BNO_GYR_MODE_OFFSET     (0x00)
#define BNO_MAG_MODE_OFFSET     (0x03)
#define BNO_MAG_PWRMODE_OFFSET  (0x05)
// unit selection
#define BNO_GYR_UNITSEL_OFFSET  (0x01)
#define BNO_EUL_UNITSEL_OFFSET  (0x02)
#define BNO_TEMP_UNITSEL_OFFSET (0x04)

#define BNO_ACC_SCALE_M_2 100.0f
#define BNO_ACC_SCALE_MG  1.0f

#define BNO_GYR_SCALE_DPS 16.0f
#define BNO_GYR_SCALE_RPS 900.0f

#define BNO_MAG_SCALE 16.0f

#define BNO_EUL_SCALE_DEG 16.0f
#define BNO_EUL_SCALE_RAD 900.0f

#define BNO_QUA_SCALE (0x4000)

typedef enum bno055_page {
    BNO_PAGE_0 = 0x00U,
    BNO_PAGE_1 = 0x01U,
} bno055_page_t;

typedef enum bno055_pwr {
    BNO_PWR_NORMAL,
    BNO_PWR_LOW,
    BNO_PWR_SUSPEND,
} bno055_pwr_t;

typedef enum bno055_acc_range {
    BNO_ACC_RANGE_2G,
    BNO_ACC_RANGE_4G,
    BNO_ACC_RANGE_8G,
    BNO_ACC_RANGE_16G,
} bno055_acc_range_t;

typedef enum bno055_acc_band {
    BNO_ACC_BAND_7_81 = (0 << BNO_ACC_BAND_OFFSET),
    BNO_ACC_BAND_15_63 = (1 << BNO_ACC_BAND_OFFSET),
    BNO_ACC_BAND_31_25 = (2 << BNO_ACC_BAND_OFFSET),
    BNO_ACC_BAND_62_5 = (3 << BNO_ACC_BAND_OFFSET),
    BNO_ACC_BAND_125 = (4 << BNO_ACC_BAND_OFFSET),
    BNO_ACC_BAND_250 = (5 << BNO_ACC_BAND_OFFSET),
    BNO_ACC_BAND_500 = (6 << BNO_ACC_BAND_OFFSET),
    BNO_ACC_BAND_1000 = (7 << BNO_ACC_BAND_OFFSET),
} bno055_acc_band_t;

typedef enum bno055_acc_mode {
    BNO_ACC_MODE_NORMAL = (0 << BNO_ACC_MODE_OFFSET),
    BNO_ACC_MODE_SUSPEND = (1 << BNO_ACC_MODE_OFFSET),
    BNO_ACC_MODE_LOW1 = (2 << BNO_ACC_MODE_OFFSET),
    BNO_ACC_MODE_STANDY = (3 << BNO_ACC_MODE_OFFSET),
    BNO_ACC_MODE_LOW2 = (4 << BNO_ACC_MODE_OFFSET),
    BNO_ACC_MODE_DEEP_SUSPEND = (5 << BNO_ACC_MODE_OFFSET),
} bno055_acc_mode_t;

typedef enum bno055_gyr_range {
    BNO_GYR_RANGE_2000_DPS,
    BNO_GYR_RANGE_1000_DPS,
    BNO_GYR_RANGE_500_DPS,
    BNO_GYR_RANGE_250_DPS,
    BNO_GYR_RANGE_125_DPS,
} bno055_gyr_range_t;

typedef enum bno055_gyr_band {
    BNO_GYR_BAND_523 = (0 << BNO_GYR_BAND_OFFSET),
    BNO_GYR_BAND_230 = (1 << BNO_GYR_BAND_OFFSET),
    BNO_GYR_BAND_116,
    BNO_GYR_BAND_47,
    BNO_GYR_BAND_23,
    BNO_GYR_BAND_12,
    BNO_GYR_BAND_64,
    BNO_GYR_BAND_32,
} bno055_gyr_band_t;

typedef enum bno055_gyr_mode {
    BNO_GYR_MODE_NORMAL,       /*!< Gyro normal mode */
    BNO_GYR_MODE_FPU,          /*!< Gyro fast powerup mode */
    BNO_GYR_MODE_DEEP_SUSPEND, /*!< Gyro deep suspend mode*/
    BNO_GYR_MODE_SUSPEND,      /*!< Gyro suspend mode */
    BNO_GYR_MODE_APS,          /*!< Gyro advanced powersave mode*/
} bno055_gyr_mode_t;

typedef enum bno055_mag_rate {
    BNO_MAG_RATE_2HZ,
    BNO_MAG_RATE_6HZ,
    BNO_MAG_RATE_8HZ,
    BNO_MAG_RATE_10HZ,
    BNO_MAG_RATE_15HZ,
    BNO_MAG_RATE_20HZ,
    BNO_MAG_RATE_25HZ,
    BNO_MAG_RATE_30HZ,
} bno055_mag_rate_t;

typedef enum bno055_mag_mode {
    BNO_MAG_MODE_LOW_PWR = (0 << BNO_MAG_MODE_OFFSET),
    BNO_MAG_MODE_REGULAR = (1 << BNO_MAG_MODE_OFFSET),
    BNO_MAG_MODE_ENH_REG,
    BNO_MAG_MODE_HIGH_ACC,
} bno055_mag_mode_t;

typedef enum bno055_mag_pwr {
    BNO_MAG_PWRMODE_NORMAL = (0 << BNO_MAG_PWRMODE_OFFSET),
    BNO_MAG_PWRMODE_SLEEP = (1 << BNO_MAG_PWRMODE_OFFSET),
    BNO_MAG_PWRMODE_SUSPEND,
    BNO_MAG_PWRMODE_FORCE
} bno055_mag_pwr_t;

typedef enum bno055_acc_unitsel {
    BNO_ACC_UNITSEL_M_S2,
    BNO_ACC_UNITSEL_MG,
} bno055_acc_unitsel_t;

typedef enum bno055_gyr_unitsel {
    BNO_GYR_UNIT_DPS = (0 << BNO_GYR_UNITSEL_OFFSET),
    BNO_GYR_UNIT_RPS = (1 << BNO_GYR_UNITSEL_OFFSET),
} bno055_gyr_unitsel_t;

typedef enum bno055_eul_unitsel {
    BNO_EUL_UNIT_DEG = (0 << BNO_EUL_UNITSEL_OFFSET),
    BNO_EUL_UNIT_RAD = (1 << BNO_EUL_UNITSEL_OFFSET),
} bno055_eul_unitsel_t;

typedef enum bno055_temp_unitsel {
    BNO_TEMP_UNIT_C = (0 << BNO_TEMP_UNITSEL_OFFSET),
    BNO_TEMP_UNIT_F = (1 << BNO_TEMP_UNITSEL_OFFSET),
} bno055_temp_unitsel_t;

typedef enum bno055_opmode {
    BNO_MODE_CONFIG,
    BNO_MODE_AO,
    BNO_MODE_MO,
    BNO_MODE_GO,
    BNO_MODE_AM,
    BNO_MODE_AG,
    BNO_MODE_MG,
    BNO_MODE_AMG,
    BNO_MODE_IMU,
    BNO_MODE_COMPASS,
    BNO_MODE_M4G,
    BNO_MODE_NDOF_FMC_OFF,
    BNO_MODE_NDOF,
} bno055_opmode_t;

enum bno055_temp_src {
    BNO_TEMP_SRC_ACC,
    BNO_TEMP_SRC_GYR,
};

typedef enum _bno055_axis_remap {
    BNO_AXIS_REMAP_X,
    BNO_AXIS_REMAP_Y,
    BNO_AXIS_REMAP_Z,
} bno055_axis_remap;

/**
 * BNO055 error codes
 */
typedef enum _error_bno {
    BNO_OK,                /*!<No error*/
    BNO_ERR_I2C,           /*!<Error on the I2C bus*/
    BNO_ERR_PAGE_TOO_HIGH, /*!<Page set too high
                              [page](`BNO_PAGE_0`,`BNO_PAGE_1`)*/
    BNO_ERR_SETTING_PAGE,
    BNO_ERR_NULL_PTR,
    BNO_ERR_AXIS_REMAP,
    BNO_ERR_WRONG_CHIP_ID,
} error_bno;

typedef struct bno055_euler {
    f32 roll;
    f32 pitch;
    f32 yaw;
} bno055_euler_t;

typedef struct bno055_vec3 {
    f32 x;
    f32 y;
    f32 z;
} bno055_vec3_t;

typedef struct bno055_vec4 {
    f32 x;
    f32 y;
    f32 z;
    f32 w;
} bno055_vec4_t;

// #pragma pack(1)

typedef struct _bno055_t {
    I2C_HandleTypeDef* i2c;
    bno055_opmode_t mode;
    error_bno err;
    u8 addr;
    struct _bno055_t* ptr;

    bno055_pwr_t _pwr_mode;
    bno055_page_t _page;

    // Unit selection for the sensors
    bno055_acc_unitsel_t _acc_unit;
    bno055_temp_unitsel_t _temp_unit;
    bno055_gyr_unitsel_t _gyr_unit;
    bno055_eul_unitsel_t _eul_unit;

    // Accelerometer settings
    bno055_acc_range_t _acc_range;
    bno055_acc_band_t _acc_bandwidth;
    bno055_acc_mode_t _acc_mode;

    // Gyroscope settings
    bno055_gyr_range_t _gyr_range;
    bno055_gyr_band_t _gyr_bandwith;
    bno055_gyr_mode_t _gyr_mode;

    // Magnetometer settings
    bno055_mag_rate_t _mag_out_rate;
    bno055_mag_mode_t _mag_mode;
    bno055_mag_pwr_t _mag_pwr_mode;

    // Will be used later
    void (*read_reg)(u8 addr, u8 mem_addr, u8 size, u8* buf);
    void (*write_reg)(u8 addr, u8 mem_addr, u8 size, u8* buf);

    /**
     * # Read Temperature from internal temperature sensor
     * --------------------------------------------------------
     *
     * Args:
     * > `imu[bno055_t*]`: BNO055 struct
     * > `buf[s8* (int8_t)]`: buffer to store the value to
     * --------------------------------------------------------
     * Return:
     * > `error_bno`: Error code if something fails
     * --------------------------------------------------------
     */
    error_bno (*temperature)(struct _bno055_t* imu, s8* buf);

    error_bno (*acc_x)(struct _bno055_t* imu, f32* buf);
    error_bno (*acc_y)(struct _bno055_t* imu, f32* buf);
    error_bno (*acc_z)(struct _bno055_t* imu, f32* buf);
    error_bno (*acc)(struct _bno055_t* imu, bno055_vec3_t* buf);

    error_bno (*linear_acc_x)(struct _bno055_t* imu, f32* buf);
    error_bno (*linear_acc_y)(struct _bno055_t* imu, f32* buf);
    error_bno (*linear_acc_z)(struct _bno055_t* imu, f32* buf);
    error_bno (*linear_acc)(struct _bno055_t* imu, bno055_vec3_t* buf);

    error_bno (*gyro_x)(struct _bno055_t* imu, f32* buf);
    error_bno (*gyro_y)(struct _bno055_t* imu, f32* buf);
    error_bno (*gyro_z)(struct _bno055_t* imu, f32* buf);
    error_bno (*gyro)(struct _bno055_t* imu, bno055_vec3_t* buf);

    error_bno (*mag_x)(struct _bno055_t* imu, f32* buf);
    error_bno (*mag_y)(struct _bno055_t* imu, f32* buf);
    error_bno (*mag_z)(struct _bno055_t* imu, f32* buf);
    error_bno (*mag)(struct _bno055_t* imu, bno055_vec3_t* buf);

    error_bno (*gravity_x)(struct _bno055_t* imu, f32* buf);
    error_bno (*gravity_y)(struct _bno055_t* imu, f32* buf);
    error_bno (*gravity_z)(struct _bno055_t* imu, f32* buf);
    error_bno (*gravity)(struct _bno055_t* imu, bno055_vec3_t* buf);

    error_bno (*euler_roll)(struct _bno055_t* imu, f32* buf);
    error_bno (*euler_pitch)(struct _bno055_t* imu, f32* buf);
    error_bno (*euler_yaw)(struct _bno055_t* imu, f32* buf);
    error_bno (*euler)(struct _bno055_t* imu, bno055_euler_t* buf);

    error_bno (*quaternion_w)(struct _bno055_t* imu, f32* buf);
    error_bno (*quaternion_x)(struct _bno055_t* imu, f32* buf);
    error_bno (*quaternion_y)(struct _bno055_t* imu, f32* buf);
    error_bno (*quaternion_z)(struct _bno055_t* imu, f32* buf);
    error_bno (*quaternion)(struct _bno055_t* imu, bno055_vec4_t* buf);

    error_bno (*remap_axis)(struct _bno055_t* imu,
                            const bno055_axis_remap xremap,
                            const bno055_axis_remap yremap,
                            const bno055_axis_remap zremap);
    error_bno (*acc_config)(struct _bno055_t* bno,
                            const bno055_acc_range_t range,
                            const bno055_acc_band_t bandwidth,
                            const bno055_acc_mode_t mode);
    error_bno (*gyr_config)(struct _bno055_t* bno,
                            const bno055_gyr_range_t range,
                            const bno055_gyr_band_t bandwidth,
                            const bno055_gyr_mode_t mode);
    error_bno (*mag_config)(struct _bno055_t* bno,
                            const bno055_mag_rate_t out_rate,
                            const bno055_mag_pwr_t pwr_mode,
                            const bno055_mag_mode_t mode);
} bno055_t;

bno055_t bno055_new();

error_bno bno055_init(bno055_t* imu);

error_bno bno055_reset(bno055_t* imu);
error_bno bno055_on(bno055_t* imu);

// Temperature
error_bno bno055_temperature(bno055_t* imu, s8* buf);

// Accelerometer
error_bno bno055_acc_x(bno055_t* imu, f32* buf);
error_bno bno055_acc_y(bno055_t* imu, f32* buf);
error_bno bno055_acc_z(bno055_t* imu, f32* buf);
error_bno bno055_acc(bno055_t* imu, bno055_vec3_t* buf);

// Linear accelerometer
error_bno bno055_linear_acc_x(bno055_t* imu, f32* buf);
error_bno bno055_linear_acc_y(bno055_t* imu, f32* buf);
error_bno bno055_linear_acc_z(bno055_t* imu, f32* buf);
error_bno bno055_linear_acc(bno055_t* imu, bno055_vec3_t* buf);

// Gyroscope
error_bno bno055_gyro_x(bno055_t* imu, f32* buf);
error_bno bno055_gyro_y(bno055_t* imu, f32* buf);
error_bno bno055_gyro_z(bno055_t* imu, f32* buf);
error_bno bno055_gyro(bno055_t* imu, bno055_vec3_t* buf);

// Magnetometer
error_bno bno055_mag_x(bno055_t* imu, f32* buf);
error_bno bno055_mag_y(bno055_t* imu, f32* buf);
error_bno bno055_mag_z(bno055_t* imu, f32* buf);
error_bno bno055_mag(bno055_t* imu, bno055_vec3_t* buf);

// Gravity
error_bno bno055_gravity_x(bno055_t* imu, f32* buf);
error_bno bno055_gravity_y(bno055_t* imu, f32* buf);
error_bno bno055_gravity_z(bno055_t* imu, f32* buf);
error_bno bno055_gravity(bno055_t* imu, bno055_vec3_t* buf);

// Euler angles
error_bno bno055_euler_roll(bno055_t* imu, f32* buf);
error_bno bno055_euler_pitch(bno055_t* imu, f32* buf);
error_bno bno055_euler_yaw(bno055_t* imu, f32* buf);
error_bno bno055_euler(bno055_t* imu, bno055_euler_t* buf);

// Quaternions
error_bno bno055_quaternion_w(bno055_t* imu, f32* buf);
error_bno bno055_quaternion_x(bno055_t* imu, f32* buf);
error_bno bno055_quaternion_y(bno055_t* imu, f32* buf);
error_bno bno055_quaternion_z(bno055_t* imu, f32* buf);
error_bno bno055_quaternion(bno055_t* imu, bno055_vec4_t* buf);

bool bno055_calibrate(bno055_t* imu);
error_bno bno055_remap_axis(bno055_t* imu, const bno055_axis_remap xremap,
                            const bno055_axis_remap yremap,
                            const bno055_axis_remap zremap);
error_bno bno055_ext_crystal(bno055_t* imu, const bool use_xtal);

error_bno bno055_set_page(bno055_t* imu, const bno055_page_t page);
error_bno bno055_set_opmode(bno055_t* imu, const bno055_opmode_t mode);

//==========================| BNO setter |=============================
error_bno bno055_set_page(bno055_t* bno, const bno055_page_t page);
error_bno bno055_set_opmode(bno055_t* bno, const bno055_opmode_t mode);

error_bno bno055_set_unit(bno055_t* bno, const bno055_temp_unitsel_t t_unit,
                          const bno055_gyr_unitsel_t g_unit,
                          const bno055_acc_unitsel_t a_unit,
                          const bno055_eul_unitsel_t e_unit);

error_bno bno055_acc_conf(bno055_t* bno, const bno055_acc_range_t range,
                          const bno055_acc_band_t bandwidth,
                          const bno055_acc_mode_t mode);
error_bno bno055_gyr_conf(bno055_t* bno, const bno055_gyr_range_t range,
                          const bno055_gyr_band_t bandwidth,
                          const bno055_gyr_mode_t mode);
error_bno bno055_mag_conf(bno055_t* bno, const bno055_mag_rate_t out_rate,
                          const bno055_mag_pwr_t pwr_mode,
                          const bno055_mag_mode_t mode);

error_bno bno055_set_pwr_mode(bno055_t* bno, const bno055_pwr_t pwr);
error_bno bno055_set_temp_src(bno055_t* bno, const enum bno055_temp_src src);

//==================================================================

// getter
uint8_t bno055_get_chip_id(const bno055_t bno);

char* bno055_err_str(error_bno err);
error_bno bno055_post_result(bno055_t* bno, char* buf);

error_bno bno055_read_regs(bno055_t imu, u8 addr, uint8_t* buf,
                           uint32_t buf_size);

error_bno bno055_write_regs(bno055_t imu, uint32_t addr, uint8_t* buf,
                            uint32_t buf_size);

#endif
