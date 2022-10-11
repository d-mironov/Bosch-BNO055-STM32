/**
 * This Library was originally written by Olivier Van den Eede (4ilo) in 2016.
 * Some refactoring was done and SPI support was added by Aleksander Alekseev (afiskon) in 2018.
 *
 * https://github.com/afiskon/stm32-ssd1306
 */

#ifndef __DP2_H__
#define __DP2_H__

#include <stddef.h>
#include <_ansi.h>

_BEGIN_STD_C

#include "dp2_conf.h"

#if defined(STM32F0)
#include "stm32f0xx_hal.h"
#elif defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#elif defined(STM32L0)
#include "stm32l0xx_hal.h"
#elif defined(STM32L1)
#include "stm32l1xx_hal.h"
#elif defined(STM32L4)
#include "stm32l4xx_hal.h"
#elif defined(STM32F3)
#include "stm32f3xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#elif defined(STM32F7)
#include "stm32f7xx_hal.h"
#elif defined(STM32G0)
#include "stm32g0xx_hal.h"
#elif defined(STM32G4)
#include "stm32g4xx_hal.h"
#else
#error "SSD1306 library was tested only on STM32F0, STM32F1, STM32F3, STM32F4, STM32F7, STM32L0, STM32L1, STM32L4, STM32H7, STM32G0, STM32G4 MCU families. Please modify ssd1306.h if you know what you are doing. Also please send a pull request if it turns out the library works on other MCU's as well!"
#endif

#ifdef DP2_X_OFFSET
#define DP2_X_OFFSET_LOWER (DP2_X_OFFSET & 0x0F)
#define DP2_X_OFFSET_UPPER ((DP2_X_OFFSET >> 4) & 0x07)
#else
#define DP2_X_OFFSET_LOWER 0
#define DP2_X_OFFSET_UPPER 0
#endif

#include "ssd1306_fonts.h"

/* vvv I2C config vvv */

#ifndef DP2_I2C_PORT
#define DP2_I2C_PORT        hi2c2
#endif

#ifndef DP2_I2C_ADDR
#define DP2_I2C_ADDR        (0x3C << 1)
#endif

/* ^^^ I2C config ^^^ */

/* vvv SPI config vvv */

#ifndef DP2_SPI_PORT
#define DP2_SPI_PORT        hspi2
#endif

#ifndef DP2_CS_Port
#define DP2_CS_Port         GPIOB
#endif
#ifndef DP2_CS_Pin
#define DP2_CS_Pin          GPIO_PIN_12
#endif

#ifndef DP2_DC_Port
#define DP2_DC_Port         GPIOB
#endif
#ifndef DP2_DC_Pin
#define DP2_DC_Pin          GPIO_PIN_14
#endif

#ifndef DP2_Reset_Port
#define DP2_Reset_Port      GPIOA
#endif
#ifndef DP2_Reset_Pin
#define DP2_Reset_Pin       GPIO_PIN_8
#endif

/* ^^^ SPI config ^^^ */
#if defined(DP2_USE_I2C)
extern I2C_HandleTypeDef DP2_I2C_PORT;
#elif defined(DP2_USE_SPI)
extern SPI_HandleTypeDef DP2_SPI_PORT;
#else
#error "You should define DP2_USE_SPI or DP2_USE_I2C macro!"
#endif

// SSD1306 OLED height in pixels
#ifndef DP2_HEIGHT
#define DP2_HEIGHT          64
#endif

// SSD1306 width in pixels
#ifndef DP2_WIDTH
#define DP2_WIDTH           128
#endif

#ifndef DP2_BUFFER_SIZE
#define DP2_BUFFER_SIZE   DP2_WIDTH * DP2_HEIGHT / 8
#endif

// Enumeration for screen colors
typedef enum {
    DP2_Black = 0x00, // Black color, no pixel
    DP2_White = 0x01  // Pixel is set. Color depends on OLED
} DP2_COLOR;

typedef enum {
    DP2_OK = 0x00,
    DP2_ERR = 0x01  // Generic error.
} DP2_Error_t;

// Struct to store transformations
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Initialized;
    uint8_t DisplayOn;
} DP2_t;

typedef struct {
    uint8_t x;
    uint8_t y;
} DP2_VERTEX;

// Procedure definitions
void dp2_Init(void);
void dp2_Fill(DP2_COLOR color);
void dp2_UpdateScreen(void);
void dp2_DrawPixel(uint8_t x, uint8_t y, DP2_COLOR color);
char dp2_WriteChar(char ch, FontDef Font, DP2_COLOR color);
char dp2_WriteString(char* str, FontDef Font, DP2_COLOR color);
void dp2_SetCursor(uint8_t x, uint8_t y);
void dp2_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, DP2_COLOR color);
void dp2_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, DP2_COLOR color);
void dp2_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, DP2_COLOR color);
void dp2_Polyline(const DP2_VERTEX *par_vertex, uint16_t par_size, DP2_COLOR color);
void dp2_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, DP2_COLOR color);
void dp2_DrawBitmap(uint8_t x, uint8_t y, const unsigned char* bitmap, uint8_t w, uint8_t h, DP2_COLOR color);
/**
 * @brief Sets the contrast of the display.
 * @param[in] value contrast to set.
 * @note Contrast increases as the value increases.
 * @note RESET = 7Fh.
 */
void dp2_SetContrast(const uint8_t value);
/**
 * @brief Set Display ON/OFF.
 * @param[in] on 0 for OFF, any for ON.
 */
void dp2_SetDisplayOn(const uint8_t on);
/**
 * @brief Reads DisplayOn state.
 * @return  0: OFF.
 *          1: ON.
 */
uint8_t dp2_GetDisplayOn();

// Low-level procedures
void dp2_Reset(void);
void dp2_WriteCommand(uint8_t byte);
void dp2_WriteData(uint8_t* buffer, size_t buff_size);
DP2_Error_t dp2_FillBuffer(uint8_t* buf, uint32_t len);

_END_STD_C

#endif // __DP2_H__
