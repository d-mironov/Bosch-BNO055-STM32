#ifndef __MAIN_H
#define __MAIN_H

//#define SSD1306_HEIGHT 32
#define SSD1306_HEIGHT      32
#define SSD1306_I2C_ADDR    0x3C
#define SSD1306_COM_LR_REMAP

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"


#define ERROR_LED           GPIO_PIN_8
#define ERROR_LED_PORT      GPIOB

#define STATUS_LED          GPIO_PIN_8
#define STATUS_LED_PORT     GPIOA

#define I2C1_SCL            GPIO_PIN_6
#define I2C1_SDA            GPIO_PIN_7
#define I2C1_PORT           GPIOB

#define I2C2_SCL            GPIO_PIN_10
#define I2C2_SDA            GPIO_PIN_9
#define I2C2_PORT           GPIOB

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

