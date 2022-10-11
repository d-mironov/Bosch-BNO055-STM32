#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include <stdio.h>
#include "retarget.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

UART_HandleTypeDef __huart2;

void uart_init(void) {
    // UART GPIO init
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // PA2 and PA3 for UART2
    GPIO_InitTypeDef gpio_init;
    gpio_init.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Alternate = GPIO_AF7_USART2;
    gpio_init.Speed = GPIO_SPEED_MEDIUM;
    gpio_init.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    // setup USART
    __USART2_CLK_ENABLE();
    __huart2.Instance = USART2;
    __huart2.Init.BaudRate = 115200;
    __huart2.Init.WordLength = UART_WORDLENGTH_8B;
    __huart2.Init.StopBits = UART_STOPBITS_1;
    __huart2.Init.Parity = UART_PARITY_NONE;
    __huart2.Init.Mode = UART_MODE_TX_RX;
    __huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    __huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&__huart2);
}

#if defined(__GNUC__)
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&__huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
    return len;
}
#elif defined(__ICCARM__)
int __write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&__huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
    return len;
}
#elif defined(__CC_ARM)
int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&__huart2, (uint8_t*)(&ch), 1, 1000);
    return ch;
}
#endif


int __io_putchar(int ch)
{
    HAL_UART_Transmit(&__huart2,(uint8_t *)&ch,1,HAL_MAX_DELAY);
    return ch;
}

int __io_getchar(){
    uint8_t ch8;
    HAL_UART_Receive(&__huart2,&ch8,1,HAL_MAX_DELAY);
    return ch8;
}
