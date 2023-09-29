#include <stdbool.h>
#include <stdio.h>

#include "../../../bno055.h"
#include "../../../bno_config.h"
// #include "bno055/bno055.h"
// #include "bno055/bno_config.h"
#include "main.h"
#include "retarget.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_gpio_ex.h"

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
bno055_t bno;
error_bno err;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

static void status_led_write(bool val);
static void status_led_toggle();
// static void error_led_write(bool val);
static void error_led_toggle();
static inline void delay_ms(u32 ms);
// static void MX_I2C2_Init(void);

int main(void) {
    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    uart_init();
    MX_I2C1_Init();
    // MX_I2C2_Init();

    bno = (bno055_t){
        .i2c = &hi2c1, .addr = BNO_ADDR, .mode = BNO_MODE_IMU, ._temp_unit = 0,
        // .ptr = &bno,
    };
    status_led_toggle();
    error_led_toggle();
    delay_ms(1000);
    status_led_toggle();
    error_led_toggle();

    if ((err = bno055_init(&bno)) == BNO_OK) {
        printf("[+] BNO055 init success\r\n");
        status_led_write(1);
        delay_ms(100);
        status_led_toggle();
    } else {
        printf("[!] BNO055 init failed\r\n");
        printf("%s\n", bno055_err_str(err));
        Error_Handler();
    }
    delay_ms(100);
    err = bno055_set_unit(&bno, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_DPS,
                          BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_DEG);
    if (err != BNO_OK) {
        printf("[BNO] Failed to set units. Err: %d\r\n", err);
    } else {
        printf("[BNO] Unit selection success\r\n");
    }

    delay_ms(1000);
    s8 temperature = 0;
    // f32 acc_x = 0.0f, acc_y = 0.0f, acc_z = 0.0f;
    bno055_vec3_t acc = {0, 0, 0};
    bno055_vec3_t lia = {0, 0, 0};
    bno055_vec3_t gyr = {0, 0, 0};
    bno055_vec3_t mag = {0, 0, 0};
    bno055_vec3_t grv = {0, 0, 0};
    bno055_euler_t eul = {0, 0, 0};
    bno055_vec4_t qua = {0, 0, 0};
    while (1) {
        // bno055_temperature(&bno, &temperature);
        bno.temperature(&bno, &temperature);
        bno.acc(&bno, &acc);
        bno.linear_acc(&bno, &lia);
        bno.gyro(&bno, &gyr);
        bno.mag(&bno, &mag);
        bno.gravity(&bno, &grv);
        bno.euler(&bno, &eul);
        bno.quaternion(&bno, &qua);
        printf("[+] Temperature: %2d°C\r\n", temperature);
        printf("[+] ACC - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", acc.x, acc.y,
               acc.z);
        printf("[+] LIA - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", lia.x, lia.y,
               lia.z);
        printf("[+] GYR - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", gyr.x, gyr.y,
               gyr.z);
        printf("[+] MAG - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", mag.x, mag.y,
               mag.z);
        printf("[+] GRV - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", grv.x, grv.y,
               grv.z);
        printf("[+] Roll: %+2.2f | Pitch: %+2.2f | Yaw: %+2.2f\r\n", eul.roll,
               eul.pitch, eul.yaw);
        printf("[+] QUA - w: %+2.2f | x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n",
               qua.w, qua.x, qua.y, qua.z);
        HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED);
        delay_ms(100);
    }
}

void status_led_write(bool val) {
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED,
                      val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void status_led_toggle() { HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED); }

void error_led_write(bool val) {
    HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED,
                      val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void error_led_toggle() { HAL_GPIO_TogglePin(ERROR_LED_PORT, ERROR_LED); }

static inline void delay_ms(u32 ms) { HAL_Delay(ms); }

static void MX_I2C1_Init(void) {
    __HAL_RCC_I2C1_CLK_ENABLE();
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

// static void MX_I2C2_Init(void) {
//     __HAL_RCC_I2C2_CLK_ENABLE();
//     hi2c2.Instance = I2C2;
//     hi2c2.Init.ClockSpeed = 100000;
//     hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
//     hi2c2.Init.OwnAddress1 = 0;
//     hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//     hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//     hi2c2.Init.OwnAddress2 = 0;
//     hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//     hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//     if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
//         Error_Handler();
//     }
// }

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pin = ERROR_LED;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ERROR_LED_PORT, &gpio);

    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pin = STATUS_LED;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(STATUS_LED_PORT, &gpio);

    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Alternate = GPIO_AF4_I2C2;
    gpio.Pin = I2C1_SCL | I2C1_SDA;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(I2C1_PORT, &gpio);

    // gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    // gpio.Mode = GPIO_MODE_AF_OD;
    // gpio.Alternate = GPIO_AF4_I2C2;
    // gpio.Pin =  I2C2_SCL;
    // gpio.Pull = GPIO_NOPULL;
    // HAL_GPIO_Init(I2C2_PORT, &gpio);

    // gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    // gpio.Mode = GPIO_MODE_AF_OD;
    // gpio.Alternate = GPIO_AF9_I2C2;
    // gpio.Pin =  I2C2_SDA;
    // gpio.Pull = GPIO_NOPULL;
    // HAL_GPIO_Init(I2C2_PORT, &gpio);
}

void SystemClock_Config(void) {
    /* SysTick_IRQn interrupt configuration */
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 8;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    //__disable_irq();
    while (1) {
        error_led_toggle();
        HAL_Delay(200);
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
