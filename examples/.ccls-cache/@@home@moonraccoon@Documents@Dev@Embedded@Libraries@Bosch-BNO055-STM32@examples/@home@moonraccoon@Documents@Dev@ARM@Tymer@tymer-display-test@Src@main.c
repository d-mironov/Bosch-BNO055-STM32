#include <stdbool.h>
#include <stdio.h>

#include "dp1/dp1.h"
#include "dp1/ssd1306_fonts.h"
#include "dp2/dp2.h"
#include "main.h"
#include "retarget.h"
#include "stm32f4xx_hal_gpio_ex.h"

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);

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
    MX_I2C2_Init();

    // ssd1306_TestAll();
    //  Intialize screen
    dp1_Init();
    dp1_SetDisplayOn(1);
    dp1_Fill(DP1_Black);
    dp1_SetCursor(1, 1);
    dp1_WriteString("Hello world!", Font_6x8, DP1_White);
    dp1_UpdateScreen();

    HAL_Delay(200);

    dp2_Init();
    dp2_SetDisplayOn(1);
    dp2_Fill(DP2_Black);
    dp2_SetCursor(1, 1);
    dp2_WriteString("Hello world!", Font_6x8, DP2_White);
    dp2_UpdateScreen();

    HAL_Delay(2000);
    while (1) {
        // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
        ////HAL_UART_Transmit(&huart2, "Hello world\r\n", 14, 10000);
        // HAL_Delay(500);
        HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED);
        HAL_Delay(500);
        dp1_Fill(DP1_Black);
        dp1_SetCursor(50, 1);
        dp1_Line(0, 10, 128, 10, DP1_White);
        dp1_WriteString("13:26", Font_6x8, DP1_White);
        dp1_UpdateScreen();

        dp2_Fill(DP2_Black);
        dp2_SetCursor(50, 1);
        dp2_Line(0, 10, 128, 10, DP2_White);
        dp2_WriteString("13:26", Font_6x8, DP2_White);
        dp2_UpdateScreen();
    }
}

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

static void MX_I2C2_Init(void) {
    __HAL_RCC_I2C2_CLK_ENABLE();
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
}

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

    GPIO_InitTypeDef gpio = {0};
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pin = ERROR_LED | STATUS_LED;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ERROR_LED_PORT, &gpio);

    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Alternate = GPIO_AF4_I2C2;
    gpio.Pin = I2C1_SCL | I2C1_SDA;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(I2C1_PORT, &gpio);

    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Alternate = GPIO_AF4_I2C2;
    gpio.Pin = I2C2_SCL;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(I2C2_PORT, &gpio);

    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Alternate = GPIO_AF9_I2C2;
    gpio.Pin = I2C2_SDA;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(I2C2_PORT, &gpio);
}

void SystemClock_Config(void) {
    // RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    // RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    ///** Configure the main internal regulator output voltage
    //*/
    //__HAL_RCC_PWR_CLK_ENABLE();
    //__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    ///** Initializes the RCC Oscillators according to the specified parameters
    //* in the RCC_OscInitTypeDef structure.
    //*/
    // RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    // RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    // RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    // RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    // RCC_OscInitStruct.PLL.PLLM = 25;
    // RCC_OscInitStruct.PLL.PLLN = 192;
    // RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    // RCC_OscInitStruct.PLL.PLLQ = 4;
    // if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    //{
    //  Error_Handler();
    //}

    ///** Initializes the CPU, AHB and APB buses clocks
    //*/
    // RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    //                            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    // RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    // RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    // RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    // RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    // if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    //{
    //   Error_Handler();
    // }
    //   /**Configure the Systick interrupt time
    //   */
    // HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    //  /**Configure the Systick
    //  */
    // HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

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
        HAL_GPIO_TogglePin(ERROR_LED_PORT, ERROR_LED);
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
