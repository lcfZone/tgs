/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AD7768.h"
#include "debug.h"
#include "LTC2664.h"
#include "ADXL382.h"
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern uint8_t rxBuffer[512];
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern uint8_t receive_flag;
extern const float LTC2664_MIN_OUTPUT[5];
extern const float LTC2664_MAX_OUTPUT[5];

int32_t test[1000];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_SPI6_Init();
  MX_UART8_Init();

#ifdef LTC2664
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);

//    LTC2664_write(LTC2664_CMD_TOGGLE_SEL, 0, 0);
//    HAL_Delay(1);
//    LTC2664_write(LTC2664_CMD_GLOBAL_TOGGLE, 0, 0);
//    HAL_Delay(1);

    LTC2664_write_command(LTC2664_CMD_SPAN, 0, LTC2664_SPAN_PLUS_MINUS_5V);
    HAL_Delay(1);
    LTC2664_write_command(LTC2664_CMD_SPAN, 1, LTC2664_SPAN_PLUS_MINUS_5V);
    HAL_Delay(1);
    LTC2664_write_command(LTC2664_CMD_SPAN, 2, LTC2664_SPAN_PLUS_MINUS_5V);
    HAL_Delay(1);

    LTC2664_write_command(LTC2664_CMD_CONFIG, 0, 1);
    HAL_Delay(1);

    /* Calculate mid-scale code for 0V output */
    uint16_t code0 = LTC2664_voltage_to_code(
		0.0f,
		LTC2664_MIN_OUTPUT[LTC2664_SPAN_PLUS_MINUS_5V],
		LTC2664_MAX_OUTPUT[LTC2664_SPAN_PLUS_MINUS_5V]
    );
    uint16_t code1 = LTC2664_voltage_to_code(
		0.0f,
		LTC2664_MIN_OUTPUT[LTC2664_SPAN_PLUS_MINUS_5V],
		LTC2664_MAX_OUTPUT[LTC2664_SPAN_PLUS_MINUS_5V]
    );
    uint16_t code2 = LTC2664_voltage_to_code(
		0.0f,
		LTC2664_MIN_OUTPUT[LTC2664_SPAN_PLUS_MINUS_5V],
		LTC2664_MAX_OUTPUT[LTC2664_SPAN_PLUS_MINUS_5V]
    );

    /* Write and update channel 0 with the code */
    LTC2664_write(LTC2664_CMD_WRITE_N_UPDATE_N, 0, code0);
    HAL_Delay(1);
    LTC2664_write(LTC2664_CMD_WRITE_N_UPDATE_N, 1, code1);
    HAL_Delay(1);
    LTC2664_write(LTC2664_CMD_WRITE_N_UPDATE_N, 2, code2);

    set_opa(OPA_GAIN_025);
#endif

#ifdef AD7768
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  my_ad7768_init();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  
  offsetCalibration();
  uint16_t cnt = 0;

  while (1)
  {
    while (!receive_flag)
    {
       HAL_Delay(1);
    }
    receive_flag = 0;
    printf("------------\r\n");
    for (int i = 0; i < 4; i++)
    {
      printf("%u\r\n", rxBuffer[i]);
    }
    cnt++;
    if (cnt >= 100)
    {
      int32_t res = (rxBuffer[1] << 16) | (rxBuffer[2] << 8) | rxBuffer[3];
      if (res & 0x800000) {
        res |= 0xFF000000;
      }
      test[cnt - 100] = res;
    }
    if (cnt == 1099)
    {
      uint64_t sum_sq = 0;
    for (uint16_t i = 0; i < 1000; i++) {
        int64_t v = test[i];
        sum_sq += (uint64_t)(v * v);
        printf("%lld\r\n", (uint64_t)(v * v));
    }
    double mean_sq = (double)sum_sq / (double)1000;
    mean_sq = sqrt(mean_sq);
    printf("rms %f\r\n", mean_sq);
    HAL_Delay(10000);
    }
    
  }
#endif

#ifdef ADXL382
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
    struct adxl38x_dev *adxl38x_desc;
	int ret;
	uint8_t register_value;
	uint8_t status0;
	uint8_t fifo_status[2];
	uint8_t fifo_data[36];
	uint8_t set_fifo_entries = 0x0C;
	uint16_t fifo_entries;
	struct adxl38x_fractional_val data_frac[15];
	enum adxl38x_id devID;

	ret = adxl38x_soft_reset(adxl38x_desc);
	if (ret == -1)
		printf("Reset was not successful\r\n");
	else if (ret)
		goto error;
	ret = adxl38x_get_deviceID(adxl38x_desc, &devID);
	if (ret)
		goto error;
	ret = adxl38x_set_range(adxl38x_desc, ADXL382_RANGE_15G);
	if (ret)
		goto error;

	// FIFO sequence
	// Put the part in standby mode
	ret = adxl38x_set_op_mode(adxl38x_desc, ADXL38X_MODE_STDBY);
	if (ret)
		goto error;
	// Set FILTER register to 0x70 (Bypass EQ, LPF_MODE 0b11)
	register_value = 0x70;
	ret = adxl38x_write_device_data(adxl38x_desc, ADXL38X_FILTER, 1,
					&register_value);
	if (ret)
		goto error;

	// Set DIG_EN register to 0x78 (Enable XYZ axes and FIFO enable)
	register_value = 0x78;
	ret = adxl38x_write_device_data(adxl38x_desc, ADXL38X_DIG_EN, 1,
					&register_value);
	if (ret)
		goto error;

	// Set FIFO_CFG0 to 0x60 (Channel ID enable and FIFO stream mode)
	ret = adxl38x_accel_set_FIFO(adxl38x_desc, set_fifo_entries,
				     false, ADXL38X_FIFO_STREAM, true, false);

	// Set INT0_MAP0 to 0x08 (FIFO_WATERMARK_INT0)
	register_value = 0x08;
	ret = adxl38x_write_device_data(adxl38x_desc, ADXL38X_INT0_MAP0, 1,
					&register_value);
	if (ret)
		goto error;

	// Put the part in HP mode and read data when FIFO watermark pin is set
	ret = adxl38x_set_op_mode(adxl38x_desc, ADXL38X_MODE_HP);
	if (ret)
		goto error;

	printf("Starting watermark check\r\n");
	while (true) {
		// Read status to assert if FIFO_WATERMARK bit set
		ret = adxl38x_read_device_data(adxl38x_desc, ADXL38X_STATUS0, 1, &status0);
		if (ret)
			goto error;
		printf("Status 0: %d\r\n", status0);
		ret = adxl38x_read_device_data(adxl38x_desc, ADXL38X_FIFO_STATUS0, 2,
					       fifo_status);
		if (ret)
			goto error;
		fifo_entries = fifo_status[0] | (fifo_status[1] << 8);
		fifo_entries = fifo_entries & 0x01ff;


		// Read FIFO status and data if FIFO_WATERMARK is set
		if (status0 & BIT(3)) {
			printf(" FIFO_WATERMARK is set. Total fifo entries =  %d\r\n", fifo_entries);
			if (fifo_entries < set_fifo_entries)
				goto unmatch_error;

			// Read data from FIFO (can read at least upto 12 samples * 3 bytes (chID, data))
			ret = adxl38x_read_device_data(adxl38x_desc, ADXL38X_FIFO_DATA, 36, fifo_data);
			if (ret)
				goto error;

			// Parse Data for fist five samples
			printf("First four entries (absolute values printed for magnitude between -1g & 1g):\r\n");
			for (int b = 0; b < 36; b += 3) {
				ret = adxl38x_data_raw_to_gees(adxl38x_desc, (fifo_data + b + 1), data_frac);
				if (ret)
					goto error;
				printf("%c : %lld.%07dg\r\n", getaxis(fifo_data[b]), data_frac->integer,
					abs(data_frac->fractional));
			}
		}
	}

error:
	if (ret)
		printf("Error occurred!");
	else
		printf("The program has ended after successful execution\r\n");
	return 0;
unmatch_error:
	printf("Number of entries in FIFO not matching the number set in FIFO config\r\n");
	return 0;
#endif

    while (1) {
    }

}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
