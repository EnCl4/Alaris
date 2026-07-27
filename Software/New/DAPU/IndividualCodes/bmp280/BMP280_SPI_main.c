/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
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
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
#define BMP280_CS_PORT GPIOB
#define BMP280_CS_PIN GPIO_PIN_2


#define BMP280_REG_ID 0xD0
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_STATUS 0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_REG_CALIB00 0x88

#define BMP280_RESET_VALUE 0xB6

static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

static int32_t t_fine;

static void BMP280_CS_Low(void) { HAL_GPIO_WritePin(BMP280_CS_PORT, BMP280_CS_PIN, GPIO_PIN_RESET); }
static void BMP280_CS_High(void) { HAL_GPIO_WritePin(BMP280_CS_PORT, BMP280_CS_PIN, GPIO_PIN_SET); }

static void BMP280_SPI_Read(uint8_t reg, uint8_t *buf, uint16_t len)
{
uint8_t addr = reg | 0x80; // MSB=1 for read
BMP280_CS_Low();
HAL_SPI_Transmit(&hspi2, &addr, 1, HAL_MAX_DELAY);
HAL_SPI_Receive(&hspi2, buf, len, HAL_MAX_DELAY);
BMP280_CS_High();
}

static void BMP280_SPI_Write(uint8_t reg, uint8_t data)
{
uint8_t buf[2];
buf[0] = reg & 0x7F; // MSB=0 for write
buf[1] = data;
BMP280_CS_Low();
HAL_SPI_Transmit(&hspi2, buf, 2, HAL_MAX_DELAY);
BMP280_CS_High();
}


static uint8_t BMP280_Read8(uint8_t reg)
{
uint8_t val;
BMP280_SPI_Read(reg, &val, 1);
return val;
};

static void BMP280_ReadCalibration(void)
{
uint8_t calib[24];
BMP280_SPI_Read(BMP280_REG_CALIB00, calib, 24);


dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);


dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);
}

uint8_t BMP280_Init(void)
{
uint8_t id = BMP280_Read8(BMP280_REG_ID);
if (id != 0x58) return 0; // BMP280 chip ID


// Soft reset
BMP280_SPI_Write(BMP280_REG_RESET, BMP280_RESET_VALUE);
HAL_Delay(10);


BMP280_ReadCalibration();


// Config: standby 1000 ms, filter off
BMP280_SPI_Write(BMP280_REG_CONFIG, (5 << 5) | (0 << 2));


// Ctrl_meas: temp oversampling x2, press oversampling x16, normal mode
BMP280_SPI_Write(BMP280_REG_CTRL_MEAS, (2 << 5) | (5 << 2) | 3);


return 1;
};

static int32_t BMP280_ReadRawTemp(void)
{
uint8_t buf[3];
BMP280_SPI_Read(BMP280_REG_TEMP_MSB, buf, 3);
return (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
};

static int32_t BMP280_ReadRawPress(void){
uint8_t buf[3];
BMP280_SPI_Read(BMP280_REG_PRESS_MSB, buf, 3);
return (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
};


float BMP280_ReadTemperature(void)
{
int32_t adc_T = BMP280_ReadRawTemp();


int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;


t_fine = var1 + var2;


float T = (t_fine * 5 + 128) >> 8;
return T / 100.0f;
};


float BMP280_ReadPressure(void)
{
int32_t adc_P = BMP280_ReadRawPress();


int64_t var1 = ((int64_t)t_fine) - 128000;
int64_t var2 = var1 * var1 * (int64_t)dig_P6;
var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
var2 = var2 + (((int64_t)dig_P4) << 35);
var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;


if (var1 == 0) return 0; // avoid exception


int64_t p = 1048576 - adc_P;
p = (((p << 31) - var2) * 3125) / var1;
var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
var2 = (((int64_t)dig_P8) * p) >> 19;


p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);


return (float)p / 256.0f; // Pa
};


float BMP280_ComputeAltitude(float pressure_pa, float sea_level_pa)
{
return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1903f));
};


// Example usage in main loop
void BMP280_Task(float *temperature, float *pressure, float *altitude)
{
    *temperature = BMP280_ReadTemperature();
    *pressure    = BMP280_ReadPressure();
    *altitude    = BMP280_ComputeAltitude(*pressure, 101325.0f);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	float te, pr, al;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  if (!BMP280_Init())
  {
      // Handle error: sensor not detected
  }

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable trace
  DWT->CYCCNT = 0;                                // Reset counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable cycle counter
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  uint32_t start_cycles = DWT->CYCCNT;

	  BMP280_Task(&te, &pr, &al);

	  uint32_t end_cycles = DWT->CYCCNT;
	  uint32_t delta_cycles = end_cycles - start_cycles;
	  printf(">T:%.2f\n>P:%.2f\n>Alt:%.2f\r\n", te, pr, al);
	  printf("Ciclos: %lu \n", delta_cycles);
	  float time_us = (float)delta_cycles / 84.0f;
	  printf("Us: %.3f \n", time_us);
	  HAL_Delay(10);



  }
  /* USER CODE END 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
