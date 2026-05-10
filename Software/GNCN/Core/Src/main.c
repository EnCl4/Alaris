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
#include "cmsis_os.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
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
uint8_t rx_data[8];
uint8_t tx_data[8];  // response (optional)

uint8_t counter = 0;

            uint16_t varA = 0;
            uint16_t varB = 0;

            uint8_t flags = 0;
            uint8_t checksum = 0;

            // ---- 3. Validate checksum ----
            uint8_t calc = 0;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1)
    {
        // ---- 1. Validate header ----
    	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);




            // ---- 2. Extract data ----
            counter = rx_data[1];

            varA = (rx_data[2] << 8) | rx_data[3];
            varB = (rx_data[4] << 8) | rx_data[5];

            flags = rx_data[6];
            checksum = rx_data[7];

            // ---- 3. Validate checksum ----
            calc = rx_data[0] ^ rx_data[1] ^ rx_data[2] ^ rx_data[3] ^
                           rx_data[4] ^ rx_data[5] ^ rx_data[6];


        //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);

        // ---- 4. Prepare response (optional) ----
        tx_data[0] = 0xB1;  // response header
        tx_data[1] = rx_data[1]; // echo counter (good for sync/debug)

        // ---- 5. RE-ARM SPI (CRITICAL) ----
        HAL_SPI_TransmitReceive_IT(&hspi1, tx_data, rx_data, 8);
    }
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//int _write(int file, char *ptr, int len)
//{
//    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
//    return len;
//}


//typedef struct
//  {
//      uint32_t pulse_us;
//      uint32_t period_us;
//      float duty;
//
//      uint32_t last_update_tick;
//  } PWM_Channel_t;
//
//
//
//  #define PWM_CHANNELS 6
//  volatile PWM_Channel_t pwm[PWM_CHANNELS];

//  void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//  {
//      uint32_t period, pulse;
//
//      if(htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1)
//          return;
//
//      if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//      {
//          period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//          pulse  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
//      }
//
//      if(period == 0) return;
//
//      uint8_t idx = 0xFF;
//
//      if(htim->Instance == TIM1) idx = 0;
//      else if(htim->Instance == TIM2) idx = 1;
//      else if(htim->Instance == TIM3) idx = 2;
//      else if(htim->Instance == TIM4) idx = 3;
//      else if(htim->Instance == TIM5) idx = 4;
//      else if(htim->Instance == TIM8) idx = 5;
//
//      if(idx < PWM_CHANNELS)
//      {
//          pwm[idx].pulse_us = pulse;   // assuming prescaler = 1 MHz
//          pwm[idx].period_us = period;
//          pwm[idx].duty = (float)pulse / (float)period;
//          //pwm[idx].last_update_tick = xTaskGetTickCountFromISR();
//      }
//  }



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





//  float mapPWM(uint32_t pulse)
//  {
//      // Typical RC: 1000–2000 µs
//      return ((float)pulse - 1500.0f) / 500.0f; // → [-1, 1]
//  }


//  typedef struct
//  {
//      float Kp;
//      float Ki;
//      float Kd;
//
//      float integrator;
//      float prev_error;
//
//      float output_min;
//      float output_max;
//
//      float dt;   // sampling time (seconds)
//  } PID_t;
//
//  PID_t roll_pid;
//  PID_t pitch_pid;
//  PID_t yaw_pid;
//  PID_t thr_pid;

//
//  float PID_Update(PID_t *pid, float setpoint, float measurement)
//  {
//      float error = setpoint - measurement;
//
//      // Proportional
//      float P = pid->Kp * error;
//
//      // Integral (with anti-windup clamp)
//      pid->integrator += pid->Ki * error * pid->dt;
//
//      if(pid->integrator > pid->output_max)
//          pid->integrator = pid->output_max;
//      else if(pid->integrator < pid->output_min)
//          pid->integrator = pid->output_min;
//
//      // Derivative (on error)
//      float derivative = (error - pid->prev_error) / pid->dt;
//      float D = pid->Kd * derivative;
//
//      // Total output
//      float output = P + pid->integrator + D;
//
//      // Output saturation
//      if(output > pid->output_max)
//          output = pid->output_max;
//      else if(output < pid->output_min)
//          output = pid->output_min;
//
//      // Save state
//      pid->prev_error = error;
//
//      return output;
//  }

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_SPI_TransmitReceive_IT(&hspi1, tx_data, rx_data, 8);

  /* Start PWM input capture */

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);

  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  //HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 910);


  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable trace
  DWT->CYCCNT = 0;                                // Reset counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable cycle counter

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
