/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "icm20948.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <math.h>
#include "fatfs.h"
#include "sd_functions.h"
#include "sd_benchmark.h"
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
/* USER CODE BEGIN Variables */

osMutexId_t uartMutex;

int _write(int file, char *ptr, int len)
{
    osMutexAcquire(uartMutex, osWaitForever);
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    osMutexRelease(uartMutex);
    return len;
}



/* USER CODE END Variables */
/* Definitions for ICM20948_Accel */
osThreadId_t ICM20948_AccelHandle;
const osThreadAttr_t ICM20948_Accel_attributes = {
  .name = "ICM20948_Accel",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ICM20948_Gyro */
osThreadId_t ICM20948_GyroHandle;
const osThreadAttr_t ICM20948_Gyro_attributes = {
  .name = "ICM20948_Gyro",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for BMP280_Temp_Pre */
osThreadId_t BMP280_Temp_PreHandle;
const osThreadAttr_t BMP280_Temp_Pre_attributes = {
  .name = "BMP280_Temp_Pre",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for SD_data */
osThreadId_t SD_dataHandle;
const osThreadAttr_t SD_data_attributes = {
  .name = "SD_data",
  .stack_size = 128 * 64,
  .priority = (osPriority_t) osPriorityHigh7,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartICm20948_Accel(void *argument);
void StartICM20948_Gyro(void *argument);
void BMP280_Temp_Press_Alt(void *argument);
void SD_dataProcess(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	//BMP280_Init();

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
		/* Create mutex */
			  const osMutexAttr_t uartMutexAttr = {
			      .name = "UART_Mutex"
			  };
			  uartMutex = osMutexNew(&uartMutexAttr);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ICM20948_Accel */
  ICM20948_AccelHandle = osThreadNew(StartICm20948_Accel, NULL, &ICM20948_Accel_attributes);

  /* creation of ICM20948_Gyro */
  ICM20948_GyroHandle = osThreadNew(StartICM20948_Gyro, NULL, &ICM20948_Gyro_attributes);

  /* creation of BMP280_Temp_Pre */
  BMP280_Temp_PreHandle = osThreadNew(BMP280_Temp_Press_Alt, NULL, &BMP280_Temp_Pre_attributes);

  /* creation of SD_data */
  SD_dataHandle = osThreadNew(SD_dataProcess, NULL, &SD_data_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartICm20948_Accel */
int cell = 0;
/**
  * @brief  Function implementing the ICM20948_Accel thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartICm20948_Accel */
void StartICm20948_Accel(void *argument)
{
  /* USER CODE BEGIN StartICm20948_Accel */
  /* Infinite loop */
  for(;;)
  {
	  cell++;
    osDelay(1000);
  }
  /* USER CODE END StartICm20948_Accel */
}

/* USER CODE BEGIN Header_StartICM20948_Gyro */
/**
* @brief Function implementing the ICM20948_Gyro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartICM20948_Gyro */
void StartICM20948_Gyro(void *argument)
{
  /* USER CODE BEGIN StartICM20948_Gyro */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartICM20948_Gyro */
}

/* USER CODE BEGIN Header_BMP280_Temp_Press_Alt */
/**
* @brief Function implementing the BMP280_Temp_Pre thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BMP280_Temp_Press_Alt */
void BMP280_Temp_Press_Alt(void *argument)
{
  /* USER CODE BEGIN BMP280_Temp_Press_Alt */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END BMP280_Temp_Press_Alt */
}

/* USER CODE BEGIN Header_SD_dataProcess */
/**
* @brief Function implementing the SD_data thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SD_dataProcess */
void SD_dataProcess(void *argument)
{
  /* USER CODE BEGIN SD_dataProcess */
	int count = 0;
	char buffer[32];
  /* Infinite loop */
  for(;;)
  {
	//char buffer[32];   // enough for a float + newline

//	sd_mount();
////	//snprintf(buffer, sizeof(buffer), "%.3f\n", te);
//	sd_append_file("log.txt", "added\n");
//	sd_unmount();

	//sd_benchmark();

	  sd_mount();

	  snprintf(buffer, sizeof(buffer), "count: %d, C2: %d\r\n", count, cell);
	  sd_append_file("log.txt", buffer);

	  sd_unmount();
	  printf("1\n");
	  count++;

    osDelay(10);
  }
  /* USER CODE END SD_dataProcess */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

