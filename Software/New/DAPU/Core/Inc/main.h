/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/** Enables the GPIOE clock and configures PE3 as an output. Safe to call
 *  before HAL_Init() and before the clock tree is set up. */
void DAPU_BootLedInit(void);

/** `count` quick toggles of the on-board LED. Busy-wait, so it works with no
 *  SysTick, and polarity independent because it toggles. */
void DAPU_BootLedFlash(uint8_t count);

/** Never returns. Blinks the on-board LED in groups of `pulses` so a failure
 *  before the scheduler is still diagnosable without a debugger.
 *  1 pulse = Error_Handler, 3 pulses = hard fault. */
void DAPU_BlinkForever(uint8_t pulses);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BMP280_CS_Pin GPIO_PIN_7
#define BMP280_CS_GPIO_Port GPIOE
#define ICM20948_CS_Pin GPIO_PIN_8
#define ICM20948_CS_GPIO_Port GPIOE
#define MS5611_CS_Pin GPIO_PIN_9
#define MS5611_CS_GPIO_Port GPIOE
#define L_workingStatus_Pin GPIO_PIN_10
#define L_workingStatus_GPIO_Port GPIOE
#define Buzzer_Pin GPIO_PIN_8
#define Buzzer_GPIO_Port GPIOA
#define LED_RUN_Pin GPIO_PIN_3
#define LED_RUN_GPIO_Port GPIOB
#define LED_SENSORS_Pin GPIO_PIN_4
#define LED_SENSORS_GPIO_Port GPIOB
#define LED_SD_Pin GPIO_PIN_5
#define LED_SD_GPIO_Port GPIOB
#define LED_GNCU_Pin GPIO_PIN_6
#define LED_GNCU_GPIO_Port GPIOB
#define LED_CGU_Pin GPIO_PIN_7
#define LED_CGU_GPIO_Port GPIOB
#define LED_BOARD_Pin GPIO_PIN_3
#define LED_BOARD_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
/* On-board LED of the WeAct core board (PE3). These boards wire the LED
 * between 3V3 and the pin, so driving the pin LOW lights it. If yours lights
 * the other way round, set this to 0 - blinking works either way, only the
 * "solid on" states below are affected. */
#define LED_BOARD_ACTIVE_LOW    1

#if LED_BOARD_ACTIVE_LOW
#define LED_BOARD_ON            GPIO_PIN_RESET
#define LED_BOARD_OFF           GPIO_PIN_SET
#else
#define LED_BOARD_ON            GPIO_PIN_SET
#define LED_BOARD_OFF           GPIO_PIN_RESET
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
