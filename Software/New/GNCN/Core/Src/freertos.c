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
#include <stdlib.h>
#include <stdio.h>
#include "usart.h"
#include "tim.h"
#include "math.h"
#include "i2c.h"
#include <stdbool.h>
#include "spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
bool debug = 1;

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}


extern uint8_t rx_data[8];
extern uint8_t tx_data[8];  // response (optional)

extern uint8_t counter;

extern uint16_t x = 0;
extern uint16_t y = 0;
extern uint16_t z = 0;

extern bool connected;

volatile uint32_t rise = 0;
volatile uint32_t fall = 0;
volatile uint32_t pulse_width = 0;
volatile uint8_t is_rising = 1;

volatile uint32_t rise4 = 0;
volatile uint32_t fall4 = 0;
volatile uint32_t pulse_width4 = 0;
volatile uint8_t is_rising4 = 1;



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) //é melhor usar CASE e switch aqui
{

    if (htim->Instance == TIM1 &&
        htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint32_t val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (is_rising)
        {
            rise = val;

            // switch to falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                                         TIM_CHANNEL_1,
                                         TIM_INPUTCHANNELPOLARITY_FALLING);

            is_rising = 0;
        }
        else
        {
            fall = val;

            // handle overflow
            if (fall >= rise)
                pulse_width = fall - rise;

            else
                pulse_width = (htim->Instance->ARR - rise + fall);

            // switch back to rising
            __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                                         TIM_CHANNEL_1,
                                         TIM_INPUTCHANNELPOLARITY_RISING);

            is_rising = 1;
        }
    }

    if (htim->Instance == TIM4 &&
            htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            uint32_t val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            if (is_rising4)
            {
                rise4 = val;

                // switch to falling edge
                __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                                             TIM_CHANNEL_1,
                                             TIM_INPUTCHANNELPOLARITY_FALLING);

                is_rising4 = 0;
            }
            else
            {
                fall4 = val;

                // handle overflow
                if (fall4 >= rise4)
                    pulse_width4 = fall4 - rise4;
                else
                    pulse_width4 = (htim->Instance->ARR - rise4 + fall4);

                // switch back to rising
                __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                                             TIM_CHANNEL_1,
                                             TIM_INPUTCHANNELPOLARITY_RISING);

                is_rising4 = 1;
            }
        }

}


float s1 = 0;
float s2 = 0;

void PWM_set(void){




}

typedef struct
  {
      float pulse_us;
      float period_us;
      float duty;

      uint32_t last_update_tick;
  } PWM_Channel_t;

float phi;
float xi;
float theta;
float beta;
float h;
float phi_c = 0;
float Xc = 0;

float xi_inf = 3.141592653/2;  //colocar pi real
float K_path = 0.02;

float p[3] = {x, y, z};
float way_points[3][3] = {{3, 3, 3},{3, 3, 3},{3, 3, 3}};  //pegar as coordenadas dos waypoints com o bohn
float r[3] = {0, 0, 0};
float q[3] = {0, 0, 0};

#define PWM_CHANNELS 6
volatile PWM_Channel_t pwm_read[PWM_CHANNELS];


uint8_t Buffer[25] = {0};
uint8_t Space[] = " - ";
uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
uint8_t EndMSG[] = "Done! \r\n\r\n";

void I2C_Scan(void)
{
    HAL_StatusTypeDef result;
    uint8_t i;

    printf("\r\n--- I2C SCAN START ---\r\n");

    for (i = 1; i < 128; i++)
    {
        result = HAL_I2C_IsDeviceReady(
                    &hi2c1,
                    (uint16_t)(i << 1),
                    2,
                    5);

        if (result == HAL_OK)
        {
            printf("I2C device found at address: 0x%02X\r\n", i);
        }
    }

    printf("--- I2C SCAN END ---\r\n");
}

typedef struct {
    float heading;
    float roll;
    float pitch;
} BNO055_Euler_t;

#define BNO055_ADDR (0x28 << 1)   // or 0x29 << 1 depending on ADR pin
#define BNO055_EULER_H_LSB 0x1A

BNO055_Euler_t BNO055_ReadEuler(I2C_HandleTypeDef *hi2c)
{
    uint8_t buffer[6];
    BNO055_Euler_t euler = {0};

    if (HAL_I2C_Mem_Read(hi2c,
                         BNO055_ADDR,
                         BNO055_EULER_H_LSB,
                         I2C_MEMADD_SIZE_8BIT,
                         buffer,
                         6,
                         100) != HAL_OK)
    {
        // optional: handle error here
        return euler;
    }

    int16_t heading_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t roll_raw    = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t pitch_raw   = (int16_t)((buffer[5] << 8) | buffer[4]);

    euler.heading = heading_raw / 16.0f;
    euler.roll    = roll_raw    / 16.0f;
    euler.pitch   = pitch_raw   / 16.0f;

    return euler;
}


BNO055_Euler_t e;


  typedef struct
  {
      float Kp;
      float Ki;
      float Kd;

      float integrator;
      float prev_error;

      float output_min;
      float output_max;

      float dt;   // sampling time (seconds)
  } PID_t;


PID_t deflexaoAileron;
PID_t roll_pid;
PID_t pitch_pid;
PID_t yaw_pid;
PID_t thr_pid;


  float PID_Update(PID_t *pid, float setpoint, float measurement)
  {
      float error = setpoint - measurement;

      // Proportional
      float P = pid->Kp * error;

      // Integral (with anti-windup clamp)
      pid->integrator += pid->Ki * error * pid->dt;

      if(pid->integrator > pid->output_max)
          pid->integrator = pid->output_max;
      else if(pid->integrator < pid->output_min)
          pid->integrator = pid->output_min;

      // Derivative (on error)
      float derivative = (error - pid->prev_error) / pid->dt;
      float D = pid->Kd * derivative;

      // Total output
      float output = P + pid->integrator + D;

      // Output saturation
      if(output > pid->output_max)
          output = pid->output_max;
      else if(output < pid->output_min)
          output = pid->output_min;

      // Save state
      pid->prev_error = error;

      return output;
  }


 float produto_escalar(const float v1[], const float v2[], int tamanho) {
      float resultado = 0.0;
      for (int i = 0; i < tamanho; i++) {
          resultado += v1[i] * v2[i];
      }
      return resultado;
  }


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for getPWM */
osThreadId_t getPWMHandle;
const osThreadAttr_t getPWM_attributes = {
  .name = "getPWM",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BNOAccel */
osThreadId_t BNOAccelHandle;
const osThreadAttr_t BNOAccel_attributes = {
  .name = "BNOAccel",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Print */
osThreadId_t PrintHandle;
const osThreadAttr_t Print_attributes = {
  .name = "Print",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for PhiControll */
osThreadId_t PhiControllHandle;
const osThreadAttr_t PhiControll_attributes = {
  .name = "PhiControll",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for SideSlipControl */
osThreadId_t SideSlipControlHandle;
const osThreadAttr_t SideSlipControl_attributes = {
  .name = "SideSlipControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for PitchControll */
osThreadId_t PitchControllHandle;
const osThreadAttr_t PitchControll_attributes = {
  .name = "PitchControll",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh5,
};
/* Definitions for AltitudeControl */
osThreadId_t AltitudeControlHandle;
const osThreadAttr_t AltitudeControl_attributes = {
  .name = "AltitudeControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for AirSpeedControl */
osThreadId_t AirSpeedControlHandle;
const osThreadAttr_t AirSpeedControl_attributes = {
  .name = "AirSpeedControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh6,
};
/* Definitions for HorizontalNav */
osThreadId_t HorizontalNavHandle;
const osThreadAttr_t HorizontalNav_attributes = {
  .name = "HorizontalNav",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for WayPointsTreat */
osThreadId_t WayPointsTreatHandle;
const osThreadAttr_t WayPointsTreat_attributes = {
  .name = "WayPointsTreat",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void getPWMTask(void *argument);
void BNOAccelRun(void *argument);
void PrintDebug(void *argument);
void StartPhiControll(void *argument);
void StartSideSlipControll(void *argument);
void StartPitchControll(void *argument);
void StartAltitudeControll(void *argument);
void StartAirSpeedControll(void *argument);
void StartHorizontalNav(void *argument);
void StartWayPointsTreat(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
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
  /* creation of getPWM */
  getPWMHandle = osThreadNew(getPWMTask, NULL, &getPWM_attributes);

  /* creation of BNOAccel */
  BNOAccelHandle = osThreadNew(BNOAccelRun, NULL, &BNOAccel_attributes);

  /* creation of Print */
  PrintHandle = osThreadNew(PrintDebug, NULL, &Print_attributes);

  /* creation of PhiControll */
  PhiControllHandle = osThreadNew(StartPhiControll, NULL, &PhiControll_attributes);

  /* creation of SideSlipControl */
  SideSlipControlHandle = osThreadNew(StartSideSlipControll, NULL, &SideSlipControl_attributes);

  /* creation of PitchControll */
  PitchControllHandle = osThreadNew(StartPitchControll, NULL, &PitchControll_attributes);

  /* creation of AltitudeControl */
  AltitudeControlHandle = osThreadNew(StartAltitudeControll, NULL, &AltitudeControl_attributes);

  /* creation of AirSpeedControl */
  AirSpeedControlHandle = osThreadNew(StartAirSpeedControll, NULL, &AirSpeedControl_attributes);

  /* creation of HorizontalNav */
  HorizontalNavHandle = osThreadNew(StartHorizontalNav, NULL, &HorizontalNav_attributes);

  /* creation of WayPointsTreat */
  WayPointsTreatHandle = osThreadNew(StartWayPointsTreat, NULL, &WayPointsTreat_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_getPWMTask */
/**
  * @brief  Function implementing the getPWM thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_getPWMTask */
void getPWMTask(void *argument)
{
  /* USER CODE BEGIN getPWMTask */
	int x = 0;



  /* Infinite loop */
  for(;;)
  {


	  x++;
	  if (x == 1000){
		  x = 0;
	  }
	  s1 = 10000*sinf(2 * M_PI * 15 * x / 1000);
	  s2 = 500*sinf(2 * M_PI * 2 * x / 1000);

	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (1500 + s2));
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (10000 + s1));


	  pwm_read[0].period_us = 20000;
	  pwm_read[0].pulse_us = pulse_width4;
	  pwm_read[0].duty = pwm_read[0].pulse_us/pwm_read[0].period_us;

	  pwm_read[1].period_us = 20000;
	  pwm_read[1].pulse_us = pulse_width;
	  pwm_read[1].duty = pwm_read[1].pulse_us/pwm_read[1].period_us;


	  pwm_read[2].period_us = 20000;
	  pwm_read[2].pulse_us = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_2);
	  pwm_read[2].duty = pwm_read[2].pulse_us/pwm_read[2].period_us;

	  pwm_read[3].period_us = 20000;
	  pwm_read[3].pulse_us = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
	  pwm_read[3].duty = pwm_read[3].pulse_us/pwm_read[3].period_us;

	  pwm_read[4].period_us = 20000;
	  pwm_read[4].pulse_us = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
	  pwm_read[4].duty = pwm_read[4].pulse_us/pwm_read[4].period_us;

	  pwm_read[5].period_us = 20000;
	  pwm_read[5].pulse_us = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_2);
	  pwm_read[5].duty = pwm_read[5].pulse_us/pwm_read[5].period_us;


	  //printf("CONTROLE\r\n");

	  osDelay(20);
  }
  /* USER CODE END getPWMTask */
}

/* USER CODE BEGIN Header_BNOAccelRun */
/**
* @brief Function implementing the BNOAccel thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BNOAccelRun */
void BNOAccelRun(void *argument)
{
  /* USER CODE BEGIN BNOAccelRun */
	//bno055_assignI2C(&hi2c1);
	//bno055_setup();
	//bno055_setOperationModeNDOF();

	// after power-up
	osDelay(700);

	// config mode
	uint8_t mode = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_ADDR, 0x3D, 1, &mode, 1, 100);
	osDelay(20);

	// NDOF mode
	mode = 0x0C;
	HAL_I2C_Mem_Write(&hi2c1, BNO055_ADDR, 0x3D, 1, &mode, 1, 100);
	osDelay(20);


  /* Infinite loop */
  for(;;)
  {


	  if(connected == 0){
		  e = BNO055_ReadEuler(&hi2c1);
		  e.pitch = e.pitch*(-1);
		  phi = e.roll; //passar isso para rad
	  }
	  //I2C_Scan();
	//bno055_vector_t v = bno055_getVectorEuler();
	//printf("\n >Heading:%.2f \n>Roll:%.2f\n>Pitch:%.2f\r\n", v.x, v.y, v.z);
    osDelay(10);
  }
  /* USER CODE END BNOAccelRun */
}

/* USER CODE BEGIN Header_PrintDebug */
/**
* @brief Function implementing the Print thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PrintDebug */
void PrintDebug(void *argument)
{
  /* USER CODE BEGIN PrintDebug */
  if (debug == 1){
  /* Infinite loop */
  for(;;)
  {
	  printf(">duty0:%.5f\r\n", pwm_read[0].duty);
	  printf(">duty1:%.5f\r\n", pwm_read[1].duty);
	  printf(">duty2:%.3f\r\n", pwm_read[2].duty);
	  printf(">duty3:%.3f\r\n", pwm_read[3].duty);
	  printf(">duty4:%.3f\r\n", pwm_read[4].duty);
	  printf(">duty5:%.5f\r\n", pwm_read[5].duty);

	  printf(">sin2:%.5f\r\n", 1500 + s2);
	  printf(">sin1:%.5f\r\n", 1500 + s1);
	  printf(">counter:%d\r\n", counter);
//	  printf("\nRX: ");
//	  for (int i = 0; i < 8; i++)
//	  {
//	      printf("%02X ", rx_data[i]);
//	  }
//	  printf("\r\n");
	  printf("\n\r\n>H:%.2f\r\n>R:%.2f\r\n>P:%.2f\r\n\n", e.heading, e.roll, e.pitch);

	  osDelay(20);
  }}
  else{
	  osDelay(100000);
  }
  /* USER CODE END PrintDebug */
}

/* USER CODE BEGIN Header_StartPhiControll */
/**
* @brief Function implementing the PhiControll thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPhiControll */
void StartPhiControll(void *argument)
{
  /* USER CODE BEGIN StartPhiControll */
  roll_pid.Kp = 1;
  roll_pid.Ki = 1;
  roll_pid.Kd = 1;

  roll_pid.output_max = 40;
  roll_pid.output_min = -40;


  deflexaoAileron.Kp = 1;
  deflexaoAileron.Ki = 1;
  deflexaoAileron.Kd = 1;

  deflexaoAileron.output_max = 30;
  deflexaoAileron.output_min = -30;



  /* Infinite loop */
  for(;;)
  {
	  roll_pid.dt = 0.01;
	  phi_c = PID_Update(&roll_pid, Xc, X);


	  deflexaoAileron.dt = 0.01; //segundos
	  float deflexaoa = PID_Update(&deflexaoAileron, phi_c, phi);
	  osDelay(10); //100Hz
  }
  /* USER CODE END StartPhiControll */
}

/* USER CODE BEGIN Header_StartSideSlipControll */
/**
* @brief Function implementing the SideSlipControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSideSlipControll */
void StartSideSlipControll(void *argument)
{
  /* USER CODE BEGIN StartSideSlipControll */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSideSlipControll */
}

/* USER CODE BEGIN Header_StartPitchControll */
/**
* @brief Function implementing the PitchControll thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPitchControll */
void StartPitchControll(void *argument)
{
  /* USER CODE BEGIN StartPitchControll */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartPitchControll */
}

/* USER CODE BEGIN Header_StartAltitudeControll */
/**
* @brief Function implementing the AltitudeControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAltitudeControll */
void StartAltitudeControll(void *argument)
{
  /* USER CODE BEGIN StartAltitudeControll */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartAltitudeControll */
}

/* USER CODE BEGIN Header_StartAirSpeedControll */
/**
* @brief Function implementing the AirSpeedControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAirSpeedControll */
void StartAirSpeedControll(void *argument)
{
  /* USER CODE BEGIN StartAirSpeedControll */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartAirSpeedControll */
}

/* USER CODE BEGIN Header_StartHorizontalNav */
/**
* @brief Function implementing the HorizontalNav thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHorizontalNav */
void StartHorizontalNav(void *argument)
{
  /* USER CODE BEGIN StartHorizontalNav */
  /* Infinite loop */
  for(;;)
  {
	  float Xq = atan2(q[1], q[0]);
	  float epy = - sin(Xq) * (p[0] - r[0]) + cos(Xq) * (p[1] - r[1]);
	  Xc = Xq - xi_inf*(2/3.141592653)*atan(K_path*epy);
    osDelay(10);
  }
  /* USER CODE END StartHorizontalNav */
}

/* USER CODE BEGIN Header_StartWayPointsTreat */
/**
* @brief Function implementing the WayPointsTreat thread.
* @param argument: Not used
* @retval None
*/

bool trocarWayPoint = 0;

/* USER CODE END Header_StartWayPointsTreat */
void StartWayPointsTreat(void *argument)
{
  /* USER CODE BEGIN StartWayPointsTreat */
  int i = 1;
  float n[3] = {0, 0, 0};
  /* Infinite loop */
  for(;;)
  {
	  r = way_points[i-1]; //talvez mudar para o modo feio
	  if(trocarWayPoint == 1){

		  float aux2[3] = {way_points[i][0] - way_points[i-1][0], way_points[i][1] - way_points[i-1][1], way_points[i][2] - way_points[i-1][2]};
		  float norma_q = sqrt(pow(aux2[0],2) + pow(aux2[1],2) + pow(aux2[2],2));
		  float q_i_m1[3] = {aux2[0]/norma_q, aux2[1]/norma_q, aux2[2]/norma_q};
		  q[0] = q_i_m1[0];
		  q[1] = q_i_m1[1];
		  q[2] = q_i_m1[2];
		  float aux3[3] = {way_points[i+1][0] - way_points[i][0], way_points[i+1][1] - way_points[i][1], way_points[i+1][2] - way_points[i][2]};
		  norma_q = sqrt(pow(aux3[0],2) + pow(aux3[1],2) + pow(aux3[2],2));
		  float q_1[3] = {aux3[0]/norma_q, aux3[1]/norma_q, aux3[2]/norma_q};
		  float aux4[3] = {q_i_m1[0] + q_1[0], q_i_m1[1] + q_1[1], q_i_m1[2] + q_1[2]};
		  norma_q = sqrt(pow(aux4[0],2) + pow(aux4[1],2) + pow(aux4[2],2));
		  n[0] = aux4[0]/norma_q;
		  n[1] = aux4[1]/norma_q;
		  n[2] = aux4[2]/norma_q;
		  trocarWayPoint = 0;
	  }

	  float aux[3] = {p[0] - r[0], p[1] - r[1], p[2] - r[2]};
	  float H = produto_escalar(aux, n, 3);

	  if (H >= 0){
		i++;
		trocarWayPoint = 1;
	  }

    osDelay(10);
  }
  /* USER CODE END StartWayPointsTreat */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

