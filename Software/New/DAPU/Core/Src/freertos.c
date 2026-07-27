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
#include "dapu_config.h"
#include "dapu_state.h"
#include "dapu_spi.h"
#include "dapu_analog.h"
#include "dapu_boot.h"
#include "dapu_log.h"
#include "dapu_console.h"
#include "bmp280.h"
#include "ms5611.h"
#include "icm20948.h"
#include "gps_ubx.h"
#include "rpm_fft.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Boot */
osThreadId_t BootHandle;
const osThreadAttr_t Boot_attributes = {
  .name = "Boot",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for SD_Writer */
osThreadId_t SD_WriterHandle;
const osThreadAttr_t SD_Writer_attributes = {
  .name = "SD_Writer",
  .stack_size = 1024 * 8,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for LogSample */
osThreadId_t LogSampleHandle;
const osThreadAttr_t LogSample_attributes = {
  .name = "LogSample",
  .stack_size = 512 * 8,
  .priority = (osPriority_t) osPriorityHigh6,
};
/* Definitions for GPS */
osThreadId_t GPSHandle;
const osThreadAttr_t GPS_attributes = {
  .name = "GPS",
  .stack_size = 512 * 8,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for IMU */
osThreadId_t IMUHandle;
const osThreadAttr_t IMU_attributes = {
  .name = "IMU",
  .stack_size = 512 * 8,
  .priority = (osPriority_t) osPriorityAboveNormal6,
};
/* Definitions for Pitot */
osThreadId_t PitotHandle;
const osThreadAttr_t Pitot_attributes = {
  .name = "Pitot",
  .stack_size = 256 * 8,
  .priority = (osPriority_t) osPriorityAboveNormal4,
};
/* Definitions for BMP280 */
osThreadId_t BMP280Handle;
const osThreadAttr_t BMP280_attributes = {
  .name = "BMP280",
  .stack_size = 384 * 8,
  .priority = (osPriority_t) osPriorityAboveNormal3,
};
/* Definitions for MS5611 */
osThreadId_t MS5611Handle;
const osThreadAttr_t MS5611_attributes = {
  .name = "MS5611",
  .stack_size = 384 * 8,
  .priority = (osPriority_t) osPriorityAboveNormal2,
};
/* Definitions for AOA */
osThreadId_t AOAHandle;
const osThreadAttr_t AOA_attributes = {
  .name = "AOA",
  .stack_size = 256 * 8,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for RPM */
osThreadId_t RPMHandle;
const osThreadAttr_t RPM_attributes = {
  .name = "RPM",
  .stack_size = 512 * 8,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Flex */
osThreadId_t FlexHandle;
const osThreadAttr_t Flex_attributes = {
  .name = "Flex",
  .stack_size = 256 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Console */
osThreadId_t ConsoleHandle;
const osThreadAttr_t Console_attributes = {
  .name = "Console",
  .stack_size = 1024 * 8,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void BootTask(void *argument);
void SD_WriterTask(void *argument);
void LogSampleTask(void *argument);
void GpsTask(void *argument);
void ImuTask(void *argument);
void PitotTask(void *argument);
void Bmp280Task(void *argument);
void Ms5611Task(void *argument);
void AoaTask(void *argument);
void RpmTask(void *argument);
void FlexTask(void *argument);
void ConsoleTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* CAUTION: nothing here may block or wait on HAL_GetTick().
   *
   * The first osMutexNew() below takes a FreeRTOS critical section, and the
   * ARM_CM4F port starts with uxCriticalNesting = 0xaaaaaaaa (port.c:146),
   * only zeroed by xPortStartScheduler(). The matching taskEXIT_CRITICAL()
   * therefore never restores BASEPRI, SysTick stays masked and HAL_GetTick()
   * stops advancing until osKernelStart() runs. Any HAL_Delay() placed after
   * this point hangs forever.
   *
   * Creating the RTOS objects is fine - it does not need the tick. Everything
   * that blocks lives in BootTask(), which runs once the scheduler is up. */
  dapu_state_init();
  dapu_spi_init();
  analog_init_rtos();
  log_init_rtos();
  console_init_rtos();
  rpm_fft_init();
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
  /* creation of Boot */
  BootHandle = osThreadNew(BootTask, NULL, &Boot_attributes);

  /* creation of SD_Writer */
  SD_WriterHandle = osThreadNew(SD_WriterTask, NULL, &SD_Writer_attributes);

  /* creation of LogSample */
  LogSampleHandle = osThreadNew(LogSampleTask, NULL, &LogSample_attributes);

  /* creation of GPS */
  GPSHandle = osThreadNew(GpsTask, NULL, &GPS_attributes);

  /* creation of IMU */
  IMUHandle = osThreadNew(ImuTask, NULL, &IMU_attributes);

  /* creation of Pitot */
  PitotHandle = osThreadNew(PitotTask, NULL, &Pitot_attributes);

  /* creation of BMP280 */
  BMP280Handle = osThreadNew(Bmp280Task, NULL, &BMP280_attributes);

  /* creation of MS5611 */
  MS5611Handle = osThreadNew(Ms5611Task, NULL, &MS5611_attributes);

  /* creation of AOA */
  AOAHandle = osThreadNew(AoaTask, NULL, &AOA_attributes);

  /* creation of RPM */
  RPMHandle = osThreadNew(RpmTask, NULL, &RPM_attributes);

  /* creation of Flex */
  FlexHandle = osThreadNew(FlexTask, NULL, &Flex_attributes);

  /* creation of Console */
  ConsoleHandle = osThreadNew(ConsoleTask, NULL, &Console_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* osThreadNew() returns NULL when the FreeRTOS heap is exhausted, and the
   * generated code above ignores that - the board would simply come up with
   * some tasks missing. Raise configTOTAL_HEAP_SIZE if this trips. */
  if ((BootHandle      == NULL) || (SD_WriterHandle == NULL) ||
      (LogSampleHandle == NULL) || (GPSHandle       == NULL) ||
      (IMUHandle       == NULL) || (PitotHandle     == NULL) ||
      (BMP280Handle    == NULL) || (MS5611Handle    == NULL) ||
      (AOAHandle       == NULL) || (RPMHandle       == NULL) ||
      (FlexHandle      == NULL) || (ConsoleHandle   == NULL))
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_BootTask */
/**
  * @brief  Function implementing the Boot thread.
  *         Runs the whole blocking start-up sequence - ADC scan, sensor
  *         detection, stationary calibration - at the highest priority, then
  *         releases the other tasks and exits. This cannot live in
  *         MX_FREERTOS_Init(): before osKernelStart() the FreeRTOS port leaves
  *         interrupts masked, so HAL_GetTick() never advances there.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_BootTask */
void BootTask(void *argument)
{
  /* USER CODE BEGIN BootTask */
  dapu_boot_run();

  /* One-shot: the boot sequence never needs to run again. */
  osThreadExit();
  /* USER CODE END BootTask */
}

/* USER CODE BEGIN Header_SD_WriterTask */
/**
  * @brief  Function implementing the SD_Writer thread.
  *         Priority 14 of Tab. RTOSPrioridadesDAPU: the log must never lose a
  *         sample because of anything else in the system. It only ever moves
  *         bytes out of the ring buffer, so a slow card cannot stretch the
  *         acquisition period.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_SD_WriterTask */
void SD_WriterTask(void *argument)
{
  /* USER CODE BEGIN SD_WriterTask */
  /* Sensors are not configured and the tick baseline is not
   * meaningful until the Boot task has finished. */
  dapu_boot_wait();

  /* FatFS mounting must happen from a task: its reentrancy layer uses an
   * RTOS semaphore that only works once the scheduler is running. */
  uint32_t retry_tick = 0;

  for(;;)
  {
    if (!log_is_open())
    {
      if ((HAL_GetTick() - retry_tick) >= 1000u)
      {
        retry_tick = HAL_GetTick();
        if (log_open())
        {
          printf("Logging to %s\r\n", log_filename());
        }
      }
      osDelay(50);
      continue;
    }

    log_service();
  }
  /* USER CODE END SD_WriterTask */
}

/* USER CODE BEGIN Header_LogSampleTask */
/**
  * @brief Function implementing the LogSample thread.
  *        Snapshots the shared state into one binary record at the 200 Hz base
  *        rate of the DAPU and pushes it into the ring buffer.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_LogSampleTask */
void LogSampleTask(void *argument)
{
  /* USER CODE BEGIN LogSampleTask */
  /* Sensors are not configured and the tick baseline is not
   * meaningful until the Boot task has finished. */
  dapu_boot_wait();

  uint32_t tick = osKernelGetTickCount();
  uint32_t blink = 0;

  for(;;)
  {
    tick += TASK_PERIOD_LOG_MS;
    osDelayUntil(tick);

    log_sample();

    /* 1 Hz heartbeat, mirrored on the on-board LED so the board can be
     * checked with nothing wired to it. */
    if (++blink >= (DAPU_BASE_RATE_HZ / 2u))
    {
      blink = 0;
      HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
      HAL_GPIO_TogglePin(LED_BOARD_GPIO_Port, LED_BOARD_Pin);
    }
  }
  /* USER CODE END LogSampleTask */
}

/* USER CODE BEGIN Header_GpsTask */
/**
  * @brief Function implementing the GPS thread.
  *        Drains the USART2 DMA ring; NAV-PVT arrives at 10 Hz but the ring is
  *        polled at 100 Hz so it can never wrap between two visits.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_GpsTask */
void GpsTask(void *argument)
{
  /* USER CODE BEGIN GpsTask */
  /* Sensors are not configured and the tick baseline is not
   * meaningful until the Boot task has finished. */
  dapu_boot_wait();

  uint32_t tick = osKernelGetTickCount();

  for(;;)
  {
    tick += TASK_PERIOD_GPS_MS;
    osDelayUntil(tick);

    gps_poll();

    dapu_state_lock();
    bool stale = (HAL_GetTick() - g_state.gps_last_tick) > 2000u;
    if (stale) { g_state.status |= DAPU_ST_GPS_TIMEOUT; }
    else       { g_state.status &= (uint16_t)~DAPU_ST_GPS_TIMEOUT; }
    dapu_state_unlock();
  }
  /* USER CODE END GpsTask */
}

/* USER CODE BEGIN Header_ImuTask */
/**
  * @brief Function implementing the IMU thread.
  *        Accelerometer and gyroscope at 200 Hz, magnetometer at 100 Hz which
  *        is the AK09916 native rate (Tab. Freqsensores).
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_ImuTask */
void ImuTask(void *argument)
{
  /* USER CODE BEGIN ImuTask */
  /* Sensors are not configured and the tick baseline is not
   * meaningful until the Boot task has finished. */
  dapu_boot_wait();

  uint32_t tick = osKernelGetTickCount();
  bool mag_turn = false;
  axises accel, gyro, mag;

  for(;;)
  {
    tick += TASK_PERIOD_IMU_MS;
    osDelayUntil(tick);

    if (icm20948_read_imu(&accel, &gyro))
    {
      dapu_state_lock();
      g_state.accel_g[0] = accel.x;
      g_state.accel_g[1] = accel.y;
      g_state.accel_g[2] = accel.z;
      g_state.gyro_dps[0] = gyro.x - g_state.gyro_bias_dps[0];
      g_state.gyro_dps[1] = gyro.y - g_state.gyro_bias_dps[1];
      g_state.gyro_dps[2] = gyro.z - g_state.gyro_bias_dps[2];
      g_state.imu_seq++;
      g_state.imu_last_tick = HAL_GetTick();
      g_state.status &= (uint16_t)~DAPU_ST_IMU_TIMEOUT;
      dapu_state_unlock();
    }
    else
    {
      dapu_state_set_status(DAPU_ST_IMU_TIMEOUT);
    }

    mag_turn = !mag_turn;
    if (mag_turn && ak09916_read_mag_ut(&mag))
    {
      dapu_state_lock();
      g_state.mag_ut[0] = mag.x;
      g_state.mag_ut[1] = mag.y;
      g_state.mag_ut[2] = mag.z;
      g_state.mag_seq++;
      dapu_state_unlock();
    }
  }
  /* USER CODE END ImuTask */
}

/* USER CODE BEGIN Header_PitotTask */
/**
  * @brief Function implementing the Pitot thread.
  *        200 Hz differential pressure -> IAS/TAS, density corrected with the
  *        barometric measurement as required by RA.03.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_PitotTask */
void PitotTask(void *argument)
{
  /* USER CODE BEGIN PitotTask */
  /* Sensors are not configured and the tick baseline is not
   * meaningful until the Boot task has finished. */
  dapu_boot_wait();

  uint32_t tick = osKernelGetTickCount();

  for(;;)
  {
    tick += TASK_PERIOD_PITOT_MS;
    osDelayUntil(tick);

    analog_update_pitot();
  }
  /* USER CODE END PitotTask */
}

/* USER CODE BEGIN Header_Bmp280Task */
/**
  * @brief Function implementing the BMP280 thread. 100 Hz.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_Bmp280Task */
void Bmp280Task(void *argument)
{
  /* USER CODE BEGIN Bmp280Task */
  /* Sensors are not configured and the tick baseline is not
   * meaningful until the Boot task has finished. */
  dapu_boot_wait();

  uint32_t tick = osKernelGetTickCount();
  float temp_c, press_pa;

  for(;;)
  {
    tick += TASK_PERIOD_BMP_MS;
    osDelayUntil(tick);

    if (!bmp280_present())
    {
      continue;
    }

    if (bmp280_read(&temp_c, &press_pa))
    {
      dapu_state_lock();
      g_state.bmp_temp_c   = temp_c;
      g_state.bmp_press_pa = press_pa;
      g_state.bmp_alt_m    = bmp280_altitude(press_pa, g_state.bmp_press_ref_pa);
      g_state.bmp_seq++;
      g_state.bmp_last_tick = HAL_GetTick();
      dapu_state_unlock();
    }
  }
  /* USER CODE END Bmp280Task */
}

/* USER CODE BEGIN Header_Ms5611Task */
/**
  * @brief Function implementing the MS5611 thread.
  *        Stepped every 10 ms; the D1/D2 conversion pair yields one fully
  *        compensated sample every 20 ms = 50 Hz (Tab. Freqsensores).
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_Ms5611Task */
void Ms5611Task(void *argument)
{
  /* USER CODE BEGIN Ms5611Task */
  /* Sensors are not configured and the tick baseline is not
   * meaningful until the Boot task has finished. */
  dapu_boot_wait();

  uint32_t tick = osKernelGetTickCount();

  for(;;)
  {
    tick += TASK_PERIOD_MS5611_MS;
    osDelayUntil(tick);

    if (!ms5611_present())
    {
      continue;
    }

    if (ms5611_step())
    {
      float press_pa = ms5611_pressure_pa();

      dapu_state_lock();
      g_state.ms_temp_c   = ms5611_temperature_c();
      g_state.ms_press_pa = press_pa;
      g_state.ms_alt_m    = ms5611_altitude(press_pa, g_state.ms_press_ref_pa);
      g_state.ms_seq++;
      g_state.ms_last_tick = HAL_GetTick();
      dapu_state_unlock();
    }
  }
  /* USER CODE END Ms5611Task */
}

/* USER CODE BEGIN Header_AoaTask */
/**
  * @brief Function implementing the AOA thread. 200 Hz.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_AoaTask */
void AoaTask(void *argument)
{
  /* USER CODE BEGIN AoaTask */
  /* Sensors are not configured and the tick baseline is not
   * meaningful until the Boot task has finished. */
  dapu_boot_wait();

  uint32_t tick = osKernelGetTickCount();

  for(;;)
  {
    tick += TASK_PERIOD_AOA_MS;
    osDelayUntil(tick);

    analog_update_aoa();
  }
  /* USER CODE END AoaTask */
}

/* USER CODE BEGIN Header_RpmTask */
/**
  * @brief Function implementing the RPM thread.
  *        Blocks until the ADC DMA hands over a full 1 s microphone block,
  *        then runs the 1024 point FFT. Effective output rate is therefore
  *        ~1 Hz, as derived in Embarcado.tex.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_RpmTask */
void RpmTask(void *argument)
{
  /* USER CODE BEGIN RpmTask */
  /* Sensors are not configured and the tick baseline is not
   * meaningful until the Boot task has finished. */
  dapu_boot_wait();

  float *samples = rpm_fft_sample_buffer();
  float blade_hz, rpm, snr;

  for(;;)
  {
    if (!analog_mic_wait_block(samples, 3000u))
    {
      continue;                 /* ADC stalled - nothing to do this round */
    }

    bool valid = rpm_fft_process(samples, &blade_hz, &rpm, &snr);

    dapu_state_lock();
    g_state.blade_hz = blade_hz;
    g_state.rpm      = rpm;
    g_state.thrust_n = valid ? rpm_to_thrust_n(rpm) : 0.0f;
    g_state.rpm_snr  = snr;
    g_state.rpm_seq++;
    if (valid) { g_state.status |= DAPU_ST_RPM_VALID; }
    else       { g_state.status &= (uint16_t)~DAPU_ST_RPM_VALID; }
    dapu_state_unlock();
  }
  /* USER CODE END RpmTask */
}

/* USER CODE BEGIN Header_FlexTask */
/**
  * @brief Function implementing the Flex thread.
  *        Control surface deflections plus the battery voltage, 200 Hz.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_FlexTask */
void FlexTask(void *argument)
{
  /* USER CODE BEGIN FlexTask */
  /* Sensors are not configured and the tick baseline is not
   * meaningful until the Boot task has finished. */
  dapu_boot_wait();

  uint32_t tick = osKernelGetTickCount();

  for(;;)
  {
    tick += TASK_PERIOD_FLEX_MS;
    osDelayUntil(tick);

    analog_update_flex();
  }
  /* USER CODE END FlexTask */
}

/* USER CODE BEGIN Header_ConsoleTask */
/**
  * @brief Function implementing the Console thread.
  *        Lowest priority on purpose: the live plot must never take CPU away
  *        from the SD writer, which carries the data that matters.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_ConsoleTask */
void ConsoleTask(void *argument)
{
  /* USER CODE BEGIN ConsoleTask */
  /* Sensors are not configured and the tick baseline is not
   * meaningful until the Boot task has finished. */
  dapu_boot_wait();

  uint32_t tick = osKernelGetTickCount();

  for(;;)
  {
    tick += TASK_PERIOD_PLOT_MS;
    osDelayUntil(tick);

    console_plot();
  }
  /* USER CODE END ConsoleTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  Called by FreeRTOS when a task overruns its stack.
  *         4 blinks per group on the on-board LED.
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;   /* read this in the debugger to see which task it was */
  taskDISABLE_INTERRUPTS();
  DAPU_BlinkForever(4);
}

/**
  * @brief  Called by FreeRTOS when pvPortMalloc() cannot satisfy a request.
  *         5 blinks per group - raise configTOTAL_HEAP_SIZE.
  */
void vApplicationMallocFailedHook(void)
{
  taskDISABLE_INTERRUPTS();
  DAPU_BlinkForever(5);
}

/* USER CODE END Application */
