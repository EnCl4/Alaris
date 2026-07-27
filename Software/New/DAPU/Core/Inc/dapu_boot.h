/**
  ******************************************************************************
  * @file    dapu_boot.h
  * @brief   Sensor detection and stationary boot calibration.
  ******************************************************************************
  * These run from the Boot task, NOT from MX_FREERTOS_Init().
  *
  * Creating any RTOS object before osKernelStart() leaves interrupts masked:
  * the ARM_CM4F port starts with uxCriticalNesting = 0xaaaaaaaa and only zeroes
  * it in xPortStartScheduler(), so the taskEXIT_CRITICAL() inside osMutexNew()
  * decrements to a non-zero value and never restores BASEPRI. SysTick is
  * therefore masked, HAL_GetTick() stops advancing, and any HAL_Delay() placed
  * after the first osMutexNew() hangs forever.
  *
  * Everything that blocks or depends on the tick must run after the scheduler
  * has started. The Boot task does that work at the highest priority and the
  * other tasks gate on dapu_boot_wait().
  ******************************************************************************
  */

#ifndef __DAPU_BOOT_H
#define __DAPU_BOOT_H

#include <stdbool.h>

/** Probes every sensor and records the result in g_state.status.
 *  A missing sensor is logged and skipped, never fatal - a bench test with
 *  only the barometer connected must still run. */
void dapu_boot_init_sensors(void);

/** ~2 s stationary calibration, aircraft still and level:
 *   - residual gyro bias (subtracted by the IMU task)
 *   - barometric reference pressure of each altimeter -> h = 0
 *   - pitot zero offset voltage
 *  Corresponds to the Self-Test / Warmup parameter initialisation of the
 *  state machine described in Embarcado.tex. */
void dapu_boot_calibrate(void);

/** Full boot sequence, run once by the Boot task: starts the ADC scan, probes
 *  the sensors, calibrates, then marks the system ready. */
void dapu_boot_run(void);

/** True once dapu_boot_run() has finished. */
bool dapu_boot_complete(void);

/** Blocks until the boot sequence is finished. Every acquisition task calls
 *  this before touching a sensor. */
void dapu_boot_wait(void);

#endif /* __DAPU_BOOT_H */
