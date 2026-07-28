/**
  ******************************************************************************
  * @file    icm_bus_selftest.h
  * @brief   Bare-metal ICM20948 bus bring-up test.
  ******************************************************************************
  * Runs from main(), before osKernelInitialize(), with nothing else in the
  * way: no FreeRTOS, no bus mutex, no user-bank handling, no driver.
  *
  * On I2C it scans the whole address range and then reads WHO_AM_I; on SPI it
  * sweeps all four modes at three clock speeds. If this cannot read 0xEA, no
  * amount of firmware above it will - the fault is the wiring, the power or
  * the module itself.
  *
  * Controlled by ICM_BUS_SELFTEST in dapu_config.h. Set it to 0 for normal
  * operation once the sensor answers.
  ******************************************************************************
  */

#ifndef __ICM_BUS_SELFTEST_H
#define __ICM_BUS_SELFTEST_H

/** Probes whichever bus the IMU is configured for, reports what it finds, then
 *  polls WHO_AM_I continuously so wires can be moved and the effect watched. */
void icm_bus_selftest_run(void);

#endif /* __ICM_BUS_SELFTEST_H */
