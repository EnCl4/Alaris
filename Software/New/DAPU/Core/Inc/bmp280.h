/**
  ******************************************************************************
  * @file    bmp280.h
  * @brief   Bosch BMP280 pressure/temperature sensor, SPI1 + CS on PE7.
  ******************************************************************************
  * The design document specifies a BMP390; this build targets the BMP280 that
  * is actually on the bench. Only this file and its .c change if the BMP390 is
  * fitted later - the task and the log record stay the same.
  ******************************************************************************
  */

#ifndef __BMP280_H
#define __BMP280_H

#include <stdint.h>
#include <stdbool.h>

/** Resets and configures the device. false if the chip ID does not answer. */
bool  bmp280_init(void);

/** true once bmp280_init() has succeeded. */
bool  bmp280_present(void);

/** Raw byte read back from the chip-ID register during the last init.
 *  0x58 = healthy BMP280. 0x00 means MISO is stuck low (no device driving,
 *  CS never asserted, or MISO not connected); 0xFF means it is stuck high or
 *  floating; anything else points at a clocking / SPI mode problem. */
uint8_t bmp280_chip_id(void);

/** One coherent burst read of pressure + temperature.
 *  @param temp_c  [degC]
 *  @param press_pa [Pa]
 *  @return false on a bus/format error. */
bool  bmp280_read(float *temp_c, float *press_pa);

/** Barometric altitude from pressure, relative to reference_pa. */
float bmp280_altitude(float press_pa, float reference_pa);

#endif /* __BMP280_H */
