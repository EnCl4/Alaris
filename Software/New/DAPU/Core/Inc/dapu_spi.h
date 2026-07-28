/**
  ******************************************************************************
  * @file    dapu_spi.h
  * @brief   Mutual exclusion for the shared SPI1 sensor bus.
  *
  *          BMP280 (CS PE7), ICM20948 (CS PE8) and MS5611 (CS PE9) sit on the
  *          same bus and are driven from three different tasks, so every
  *          transaction must be bracketed by lock/unlock.
  ******************************************************************************
  */

#ifndef __DAPU_SPI_H
#define __DAPU_SPI_H

/** Creates the bus mutex. Call from MX_FREERTOS_Init(). */
void dapu_spi_init(void);

/** No-ops before the scheduler starts, so boot code can use them too. */
void dapu_spi_lock(void);
void dapu_spi_unlock(void);

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/** Largest single transaction: the BMP280 calibration block is 24 bytes plus
 *  the address byte. */
#define DAPU_SPI_MAX_XFER   40u

/** One complete full-duplex transaction: CS low, exchange `len` bytes, CS high.
 *
 *  Always use this rather than HAL_SPI_Transmit() followed by
 *  HAL_SPI_Receive(). On STM32H7 every HAL_SPI_* call disables the peripheral
 *  when it returns, so a split transfer stops and restarts SCK in the middle
 *  of a transaction with CS still asserted; the slave sees spurious clock
 *  edges and the data comes back shifted. The equivalent split pattern is
 *  harmless on the much simpler F4 SPI, which is why F4 drivers port badly.
 *
 *  `rx` may be NULL for write-only transactions. */
bool dapu_spi_txrx(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                   const uint8_t *tx, uint8_t *rx, uint16_t len);

/** Bench tool (console key 'd'). Reads each device's ID register twice: once
 *  with its CS asserted, once with every CS left high. A healthy slave stays
 *  off MISO when deselected, so the two results must differ - if they are
 *  identical the device is not driving the bus at all, which separates a dead
 *  chip select from a dead MISO connection. */
void dapu_spi_diagnose(void);

/** Bench tool (console key 'w'). Square-waves each chip select in turn at
 *  ~5 Hz so the physical wire can be identified with a multimeter or scope.
 *  Blocks for roughly 3 * ms_per_pin. */
void dapu_spi_cs_wiggle(uint32_t ms_per_pin);

#endif /* __DAPU_SPI_H */
