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

#endif /* __DAPU_SPI_H */
