/**
  ******************************************************************************
  * @file    ms5611.h
  * @brief   TE MS5611-01BA03 barometer, SPI1 + CS on PE9.
  ******************************************************************************
  * The MS5611 has no continuous mode: each pressure (D1) and temperature (D2)
  * value needs its own command followed by a conversion delay (9.04 ms at
  * OSR 4096). The driver is therefore a small state machine stepped every
  * 10 ms, which yields a fully compensated sample every 20 ms = 50 Hz, the
  * rate required by Tab. Freqsensores, at maximum resolution.
  ******************************************************************************
  */

#ifndef __MS5611_H
#define __MS5611_H

#include <stdint.h>
#include <stdbool.h>

/** Resets the device, reads the PROM and validates its CRC4.
 *  false if the sensor is absent or the calibration data is corrupt. */
bool  ms5611_init(void);

/** true once ms5611_init() has succeeded. */
bool  ms5611_present(void);

/** Raw PROM word read during the last init, for diagnostics. All-zero or
 *  all-ones means the bus is not returning data at all; plausible-looking
 *  coefficients that still fail CRC point at a clocking problem. */
uint16_t ms5611_prom_word(uint8_t index);

/** Advance the conversion state machine. Call every 10 ms.
 *  @return true when a new compensated pair has just been produced. */
bool  ms5611_step(void);

/** Last compensated values, valid after ms5611_step() returned true. */
float ms5611_temperature_c(void);
float ms5611_pressure_pa(void);

/** Barometric altitude from pressure, relative to reference_pa. */
float ms5611_altitude(float press_pa, float reference_pa);

#endif /* __MS5611_H */
