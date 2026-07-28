/**
  ******************************************************************************
  * @file    dapu_analog.h
  * @brief   Timer triggered ADC1 scan + conversion to engineering units.
  ******************************************************************************
  */

#ifndef __DAPU_ANALOG_H
#define __DAPU_ANALOG_H

#include <stdint.h>
#include <stdbool.h>

/** Starts TIM2 and the circular ADC1 DMA scan. Call before the scheduler. */
void  analog_start(void);

/** Checks that the scan is really being clocked: samples the DMA counter,
 *  waits settle_ms and samples it again. False means TIM2 is not producing
 *  TRGO or the DMA is not moving, in which case every analog channel would
 *  silently read a constant. */
bool  analog_is_running(uint32_t settle_ms);

/** Latest raw count of one channel (ADC_IDX_*). */
uint16_t analog_latest(uint8_t ch);

/** Boxcar average of the last n_scans samples of one channel, in counts. */
float analog_avg_counts(uint8_t ch, uint32_t n_scans);

/** Copies all 7 channels of the most recent scan. */
void  analog_latest_scan(uint16_t out[7]);

static inline float analog_counts_to_volts(float counts)
{
    return counts * (3.3f / 65535.0f);
}

/* --- Per task updates: each takes the state lock internally -------------- */
void  analog_update_pitot(void);   /* dp, IAS, TAS, air density */
void  analog_update_aoa(void);
void  analog_update_flex(void);    /* 3 control surfaces + battery */

/* --- Microphone block handoff to the RPM task --------------------------- */

/** Blocks until a fresh 1 s block is available, then de-interleaves it into
 *  dst (RPM_FFT_SIZE floats, raw ADC counts). false on timeout. */
bool  analog_mic_wait_block(float *dst, uint32_t timeout_ms);

/** Called from MX_FREERTOS_Init() to create the block semaphore. */
void  analog_init_rtos(void);

#endif /* __DAPU_ANALOG_H */
