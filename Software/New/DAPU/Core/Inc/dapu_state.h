/**
  ******************************************************************************
  * @file    dapu_state.h
  * @brief   Mutex protected state block + microsecond timebase.
  ******************************************************************************
  */

#ifndef __DAPU_STATE_H
#define __DAPU_STATE_H

#include "dapu_types.h"

/** Single copy of the live aircraft/sensor state. Always take the lock. */
extern dapu_state_t g_state;

/** Creates the mutex. Call once from MX_FREERTOS_Init(), before osKernelStart. */
void     dapu_state_init(void);

/** Before the scheduler runs these are no-ops, so boot code can use them too. */
void     dapu_state_lock(void);
void     dapu_state_unlock(void);

/** Sets/clears bits of g_state.status. Takes the lock internally. */
void     dapu_state_set_status(uint16_t bits);
void     dapu_state_clear_status(uint16_t bits);

/** Starts the DWT cycle counter. Call once, early in main(). */
void     dapu_micros_init(void);

/** Microseconds since dapu_micros_init(). Wraps every ~71 min.
 *  Must be called at least once every 21 s for the wrap tracking to work;
 *  the 200 Hz logger guarantees that. Safe from ISRs. */
uint32_t dapu_micros(void);

#endif /* __DAPU_STATE_H */
