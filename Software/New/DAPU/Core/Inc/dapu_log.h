/**
  ******************************************************************************
  * @file    dapu_log.h
  * @brief   200 Hz binary flight log: state snapshot -> ring buffer -> SD card.
  ******************************************************************************
  * The 200 Hz sampler never touches FatFS. It packs one fixed size record and
  * pushes it into a lock free byte ring; a lower rate writer task drains the
  * ring into the card in multi kilobyte blocks. A stall of the SD card can
  * therefore never stretch the acquisition period - it can only, in the worst
  * case, overflow the ring, which is counted and flagged in the record status.
  ******************************************************************************
  */

#ifndef __DAPU_LOG_H
#define __DAPU_LOG_H

#include <stdint.h>
#include <stdbool.h>

/** Creates the RTOS objects. Call from MX_FREERTOS_Init(). */
void     log_init_rtos(void);

/** Mounts the card, picks the next free LOGnnnn.BIN and writes the header.
 *  Must run from a task (FatFS is reentrant through the RTOS). */
bool     log_open(void);

/** Flushes and closes the current file. Safe to call when nothing is open. */
void     log_close(void);

bool     log_is_open(void);

/** Packs one record from the shared state and pushes it into the ring.
 *  Called at DAPU_BASE_RATE_HZ. Never blocks. */
void     log_sample(void);

/** Drains the ring to the card. Blocks on the writer semaphore. */
void     log_service(void);

/** Records that could not be queued because the ring was full. */
uint32_t log_dropped_records(void);

/** Total records handed to the card so far. */
uint32_t log_written_records(void);

/** Name of the file currently open, or "" when closed. */
const char *log_filename(void);

#endif /* __DAPU_LOG_H */
