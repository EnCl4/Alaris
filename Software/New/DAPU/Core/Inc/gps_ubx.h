/**
  ******************************************************************************
  * @file    gps_ubx.h
  * @brief   u-blox NEO-M8N on USART2, UBX binary protocol, NAV-PVT at 10 Hz.
  ******************************************************************************
  */

#ifndef __GPS_UBX_H
#define __GPS_UBX_H

#include <stdint.h>
#include <stdbool.h>

/** Configures the receiver (baud, message set, rate) and arms the DMA RX ring.
 *  Blocking, call before the scheduler starts.
 *  @return true if at least one valid UBX frame was seen afterwards. */
bool gps_init(void);

/** Drains the UART DMA ring and feeds the UBX parser. Call from the GPS task.
 *  Updates g_state whenever a complete NAV-PVT arrives. */
void gps_poll(void);

/** Number of NAV-PVT frames decoded since boot. */
uint32_t gps_frame_count(void);

/** Number of frames dropped on a checksum mismatch since boot. */
uint32_t gps_checksum_errors(void);

#endif /* __GPS_UBX_H */
