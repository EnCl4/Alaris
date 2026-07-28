/**
  ******************************************************************************
  * @file    dapu_console.h
  * @brief   USART1 console: printf retarget, live plot, single key commands.
  ******************************************************************************
  * Commands (send one character on USART1 at 115200):
  *   p  toggle the live plot   - turn it OFF while recording the data set that
  *                               matters, so the SD writer keeps the CPU
  *   s  print a one line status summary
  *   q  flush and close the current log file (safe removal of the card)
  *   r  open a new log file
  *   d  probe the SPI1 bus: read every sensor ID with CS asserted and with
  *      CS idle, which separates a dead chip select from a dead MISO
  *   w  square-wave each chip select in turn so the wire can be traced
  ******************************************************************************
  */

#ifndef __DAPU_CONSOLE_H
#define __DAPU_CONSOLE_H

#include <stdbool.h>

/** Creates the UART mutex and arms the 1 byte command receive. */
void console_init_rtos(void);

/** Emits one Teleplot style sample line, if the plot is enabled. */
void console_plot(void);

bool console_plot_enabled(void);

/** Prints the acquisition rate / status summary used during bench tests. */
void console_print_status(void);

#endif /* __DAPU_CONSOLE_H */
