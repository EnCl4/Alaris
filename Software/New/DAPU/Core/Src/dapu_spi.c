/**
  ******************************************************************************
  * @file    dapu_spi.c
  * @brief   Mutual exclusion for the shared SPI1 sensor bus.
  ******************************************************************************
  */

#include "main.h"
#include "cmsis_os.h"
#include "dapu_spi.h"

static osMutexId_t s_bus_mutex;
static const osMutexAttr_t s_bus_mutex_attr = { .name = "SpiBusMutex" };

void dapu_spi_init(void)
{
    s_bus_mutex = osMutexNew(&s_bus_mutex_attr);
}

void dapu_spi_lock(void)
{
    if (s_bus_mutex != NULL && osKernelGetState() == osKernelRunning)
    {
        osMutexAcquire(s_bus_mutex, osWaitForever);
    }
}

void dapu_spi_unlock(void)
{
    if (s_bus_mutex != NULL && osKernelGetState() == osKernelRunning)
    {
        osMutexRelease(s_bus_mutex);
    }
}
