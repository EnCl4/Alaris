/**
  ******************************************************************************
  * @file    dapu_state.c
  * @brief   Mutex protected state block + DWT based microsecond timebase.
  ******************************************************************************
  */

#include "main.h"
#include "cmsis_os.h"
#include "dapu_state.h"

dapu_state_t g_state;

/* ==========================================================================
 * State mutex
 * ========================================================================== */

static osMutexId_t s_state_mutex;
static const osMutexAttr_t s_state_mutex_attr = { .name = "StateMutex" };

void dapu_state_init(void)
{
    s_state_mutex = osMutexNew(&s_state_mutex_attr);
}

void dapu_state_lock(void)
{
    /* Boot calibration runs before osKernelStart(); there is no concurrency
     * to protect against yet and osMutexAcquire() would fail anyway. */
    if (s_state_mutex != NULL && osKernelGetState() == osKernelRunning)
    {
        osMutexAcquire(s_state_mutex, osWaitForever);
    }
}

void dapu_state_unlock(void)
{
    if (s_state_mutex != NULL && osKernelGetState() == osKernelRunning)
    {
        osMutexRelease(s_state_mutex);
    }
}

void dapu_state_set_status(uint16_t bits)
{
    dapu_state_lock();
    g_state.status |= bits;
    dapu_state_unlock();
}

void dapu_state_clear_status(uint16_t bits)
{
    dapu_state_lock();
    g_state.status &= (uint16_t)~bits;
    dapu_state_unlock();
}

/* ==========================================================================
 * Microsecond timebase (DWT cycle counter)
 * ==========================================================================
 * SystemCoreClock is 200 MHz, so exactly 200 cycles per microsecond and the
 * remainder bookkeeping below is exact.
 */

#define DWT_LAR_ADDR    (*(volatile uint32_t *)0xE0001FB0u)
#define DWT_UNLOCK_KEY  0xC5ACCE55u

static volatile uint32_t s_cycles_per_us = 200u;
static volatile uint32_t s_last_cyc;
static volatile uint32_t s_us_accum;
static volatile uint32_t s_cyc_rem;

void dapu_micros_init(void)
{
    s_cycles_per_us = SystemCoreClock / 1000000u;
    if (s_cycles_per_us == 0u)
    {
        s_cycles_per_us = 1u;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT_LAR_ADDR = DWT_UNLOCK_KEY;      /* Cortex-M7 requires the unlock */
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    s_last_cyc = DWT->CYCCNT;
    s_us_accum = 0u;
    s_cyc_rem  = 0u;
}

uint32_t dapu_micros(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t now   = DWT->CYCCNT;
    uint32_t delta = now - s_last_cyc;   /* correct across the 32 bit wrap */
    s_last_cyc = now;

    uint32_t rem = s_cyc_rem + (delta % s_cycles_per_us);
    s_us_accum += (delta / s_cycles_per_us) + (rem / s_cycles_per_us);
    s_cyc_rem   = rem % s_cycles_per_us;

    uint32_t value = s_us_accum;

    __set_PRIMASK(primask);
    return value;
}
