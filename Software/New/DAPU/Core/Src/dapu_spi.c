/**
  ******************************************************************************
  * @file    dapu_spi.c
  * @brief   Mutual exclusion for the shared SPI1 sensor bus.
  ******************************************************************************
  */

#include "main.h"
#include "cmsis_os.h"
#include "spi.h"
#include "dapu_spi.h"
#include <stdio.h>

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

/* ==========================================================================
 * Bench diagnostics
 * ========================================================================== */

static const struct
{
    const char   *name;
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       id_reg;       /* register holding a known constant */
    uint8_t       expect;
} s_devices[] = {
    { "BMP280",   BMP280_CS_GPIO_Port,   BMP280_CS_Pin,   0xD0u, 0x58u },
    { "ICM20948", ICM20948_CS_GPIO_Port, ICM20948_CS_Pin, 0x00u, 0xEAu },
};

static uint8_t pin_index(uint16_t pin)
{
    return (uint8_t)__builtin_ctz((uint32_t)pin);
}

/* Electrical state of MISO, measured by driving the STM32's own weak pulls
 * against whatever is out on the wire. A line nobody is driving simply
 * follows the pull; a line a slave is holding does not. */
typedef enum
{
    MISO_FLOATING = 0,      /* follows the pull - nothing is driving it */
    MISO_HELD_LOW,
    MISO_HELD_HIGH,
    MISO_UNSTABLE
} miso_state_t;

static const char *miso_name(miso_state_t s)
{
    switch (s)
    {
    case MISO_FLOATING:  return "FLOATING  - nothing is driving PA6";
    case MISO_HELD_LOW:  return "held LOW  - a device or a short is pulling it down";
    case MISO_HELD_HIGH: return "held HIGH - driven high or pulled up";
    default:             return "unstable";
    }
}

static miso_state_t miso_probe(void)
{
    GPIO_InitTypeDef g = {0};
    int with_pullup, with_pulldown;

    g.Pin   = GPIO_PIN_6;               /* PA6 = SPI1_MISO */
    g.Mode  = GPIO_MODE_INPUT;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &g);
    osDelay(2);
    with_pullup = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET);

    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &g);
    osDelay(2);
    with_pulldown = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET);

    /* Hand the pin back to SPI1. */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &g);

    if (with_pullup && !with_pulldown) { return MISO_FLOATING;  }
    if (!with_pullup && !with_pulldown){ return MISO_HELD_LOW;  }
    if (with_pullup && with_pulldown)  { return MISO_HELD_HIGH; }
    return MISO_UNSTABLE;
}

/* Electrical test of the shared MISO net, with every device deselected.
 *
 * PA6 is briefly taken away from SPI1 and read as a plain input, first with
 * the internal pull-up and then with the pull-down. A line that nothing is
 * driving follows both. A line that reads low even with the pull-up enabled is
 * being clamped by something external - typically an unpowered chip whose ESD
 * diode is dragging the net down, or a strap resistor left fitted on a
 * breakout board. */
static void miso_line_test(void)
{
    GPIO_InitTypeDef gpio = {0};
    GPIO_PinState with_pullup, with_pulldown;
    const char *verdict;

    /* Make sure no slave is selected before measuring. */
    HAL_GPIO_WritePin(BMP280_CS_GPIO_Port,   BMP280_CS_Pin,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(ICM20948_CS_GPIO_Port, ICM20948_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MS5611_CS_GPIO_Port,   MS5611_CS_Pin,   GPIO_PIN_SET);

    gpio.Pin   = GPIO_PIN_6;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);
    osDelay(2);
    with_pullup = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);

    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &gpio);
    osDelay(2);
    with_pulldown = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);

    /* Hand PA6 back to SPI1. */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio);

    if (with_pullup == GPIO_PIN_SET && with_pulldown == GPIO_PIN_RESET)
    {
        verdict = "floating as it should";
    }
    else if (with_pullup == GPIO_PIN_RESET)
    {
        verdict = "HELD LOW externally - unpowered module or a strap resistor";
    }
    else
    {
        verdict = "HELD HIGH externally";
    }

    printf("  MISO PA6 with nothing selected: pull-up=%d pull-down=%d -> %s\r\n\r\n",
           (with_pullup == GPIO_PIN_SET) ? 1 : 0,
           (with_pulldown == GPIO_PIN_SET) ? 1 : 0,
           verdict);
}

void dapu_spi_diagnose(void)
{
    printf("\r\n--- SPI1 bus probe ------------------------------------------\r\n");
    printf("  CS low  = device selected, should answer\r\n");
    printf("  CS high = nothing selected, bus should float/idle\r\n\r\n");

    dapu_spi_lock();

    miso_line_test();

    for (uint32_t i = 0; i < (sizeof(s_devices) / sizeof(s_devices[0])); i++)
    {
        uint8_t tx[2]  = { (uint8_t)(s_devices[i].id_reg | 0x80u), 0xFF };
        uint8_t sel[2] = { 0, 0 };
        uint8_t idle[2] = { 0, 0 };
        const char *verdict;

        (void)dapu_spi_txrx(s_devices[i].port, s_devices[i].pin, tx, sel, 2);

        /* Identical bytes clocked out, but with no CS asserted at all. */
        (void)HAL_SPI_TransmitReceive(&hspi1, tx, idle, 2, 100);

        if (sel[1] == s_devices[i].expect)
        {
            verdict = "OK";
        }
        else if (sel[1] == idle[1])
        {
            verdict = "no response - CS or MISO not reaching the chip";
        }
        else
        {
            verdict = "answers but wrong - SPI mode or clock too fast";
        }

        printf("  %-9s PE%-2u   CS low -> 0x%02X   CS high -> 0x%02X   want 0x%02X   %s\r\n",
               s_devices[i].name, pin_index(s_devices[i].pin),
               sel[1], idle[1], s_devices[i].expect, verdict);
    }

    /* --- electrical state of MISO, which is what actually decides this ---
     * A correctly wired slave drives MISO for as long as its CS is low. If the
     * line still reads FLOATING with a device selected, that device's data
     * output is not reaching PA6 - or the chip has no power. */
    printf("\r\n  MISO (PA6) electrical state:\r\n");
    printf("    nothing selected     : %s\r\n", miso_name(miso_probe()));

    for (uint32_t i = 0; i < (sizeof(s_devices) / sizeof(s_devices[0])); i++)
    {
        miso_state_t state;

        HAL_GPIO_WritePin(s_devices[i].port, s_devices[i].pin, GPIO_PIN_RESET);
        state = miso_probe();
        HAL_GPIO_WritePin(s_devices[i].port, s_devices[i].pin, GPIO_PIN_SET);

        printf("    %-9s selected  : %s\r\n", s_devices[i].name, miso_name(state));
    }

    /* --- can the chip selects actually be driven? --- */
    printf("\r\n  Chip select drive test (pin read back after writing it):\r\n");

    for (uint32_t i = 0; i < (sizeof(s_devices) / sizeof(s_devices[0])); i++)
    {
        int reads_low, reads_high;

        HAL_GPIO_WritePin(s_devices[i].port, s_devices[i].pin, GPIO_PIN_RESET);
        osDelay(2);
        reads_low = (HAL_GPIO_ReadPin(s_devices[i].port, s_devices[i].pin) == GPIO_PIN_RESET);

        HAL_GPIO_WritePin(s_devices[i].port, s_devices[i].pin, GPIO_PIN_SET);
        osDelay(2);
        reads_high = (HAL_GPIO_ReadPin(s_devices[i].port, s_devices[i].pin) == GPIO_PIN_SET);

        printf("    %-9s PE%-2u      : %s\r\n",
               s_devices[i].name, pin_index(s_devices[i].pin),
               (reads_low && reads_high) ? "toggles correctly"
                                         : "STUCK - shorted or loaded down");
    }

    dapu_spi_unlock();
    printf("-------------------------------------------------------------\r\n");
}

void dapu_spi_cs_wiggle(uint32_t ms_per_pin)
{
    static const struct { const char *name; GPIO_TypeDef *port; uint16_t pin; } cs[] = {
        { "BMP280   PE7", BMP280_CS_GPIO_Port,   BMP280_CS_Pin   },
        { "ICM20948 PE8", ICM20948_CS_GPIO_Port, ICM20948_CS_Pin },
        { "MS5611   PE9", MS5611_CS_GPIO_Port,   MS5611_CS_Pin   },
    };

    /* Hold the bus so no sensor task drives a CS while we are testing. */
    dapu_spi_lock();

    printf("\r\nToggling each chip select at 5 Hz - probe the module pin:\r\n");

    for (uint32_t i = 0; i < (sizeof(cs) / sizeof(cs[0])); i++)
    {
        printf("  %s ...\r\n", cs[i].name);

        for (uint32_t t = 0; t < ms_per_pin; t += 100u)
        {
            HAL_GPIO_TogglePin(cs[i].port, cs[i].pin);
            osDelay(100);
        }
        HAL_GPIO_WritePin(cs[i].port, cs[i].pin, GPIO_PIN_SET);   /* leave idle */
    }

    printf("  done - all chip selects back high\r\n");
    dapu_spi_unlock();
}

bool dapu_spi_txrx(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                   const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    uint8_t scratch[DAPU_SPI_MAX_XFER];
    HAL_StatusTypeDef status;

    if (len == 0u || len > DAPU_SPI_MAX_XFER)
    {
        return false;
    }

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx,
                                     (rx != NULL) ? rx : scratch, len, 100);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    return (status == HAL_OK);
}
