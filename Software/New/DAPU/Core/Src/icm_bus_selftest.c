/**
  ******************************************************************************
  * @file    icm_bus_selftest.c
  * @brief   Bare-metal ICM20948 bus bring-up test.
  ******************************************************************************
  */

#include "main.h"
#include "dapu_config.h"
#include "icm_bus_selftest.h"
#include <stdio.h>
#include <stdbool.h>

#if ICM_BUS_SELFTEST

#define ICM_WHO_AM_I_REG    0x00u
#define ICM_WHO_AM_I_VALUE  0xEAu

/* ==========================================================================
 * I2C variant
 * ========================================================================== */
#if ICM20948_USE_I2C

#include "i2c.h"

static uint8_t s_addr8;     /* 8-bit (shifted) address that answered */

static void i2c_bus_scan(void)
{
    uint32_t found = 0;

    printf("Scanning I2C1 (PB8 SCL / PB9 SDA)...\r\n");

    for (uint8_t addr7 = 0x08u; addr7 <= 0x77u; addr7++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr7 << 1), 2, 5) == HAL_OK)
        {
            const char *who = "";

            if (addr7 == 0x68u || addr7 == 0x69u) { who = "  <- ICM20948"; }
            if (addr7 == 0x0Cu)                   { who = "  <- AK09916 magnetometer"; }
            if (addr7 == 0x76u || addr7 == 0x77u) { who = "  <- BMP280"; }

            printf("  found device at 0x%02X%s\r\n", addr7, who);
            found++;

            if ((addr7 == 0x68u || addr7 == 0x69u) && (s_addr8 == 0u))
            {
                s_addr8 = (uint8_t)(addr7 << 1);
            }
        }
    }

    if (found == 0u)
    {
        printf("  NOTHING RESPONDED - the bus itself is dead:\r\n"
               "    1. SCL/SDA swapped, or not on PB8/PB9.\r\n"
               "    2. No pull-up resistors. I2C cannot work without them and\r\n"
               "       the internal ~40k ones are often too weak on a\r\n"
               "       breadboard - fit 4.7k from SCL and SDA to 3.3 V.\r\n"
               "    3. Module unpowered - measure 3.3 V at its own VCC pin.\r\n"
               "    4. No common ground between the board and the module.\r\n");
    }
}

static uint8_t icm_read_who_am_i(void)
{
    uint8_t value = 0xFFu;
    uint8_t bank  = 0x00u;

    if (s_addr8 == 0u)
    {
        return 0xFFu;
    }

    /* WHO_AM_I is in user bank 0 - the reset default, but select it
     * explicitly so a previous run cannot confuse the result. */
    (void)HAL_I2C_Mem_Write(&hi2c1, s_addr8, 0x7Fu, I2C_MEMADD_SIZE_8BIT,
                            &bank, 1, 50);

    if (HAL_I2C_Mem_Read(&hi2c1, s_addr8, ICM_WHO_AM_I_REG,
                         I2C_MEMADD_SIZE_8BIT, &value, 1, 50) != HAL_OK)
    {
        return 0xFFu;
    }
    return value;
}

static void bus_report(void)
{
    uint8_t value;

    printf("\r\n=== ICM20948 bare-metal I2C test ===\r\n");
    printf("SCL PB8, SDA PB9. Expecting WHO_AM_I = 0x%02X at 0x68 or 0x69.\r\n\r\n",
           ICM_WHO_AM_I_VALUE);

    s_addr8 = 0u;
    i2c_bus_scan();

    if (s_addr8 == 0u)
    {
        printf("\r\nNo ICM20948 at 0x68 or 0x69. If some other address appeared\r\n"
               "above, the bus is fine but that is not the chip this driver\r\n"
               "expects.\r\n");
        return;
    }

    value = icm_read_who_am_i();

    printf("\r\nWHO_AM_I at 0x%02X = 0x%02X  %s\r\n",
           (unsigned)(s_addr8 >> 1), value,
           (value == ICM_WHO_AM_I_VALUE) ? "<-- OK, the sensor is working"
                                         : "<-- answered, but wrong ID");
}

#else   /* ==================== SPI variant ================================= */

#include "spi.h"

static const struct { uint32_t presc; const char *label; } k_speeds[] = {
    { SPI_BAUDRATEPRESCALER_256, "390 kHz" },
    { SPI_BAUDRATEPRESCALER_64,  "1.6 MHz" },
    { SPI_BAUDRATEPRESCALER_16,  "6.3 MHz" },
};

static const struct { uint32_t cpol; uint32_t cpha; const char *label; } k_modes[] = {
    { SPI_POLARITY_LOW,  SPI_PHASE_1EDGE, "mode 0" },
    { SPI_POLARITY_LOW,  SPI_PHASE_2EDGE, "mode 1" },
    { SPI_POLARITY_HIGH, SPI_PHASE_1EDGE, "mode 2" },
    { SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, "mode 3" },
};

static uint8_t icm_read_who_am_i(void)
{
    uint8_t tx[2] = { (uint8_t)(ICM_WHO_AM_I_REG | 0x80u), 0x00u };
    uint8_t rx[2] = { 0u, 0u };

    HAL_GPIO_WritePin(ICM20948_CS_GPIO_Port, ICM20948_CS_Pin, GPIO_PIN_RESET);
    (void)HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
    HAL_GPIO_WritePin(ICM20948_CS_GPIO_Port, ICM20948_CS_Pin, GPIO_PIN_SET);

    return rx[1];
}

static void spi_reconfigure(uint32_t cpol, uint32_t cpha, uint32_t presc)
{
    HAL_SPI_DeInit(&hspi1);
    hspi1.Init.CLKPolarity       = cpol;
    hspi1.Init.CLKPhase          = cpha;
    hspi1.Init.BaudRatePrescaler = presc;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_Delay(2);
}

static void bus_report(void)
{
    const uint32_t saved_cpol  = hspi1.Init.CLKPolarity;
    const uint32_t saved_cpha  = hspi1.Init.CLKPhase;
    const uint32_t saved_presc = hspi1.Init.BaudRatePrescaler;
    bool found = false;

    printf("\r\n=== ICM20948 bare-metal SPI test ===\r\n");
    printf("SCK PA5, MISO PA6, MOSI PA7, NCS PE8. Expecting 0x%02X.\r\n"
           "Remember: on this chip AD0 *is* the MISO pin.\r\n\r\n",
           ICM_WHO_AM_I_VALUE);

    HAL_GPIO_WritePin(BMP280_CS_GPIO_Port,   BMP280_CS_Pin,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(MS5611_CS_GPIO_Port,   MS5611_CS_Pin,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(ICM20948_CS_GPIO_Port, ICM20948_CS_Pin, GPIO_PIN_SET);

    printf("           %-9s %-9s %-9s\r\n",
           k_speeds[0].label, k_speeds[1].label, k_speeds[2].label);

    for (uint32_t m = 0; m < (sizeof(k_modes) / sizeof(k_modes[0])); m++)
    {
        printf("  %-7s", k_modes[m].label);

        for (uint32_t s = 0; s < (sizeof(k_speeds) / sizeof(k_speeds[0])); s++)
        {
            uint8_t value;

            spi_reconfigure(k_modes[m].cpol, k_modes[m].cpha, k_speeds[s].presc);
            (void)icm_read_who_am_i();      /* discard the first */
            value = icm_read_who_am_i();

            printf("  0x%02X %-4s", value,
                   (value == ICM_WHO_AM_I_VALUE) ? "OK" : "");

            if (value == ICM_WHO_AM_I_VALUE)
            {
                found = true;
            }
        }
        printf("\r\n");
    }

    if (!found)
    {
        printf("\r\nNO REPLY in any mode at any speed - this is hardware:\r\n"
               "  1. AD0 is not wired to PA6 (on the ICM20948 AD0 is MISO).\r\n"
               "  2. NCS is not reaching PE8.\r\n"
               "  3. Module unpowered - measure 3.3 V at its own VCC pin.\r\n"
               "  4. FSYNC left floating - tie it to GND.\r\n");
    }

    spi_reconfigure(saved_cpol, saved_cpha, saved_presc);
}

#endif  /* ICM20948_USE_I2C */

/* ==========================================================================
 * Common: report once, then poll so wires can be moved and watched live
 * ========================================================================== */

void icm_bus_selftest_run(void)
{
    bus_report();

    printf("\r\nPolling WHO_AM_I for %u s (move wires and watch)...\r\n",
           (unsigned)ICM_BUS_SELFTEST_SECONDS);

    for (uint32_t i = 0; i < (ICM_BUS_SELFTEST_SECONDS * 2u); i++)
    {
        uint8_t value = icm_read_who_am_i();

        printf("  WHO_AM_I = 0x%02X %s\r\n", value,
               (value == ICM_WHO_AM_I_VALUE) ? "<-- OK" : "");
        HAL_Delay(500);
    }

#if ICM_BUS_SELFTEST > 1
    printf("\r\nICM_BUS_SELFTEST > 1: staying here. Set it to 0 in "
           "dapu_config.h to boot normally.\r\n");
    for (;;)
    {
        uint8_t value = icm_read_who_am_i();
        printf("  WHO_AM_I = 0x%02X %s\r\n", value,
               (value == ICM_WHO_AM_I_VALUE) ? "<-- OK" : "");
        HAL_Delay(500);
    }
#else
    printf("=== end of bus test, continuing boot ===\r\n\r\n");
#endif
}

#endif /* ICM_BUS_SELFTEST */
