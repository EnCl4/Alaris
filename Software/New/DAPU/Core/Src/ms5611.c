/**
  ******************************************************************************
  * @file    ms5611.c
  * @brief   TE MS5611-01BA03 barometer, SPI1 + CS on PE9.
  ******************************************************************************
  * Compensation and CRC4 follow the MS5611-01BA03 datasheet (rev. ENG_DS, 2017).
  ******************************************************************************
  */

#include "main.h"
#include "spi.h"
#include "dapu_spi.h"
#include "ms5611.h"
#include <math.h>

#define MS5611_SPI          (&hspi1)
#define MS5611_CS_PORT      MS5611_CS_GPIO_Port
#define MS5611_CS_PIN       MS5611_CS_Pin

#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_CONV_D1_4096 0x48    /* pressure,    OSR 4096, 9.04 ms */
#define MS5611_CMD_CONV_D2_4096 0x58    /* temperature, OSR 4096, 9.04 ms */
#define MS5611_CMD_ADC_READ     0x00
#define MS5611_CMD_PROM_READ    0xA0    /* + (address << 1) */

typedef enum
{
    MS_STATE_IDLE = 0,
    MS_STATE_WAIT_D1,       /* pressure conversion running    */
    MS_STATE_WAIT_D2        /* temperature conversion running */
} ms5611_state_t;

static uint16_t s_prom[8];
static bool     s_present;
static ms5611_state_t s_state;

static uint32_t s_d1;                   /* raw pressure    */
static uint32_t s_d2;                   /* raw temperature */
static float    s_temp_c   = 0.0f;
static float    s_press_pa = 0.0f;

/* ---------------------------------------------------------------- bus ---- */

static inline void cs_low(void)  { HAL_GPIO_WritePin(MS5611_CS_PORT, MS5611_CS_PIN, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(MS5611_CS_PORT, MS5611_CS_PIN, GPIO_PIN_SET);  }

static bool ms5611_command(uint8_t cmd)
{
    bool ok;

    cs_low();
    ok = (HAL_SPI_Transmit(MS5611_SPI, &cmd, 1, 10) == HAL_OK);
    cs_high();

    return ok;
}

static bool ms5611_read_prom_word(uint8_t address, uint16_t *out)
{
    uint8_t cmd = (uint8_t)(MS5611_CMD_PROM_READ | (address << 1));
    uint8_t rx[2] = { 0, 0 };
    bool ok;

    cs_low();
    ok  = (HAL_SPI_Transmit(MS5611_SPI, &cmd, 1, 10) == HAL_OK);
    ok &= (HAL_SPI_Receive(MS5611_SPI, rx, 2, 10) == HAL_OK);
    cs_high();

    *out = (uint16_t)((rx[0] << 8) | rx[1]);
    return ok;
}

static bool ms5611_read_adc(uint32_t *out)
{
    uint8_t cmd = MS5611_CMD_ADC_READ;
    uint8_t rx[3] = { 0, 0, 0 };
    bool ok;

    cs_low();
    ok  = (HAL_SPI_Transmit(MS5611_SPI, &cmd, 1, 10) == HAL_OK);
    ok &= (HAL_SPI_Receive(MS5611_SPI, rx, 3, 10) == HAL_OK);
    cs_high();

    *out = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | (uint32_t)rx[2];

    /* The ADC returns 0 if it is read before the conversion has finished. */
    return ok && (*out != 0u);
}

/* PROM CRC4, datasheet AN520 reference implementation. */
static uint8_t ms5611_crc4(uint16_t *prom)
{
    uint16_t n_rem = 0u;
    uint16_t crc_read = prom[7];

    prom[7] = (uint16_t)(0xFF00u & prom[7]);

    for (int cnt = 0; cnt < 16; cnt++)
    {
        if ((cnt % 2) == 1)
        {
            n_rem ^= (uint16_t)(prom[cnt >> 1] & 0x00FFu);
        }
        else
        {
            n_rem ^= (uint16_t)(prom[cnt >> 1] >> 8);
        }

        for (int n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & 0x8000u)
            {
                n_rem = (uint16_t)((n_rem << 1) ^ 0x3000u);
            }
            else
            {
                n_rem = (uint16_t)(n_rem << 1);
            }
        }
    }

    n_rem = (uint16_t)(0x000Fu & (n_rem >> 12));
    prom[7] = crc_read;

    return (uint8_t)n_rem;
}

/* --------------------------------------------------------------- init ---- */

bool ms5611_init(void)
{
    s_present = false;
    s_state   = MS_STATE_IDLE;

    dapu_spi_lock();
    bool ok = ms5611_command(MS5611_CMD_RESET);
    dapu_spi_unlock();

    if (!ok)
    {
        return false;
    }
    HAL_Delay(5);                       /* reload sequence, datasheet: 2.8 ms */

    dapu_spi_lock();
    for (uint8_t i = 0; i < 8u; i++)
    {
        if (!ms5611_read_prom_word(i, &s_prom[i]))
        {
            dapu_spi_unlock();
            return false;
        }
    }
    dapu_spi_unlock();

    /* An absent device clocks out all-zeros or all-ones. */
    if ((s_prom[1] == 0u && s_prom[2] == 0u) ||
        (s_prom[1] == 0xFFFFu && s_prom[2] == 0xFFFFu))
    {
        return false;
    }

    uint8_t crc_expected = (uint8_t)(s_prom[7] & 0x000Fu);
    if (ms5611_crc4(s_prom) != crc_expected)
    {
        return false;
    }

    s_present = true;

    /* Kick off the first pressure conversion. */
    dapu_spi_lock();
    (void)ms5611_command(MS5611_CMD_CONV_D1_4096);
    dapu_spi_unlock();
    s_state = MS_STATE_WAIT_D1;

    return true;
}

bool ms5611_present(void)
{
    return s_present;
}

/* -------------------------------------------------------- compensation --- */

static void ms5611_compensate(void)
{
    const int64_t C1 = (int64_t)s_prom[1];
    const int64_t C2 = (int64_t)s_prom[2];
    const int64_t C3 = (int64_t)s_prom[3];
    const int64_t C4 = (int64_t)s_prom[4];
    const int64_t C5 = (int64_t)s_prom[5];
    const int64_t C6 = (int64_t)s_prom[6];

    int64_t dT   = (int64_t)s_d2 - (C5 << 8);
    int64_t TEMP = 2000 + ((dT * C6) >> 23);
    int64_t OFF  = (C2 << 16) + ((C4 * dT) >> 7);
    int64_t SENS = (C1 << 15) + ((C3 * dT) >> 8);

    /* Second order temperature compensation. */
    if (TEMP < 2000)
    {
        int64_t T2    = (dT * dT) >> 31;
        int64_t d     = TEMP - 2000;
        int64_t OFF2  = (5 * d * d) >> 1;
        int64_t SENS2 = (5 * d * d) >> 2;

        if (TEMP < -1500)
        {
            int64_t e = TEMP + 1500;
            OFF2  += 7 * e * e;
            SENS2 += (11 * e * e) >> 1;
        }

        TEMP -= T2;
        OFF  -= OFF2;
        SENS -= SENS2;
    }

    int64_t P = (((int64_t)s_d1 * SENS) >> 21) - OFF;
    P >>= 15;

    s_temp_c   = (float)TEMP / 100.0f;
    s_press_pa = (float)P;              /* datasheet result is already in Pa */
}

/* --------------------------------------------------------------- step ---- */

bool ms5611_step(void)
{
    bool produced = false;

    if (!s_present)
    {
        return false;
    }

    switch (s_state)
    {
    case MS_STATE_WAIT_D1:
        dapu_spi_lock();
        if (ms5611_read_adc(&s_d1))
        {
            (void)ms5611_command(MS5611_CMD_CONV_D2_4096);
            s_state = MS_STATE_WAIT_D2;
        }
        else
        {
            /* Conversion not finished or bus glitch: restart cleanly. */
            (void)ms5611_command(MS5611_CMD_CONV_D1_4096);
        }
        dapu_spi_unlock();
        break;

    case MS_STATE_WAIT_D2:
        dapu_spi_lock();
        if (ms5611_read_adc(&s_d2))
        {
            ms5611_compensate();
            produced = true;
        }
        (void)ms5611_command(MS5611_CMD_CONV_D1_4096);
        dapu_spi_unlock();
        s_state = MS_STATE_WAIT_D1;
        break;

    case MS_STATE_IDLE:
    default:
        dapu_spi_lock();
        (void)ms5611_command(MS5611_CMD_CONV_D1_4096);
        dapu_spi_unlock();
        s_state = MS_STATE_WAIT_D1;
        break;
    }

    return produced;
}

float ms5611_temperature_c(void) { return s_temp_c;   }
float ms5611_pressure_pa(void)   { return s_press_pa; }

float ms5611_altitude(float press_pa, float reference_pa)
{
    if (press_pa <= 0.0f || reference_pa <= 0.0f)
    {
        return 0.0f;
    }
    return 44330.0f * (1.0f - powf(press_pa / reference_pa, 0.1903f));
}
