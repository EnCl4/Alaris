/**
  ******************************************************************************
  * @file    bmp280.c
  * @brief   Bosch BMP280 pressure/temperature sensor, SPI1 + CS on PE7.
  ******************************************************************************
  * Compensation code follows the fixed point reference implementation in the
  * BMP280 datasheet (rev 1.19, sec. 3.11.3 / 8.2).
  ******************************************************************************
  */

#include "main.h"
#include "spi.h"
#include "dapu_spi.h"
#include "bmp280.h"
#include <math.h>

#define BMP280_SPI              (&hspi1)
#define BMP280_CS_PORT          BMP280_CS_GPIO_Port
#define BMP280_CS_PIN           BMP280_CS_Pin

#define BMP280_REG_ID           0xD0
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP280_REG_CALIB00      0x88

#define BMP280_RESET_VALUE      0xB6

/* osrs_t x1, osrs_p x2, IIR coeff 4, standby 0.5 ms.
 * t_measure ~= 8.1 ms -> ~123 Hz, comfortably above the 100 Hz task rate.
 * Note: the x16 oversampling used on the F4 bench code caps the BMP280 at
 * roughly 23 Hz and cannot meet the 100 Hz of Tab. Freqsensores. */
#define BMP280_OSRS_T           1u      /* x1  */
#define BMP280_OSRS_P           2u      /* x2  */
#define BMP280_MODE_NORMAL      3u
#define BMP280_FILTER_COEFF     2u      /* 0b010 -> coefficient 4 */
#define BMP280_T_SB             0u      /* 0.5 ms */

static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

static int32_t  t_fine;
static bool     s_present;

/* ---------------------------------------------------------------- bus ---- */

static inline void cs_low(void)  { HAL_GPIO_WritePin(BMP280_CS_PORT, BMP280_CS_PIN, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(BMP280_CS_PORT, BMP280_CS_PIN, GPIO_PIN_SET);  }

static bool bmp280_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t addr = reg | 0x80u;             /* MSB set = read */
    bool ok;

    cs_low();
    ok  = (HAL_SPI_Transmit(BMP280_SPI, &addr, 1, 10) == HAL_OK);
    ok &= (HAL_SPI_Receive(BMP280_SPI, buf, len, 50) == HAL_OK);
    cs_high();

    return ok;
}

static bool bmp280_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { (uint8_t)(reg & 0x7Fu), data };   /* MSB clear = write */
    bool ok;

    cs_low();
    ok = (HAL_SPI_Transmit(BMP280_SPI, buf, 2, 10) == HAL_OK);
    cs_high();

    return ok;
}

/* --------------------------------------------------------------- init ---- */

static bool bmp280_read_calibration(void)
{
    uint8_t c[24];

    if (!bmp280_read_regs(BMP280_REG_CALIB00, c, sizeof(c)))
    {
        return false;
    }

    dig_T1 = (uint16_t)((c[1]  << 8) | c[0]);
    dig_T2 = (int16_t) ((c[3]  << 8) | c[2]);
    dig_T3 = (int16_t) ((c[5]  << 8) | c[4]);
    dig_P1 = (uint16_t)((c[7]  << 8) | c[6]);
    dig_P2 = (int16_t) ((c[9]  << 8) | c[8]);
    dig_P3 = (int16_t) ((c[11] << 8) | c[10]);
    dig_P4 = (int16_t) ((c[13] << 8) | c[12]);
    dig_P5 = (int16_t) ((c[15] << 8) | c[14]);
    dig_P6 = (int16_t) ((c[17] << 8) | c[16]);
    dig_P7 = (int16_t) ((c[19] << 8) | c[18]);
    dig_P8 = (int16_t) ((c[21] << 8) | c[20]);
    dig_P9 = (int16_t) ((c[23] << 8) | c[22]);

    /* dig_T1 and dig_P1 are never 0 on a working part; all-zero or all-ones
     * means we are talking to nothing. */
    return (dig_T1 != 0u) && (dig_T1 != 0xFFFFu) &&
           (dig_P1 != 0u) && (dig_P1 != 0xFFFFu);
}

bool bmp280_init(void)
{
    uint8_t id = 0;

    s_present = false;

    dapu_spi_lock();

    if (!bmp280_read_regs(BMP280_REG_ID, &id, 1))
    {
        dapu_spi_unlock();
        return false;
    }
    /* 0x58 = BMP280; 0x56/0x57 are pre-production samples. */
    if (id != 0x58u && id != 0x57u && id != 0x56u)
    {
        dapu_spi_unlock();
        return false;
    }

    (void)bmp280_write_reg(BMP280_REG_RESET, BMP280_RESET_VALUE);
    dapu_spi_unlock();

    HAL_Delay(10);

    dapu_spi_lock();

    if (!bmp280_read_calibration())
    {
        dapu_spi_unlock();
        return false;
    }

    (void)bmp280_write_reg(BMP280_REG_CONFIG,
                           (uint8_t)((BMP280_T_SB << 5) | (BMP280_FILTER_COEFF << 2)));
    (void)bmp280_write_reg(BMP280_REG_CTRL_MEAS,
                           (uint8_t)((BMP280_OSRS_T << 5) | (BMP280_OSRS_P << 2) |
                                     BMP280_MODE_NORMAL));

    dapu_spi_unlock();

    HAL_Delay(20);
    s_present = true;
    return true;
}

bool bmp280_present(void)
{
    return s_present;
}

/* ---------------------------------------------------------- compensate --- */

static float compensate_temperature(int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;

    return (float)((t_fine * 5 + 128) >> 8) / 100.0f;
}

static float compensate_pressure(int32_t adc_P)
{
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)dig_P6;

    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = ((((int64_t)1) << 47) + var1) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0.0f;                        /* avoid division by zero */
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

    return (float)p / 256.0f;               /* Pa */
}

/* ---------------------------------------------------------------- read --- */

bool bmp280_read(float *temp_c, float *press_pa)
{
    uint8_t buf[6];
    bool ok;

    if (!s_present)
    {
        return false;
    }

    /* 0xF7..0xFC in one burst: press[3] then temp[3], guaranteed coherent. */
    dapu_spi_lock();
    ok = bmp280_read_regs(BMP280_REG_PRESS_MSB, buf, sizeof(buf));
    dapu_spi_unlock();

    if (!ok)
    {
        return false;
    }

    int32_t adc_P = (int32_t)(((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | (buf[2] >> 4));
    int32_t adc_T = (int32_t)(((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | (buf[5] >> 4));

    /* 0x80000 is the reset value: the first conversion is not ready yet. */
    if (adc_P == 0x80000 || adc_T == 0x80000)
    {
        return false;
    }

    *temp_c   = compensate_temperature(adc_T);   /* must run first: sets t_fine */
    *press_pa = compensate_pressure(adc_P);

    return (*press_pa > 1000.0f);
}

float bmp280_altitude(float press_pa, float reference_pa)
{
    if (press_pa <= 0.0f || reference_pa <= 0.0f)
    {
        return 0.0f;
    }
    return 44330.0f * (1.0f - powf(press_pa / reference_pa, 0.1903f));
}
