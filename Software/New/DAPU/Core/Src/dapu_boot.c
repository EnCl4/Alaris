/**
  ******************************************************************************
  * @file    dapu_boot.c
  * @brief   Sensor detection and stationary boot calibration.
  ******************************************************************************
  */

#include "main.h"
#include "cmsis_os.h"
#include "dapu_config.h"
#include "dapu_state.h"
#include "dapu_analog.h"
#include "dapu_boot.h"
#include "bmp280.h"
#include "ms5611.h"
#include "icm20948.h"
#include "gps_ubx.h"
#include <stdio.h>

static volatile bool s_boot_done;

void dapu_boot_init_sensors(void)
{
    uint16_t status = 0u;

    printf("\r\n=== ALARIS DAPU - acquisition + logging build ===\r\n");

    /* Every ABSENT is reported with the raw byte that caused it: 0x00 or 0xFF
     * means MISO is stuck (wiring, CS, or the module not in SPI mode), while
     * any other value means the bus is alive but mis-clocked. */
    if (bmp280_init())
    {
        status |= DAPU_ST_BMP280_OK;
        printf("BMP280   : OK (chip id 0x%02X)\r\n", bmp280_chip_id());
    }
    else
    {
        printf("BMP280   : ABSENT (chip id 0x%02X, expected 0x58)\r\n",
               bmp280_chip_id());
    }

    if (ms5611_init())
    {
        status |= DAPU_ST_MS5611_OK;
        printf("MS5611   : OK\r\n");
    }
    else
    {
        printf("MS5611   : ABSENT (PROM %04X %04X %04X)\r\n",
               ms5611_prom_word(0), ms5611_prom_word(1), ms5611_prom_word(2));
    }

    if (icm20948_init())
    {
        status |= DAPU_ST_ICM20948_OK;
#if ICM20948_USE_I2C
        printf("ICM20948 : OK (I2C1, address 0x%02X)\r\n", icm20948_i2c_address());
#else
        printf("ICM20948 : OK (SPI1)\r\n");
#endif

        if (ak09916_init())
        {
            status |= DAPU_ST_AK09916_OK;
            printf("AK09916  : OK\r\n");
        }
        else
        {
            printf("AK09916  : ABSENT\r\n");
        }
    }
    else
    {
        printf("ICM20948 : ABSENT (WHO_AM_I 0x%02X, expected 0xEA)\r\n",
               icm20948_whoami_raw());
    }

    if (gps_init())
    {
        printf("NEO-M8N  : OK (%lu Hz, UBX NAV-PVT @ %lu baud)\r\n",
               (unsigned long)GPS_RATE_HZ, (unsigned long)GPS_BAUD_TARGET);
    }
    else
    {
        printf("NEO-M8N  : no UBX frame received\r\n");
    }

    dapu_state_set_status(status);

    /* Sensor status LED: solid only when every expected sensor answered. */
    const uint16_t expected = DAPU_ST_BMP280_OK | DAPU_ST_MS5611_OK |
                              DAPU_ST_ICM20948_OK | DAPU_ST_AK09916_OK;
    HAL_GPIO_WritePin(LED_SENSORS_GPIO_Port, LED_SENSORS_Pin,
                      ((status & expected) == expected) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void dapu_boot_calibrate(void)
{
    double gyro_sum[3] = { 0.0, 0.0, 0.0 };
    double bmp_sum = 0.0, ms_sum = 0.0, pitot_sum = 0.0;
    uint32_t gyro_n = 0u, bmp_n = 0u, ms_n = 0u, pitot_n = 0u;

    const uint32_t t0 = HAL_GetTick();
    uint32_t next_bmp = t0;
    uint32_t next_ms  = t0;

    printf("Calibrating (%lu ms) - keep the aircraft still and level...\r\n",
           (unsigned long)BARO_REF_AVERAGE_MS);

    while ((HAL_GetTick() - t0) < BARO_REF_AVERAGE_MS)
    {
        uint32_t now = HAL_GetTick();

        if (icm20948_present())
        {
            axises accel, gyro;
            if (icm20948_read_imu(&accel, &gyro))
            {
                gyro_sum[0] += (double)gyro.x;
                gyro_sum[1] += (double)gyro.y;
                gyro_sum[2] += (double)gyro.z;
                gyro_n++;
            }
        }

        if (bmp280_present() && (int32_t)(now - next_bmp) >= 0)
        {
            float t, p;
            next_bmp += TASK_PERIOD_BMP_MS;
            if (bmp280_read(&t, &p))
            {
                bmp_sum += (double)p;
                bmp_n++;
            }
        }

        if (ms5611_present() && (int32_t)(now - next_ms) >= 0)
        {
            next_ms += TASK_PERIOD_MS5611_MS;
            if (ms5611_step())
            {
                ms_sum += (double)ms5611_pressure_pa();
                ms_n++;
            }
        }

        pitot_sum += (double)analog_avg_counts(ADC_IDX_PITOT, ADC_DECIMATION_SCANS);
        pitot_n++;

        HAL_Delay(TASK_PERIOD_IMU_MS);
    }

    dapu_state_lock();

    if (gyro_n > 0u)
    {
        g_state.gyro_bias_dps[0] = (float)(gyro_sum[0] / (double)gyro_n);
        g_state.gyro_bias_dps[1] = (float)(gyro_sum[1] / (double)gyro_n);
        g_state.gyro_bias_dps[2] = (float)(gyro_sum[2] / (double)gyro_n);
    }

    /* Falling back to the ISA sea level pressure keeps the altitude finite
     * when a barometer is missing; the value is meaningless but bounded. */
    g_state.bmp_press_ref_pa = (bmp_n > 0u) ? (float)(bmp_sum / (double)bmp_n)
                                            : BARO_SEA_LEVEL_PA;
    g_state.ms_press_ref_pa  = (ms_n  > 0u) ? (float)(ms_sum  / (double)ms_n)
                                            : BARO_SEA_LEVEL_PA;

    g_state.pitot_zero_v = (pitot_n > 0u)
        ? analog_counts_to_volts((float)(pitot_sum / (double)pitot_n))
        : 0.0f;

    g_state.status |= DAPU_ST_CALIBRATED;

    float gbx = g_state.gyro_bias_dps[0];
    float gby = g_state.gyro_bias_dps[1];
    float gbz = g_state.gyro_bias_dps[2];
    float bref = g_state.bmp_press_ref_pa;
    float mref = g_state.ms_press_ref_pa;
    float pz   = g_state.pitot_zero_v;

    dapu_state_unlock();

    printf("gyro bias : %.3f %.3f %.3f dps  (%lu samples)\r\n",
           gbx, gby, gbz, (unsigned long)gyro_n);
    printf("baro ref  : BMP280 %.1f Pa (%lu)  MS5611 %.1f Pa (%lu)\r\n",
           bref, (unsigned long)bmp_n, mref, (unsigned long)ms_n);
    printf("pitot zero: %.4f V\r\n", pz);
}

void dapu_boot_run(void)
{
    /* The ADC scan is started here rather than in MX_FREERTOS_Init() because
     * HAL_ADCEx_Calibration_Start() polls HAL_GetTick(), which does not
     * advance until the scheduler has cleared the port's critical nesting. */
    analog_start();
    osDelay(50);                        /* let the first scans land */

    /* Guard against the TIM2 trigger going missing: without it the DMA never
     * advances and every analog channel reads a constant, which is easy to
     * mistake for a wiring problem hours later. */
    if (!analog_is_running(20u))
    {
        printf("ADC scan : NOT RUNNING - TIM2 TRGO or DMA1_Stream0 is dead\r\n");
    }

    /* Deselect every device on the shared SPI1 bus before the first probe. */
    HAL_GPIO_WritePin(BMP280_CS_GPIO_Port,   BMP280_CS_Pin,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(ICM20948_CS_GPIO_Port, ICM20948_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MS5611_CS_GPIO_Port,   MS5611_CS_Pin,   GPIO_PIN_SET);

    dapu_boot_init_sensors();
    dapu_boot_calibrate();

    HAL_GPIO_WritePin(L_workingStatus_GPIO_Port, L_workingStatus_Pin, GPIO_PIN_SET);

    s_boot_done = true;

    printf("Boot complete.\r\n"
           "Console: 'p' plot, 's' status, 'q' close log, 'r' new log,\r\n"
           "         'd' SPI bus probe, 'w' chip-select wiggle\r\n");
}

bool dapu_boot_complete(void)
{
    return s_boot_done;
}

void dapu_boot_wait(void)
{
    while (!s_boot_done)
    {
        osDelay(10);
    }
}
