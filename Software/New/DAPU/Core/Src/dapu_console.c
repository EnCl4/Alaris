/**
  ******************************************************************************
  * @file    dapu_console.c
  * @brief   USART1 console: printf retarget, live plot, single key commands.
  ******************************************************************************
  */

#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "dapu_config.h"
#include "dapu_state.h"
#include "dapu_log.h"
#include "dapu_console.h"
#include "dapu_spi.h"
#include "gps_ubx.h"
#include <stdio.h>
#include <string.h>

static osMutexId_t s_uart_mutex;
static const osMutexAttr_t s_uart_mutex_attr = { .name = "UartMutex" };

static volatile bool    s_plot_enabled = (PLOT_DEFAULT_ENABLED != 0);
static volatile uint8_t s_rx_byte;
static volatile char    s_pending_cmd;

/* ==========================================================================
 * printf retarget (overrides the weak _write in syscalls.c)
 * ========================================================================== */

int _write(int file, char *ptr, int len)
{
    (void)file;

    if (s_uart_mutex != NULL && osKernelGetState() == osKernelRunning)
    {
        osMutexAcquire(s_uart_mutex, osWaitForever);
        HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, 100);
        osMutexRelease(s_uart_mutex);
    }
    else
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, 100);
    }
    return len;
}

/* ==========================================================================
 * Command handling
 * ========================================================================== */

void console_print_status(void)
{
    dapu_state_lock();
    uint16_t status  = g_state.status;
    uint32_t imu_seq = g_state.imu_seq;
    uint32_t bmp_seq = g_state.bmp_seq;
    uint32_t ms_seq  = g_state.ms_seq;
    uint32_t gps_seq = g_state.gps_seq;
    uint32_t rpm_seq = g_state.rpm_seq;
    dapu_state_unlock();

    uint32_t uptime_s = HAL_GetTick() / 1000u;
    if (uptime_s == 0u)
    {
        uptime_s = 1u;
    }

    printf("\r\n--- DAPU status -------------------------------------------\r\n");
    printf("uptime      : %lu s\r\n", (unsigned long)uptime_s);
    printf("status bits : 0x%04X  (BMP %d MS %d ICM %d MAG %d FIX %d SD %d)\r\n",
           status,
           (status & DAPU_ST_BMP280_OK)   ? 1 : 0,
           (status & DAPU_ST_MS5611_OK)   ? 1 : 0,
           (status & DAPU_ST_ICM20948_OK) ? 1 : 0,
           (status & DAPU_ST_AK09916_OK)  ? 1 : 0,
           (status & DAPU_ST_GPS_FIX_3D)  ? 1 : 0,
           (status & DAPU_ST_SD_OK)       ? 1 : 0);
    printf("measured Hz : IMU %lu  BMP %lu  MS5611 %lu  GPS %lu  RPM %lu\r\n",
           (unsigned long)(imu_seq / uptime_s),
           (unsigned long)(bmp_seq / uptime_s),
           (unsigned long)(ms_seq  / uptime_s),
           (unsigned long)(gps_seq / uptime_s),
           (unsigned long)(rpm_seq / uptime_s));
    printf("gps frames  : %lu  (checksum errors %lu)\r\n",
           (unsigned long)gps_frame_count(),
           (unsigned long)gps_checksum_errors());
    printf("log file    : %s  records %lu  dropped %lu\r\n",
           log_is_open() ? log_filename() : "(closed)",
           (unsigned long)log_written_records(),
           (unsigned long)log_dropped_records());
    printf("plot        : %s\r\n", s_plot_enabled ? "ON" : "OFF");
    printf("-----------------------------------------------------------\r\n");
}

static void console_handle_command(char c)
{
    switch (c)
    {
    case 'p':
    case 'P':
        s_plot_enabled = !s_plot_enabled;
        printf("\r\n[plot %s]\r\n", s_plot_enabled ? "ON" : "OFF");
        break;

    case 's':
    case 'S':
        console_print_status();
        break;

    case 'q':
    case 'Q':
        log_close();
        printf("\r\n[log closed - card can be removed]\r\n");
        break;

    case 'd':
    case 'D':
        dapu_spi_diagnose();
        break;

    case 'w':
    case 'W':
        dapu_spi_cs_wiggle(3000u);
        break;

    case 'r':
    case 'R':
        if (log_open())
        {
            printf("\r\n[logging to %s]\r\n", log_filename());
        }
        else
        {
            printf("\r\n[log open FAILED]\r\n");
        }
        break;

    default:
        break;
    }
}

/* ==========================================================================
 * Live plot (Teleplot / Serial Studio compatible: ">name:value")
 * ========================================================================== */

void console_plot(void)
{
    /* Commands are executed here, in task context, so the ISR stays short and
     * f_open/f_close never run from an interrupt. */
    char cmd = s_pending_cmd;
    if (cmd != 0)
    {
        s_pending_cmd = 0;
        console_handle_command(cmd);
    }

    if (!s_plot_enabled)
    {
        return;
    }

    dapu_state_lock();
    float ax = g_state.accel_g[0], ay = g_state.accel_g[1], az = g_state.accel_g[2];
    float gx = g_state.gyro_dps[0], gy = g_state.gyro_dps[1], gz = g_state.gyro_dps[2];
    float bmp_alt = g_state.bmp_alt_m;
    float ms_alt  = g_state.ms_alt_m;
    float ias     = g_state.ias_mps;
    float aoa     = g_state.aoa_deg;
    float rpm     = g_state.rpm;
    float vbat    = g_state.vbat_v;
    dapu_state_unlock();

    printf(">ax:%.3f\n>ay:%.3f\n>az:%.3f\n", ax, ay, az);
    printf(">gx:%.2f\n>gy:%.2f\n>gz:%.2f\n", gx, gy, gz);
    printf(">alt_bmp:%.2f\n>alt_ms:%.2f\n", bmp_alt, ms_alt);
    printf(">ias:%.2f\n>aoa:%.2f\n>rpm:%.0f\n>vbat:%.2f\n", ias, aoa, rpm, vbat);
}

bool console_plot_enabled(void)
{
    return s_plot_enabled;
}

/* ==========================================================================
 * Init / RX
 * ========================================================================== */

void console_init_rtos(void)
{
    s_uart_mutex = osMutexNew(&s_uart_mutex_attr);
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* USART2 also lands here at every full turn of its circular DMA ring;
     * only the console has anything to do. */
    if (huart->Instance == USART1)
    {
        s_pending_cmd = (char)s_rx_byte;
        (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rx_byte, 1);
    }
}
