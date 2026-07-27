/**
  ******************************************************************************
  * @file    dapu_log.c
  * @brief   200 Hz binary flight log: state snapshot -> ring buffer -> SD card.
  ******************************************************************************
  */

#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "dapu_config.h"
#include "dapu_state.h"
#include "dapu_log.h"
#include <string.h>
#include <stdio.h>

/* ==========================================================================
 * Lock free byte ring (one producer task, one consumer task)
 * ========================================================================== */

#if (LOG_RING_SIZE & (LOG_RING_SIZE - 1u)) != 0u
#error "LOG_RING_SIZE must be a power of two"
#endif
#define RING_MASK   (LOG_RING_SIZE - 1u)

static uint8_t s_ring[LOG_RING_SIZE] __attribute__((aligned(4)));
static volatile uint32_t s_head;        /* written by the sampler */
static volatile uint32_t s_tail;        /* written by the writer  */

static inline uint32_t ring_used(void)
{
    return (s_head - s_tail) & RING_MASK;
}

static inline uint32_t ring_free(void)
{
    return RING_MASK - ring_used();
}

/* ==========================================================================
 * File state
 * ========================================================================== */

static FIL      s_file;
static bool     s_open;
static bool     s_mounted;
static char     s_name[16];
static uint32_t s_sample_id;
static uint32_t s_dropped;
static uint32_t s_bytes_written;
static uint32_t s_last_sync_tick;

static osSemaphoreId_t s_writer_sem;

void log_init_rtos(void)
{
    s_writer_sem = osSemaphoreNew(1, 0, NULL);
}

/* ==========================================================================
 * CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)
 * ========================================================================== */

static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFu;

    while (len-- > 0u)
    {
        crc ^= (uint16_t)((uint16_t)(*data++) << 8);
        for (uint8_t bit = 0; bit < 8u; bit++)
        {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ==========================================================================
 * Record assembly
 * ========================================================================== */

static void build_record(log_record_t *r)
{
    memset(r, 0, sizeof(*r));

    r->magic     = LOG_RECORD_MAGIC;
    r->sample_id = s_sample_id++;
    r->t_us      = dapu_micros();

    dapu_state_lock();

    r->status = g_state.status;

    r->ax = g_state.accel_g[0];
    r->ay = g_state.accel_g[1];
    r->az = g_state.accel_g[2];
    r->gx = g_state.gyro_dps[0];
    r->gy = g_state.gyro_dps[1];
    r->gz = g_state.gyro_dps[2];
    r->mx = g_state.mag_ut[0];
    r->my = g_state.mag_ut[1];
    r->mz = g_state.mag_ut[2];
    r->imu_seq = g_state.imu_seq;

    r->bmp_temp_c   = g_state.bmp_temp_c;
    r->bmp_press_pa = g_state.bmp_press_pa;
    r->bmp_alt_m    = g_state.bmp_alt_m;
    r->bmp_seq      = g_state.bmp_seq;

    r->ms_temp_c   = g_state.ms_temp_c;
    r->ms_press_pa = g_state.ms_press_pa;
    r->ms_alt_m    = g_state.ms_alt_m;
    r->ms_seq      = g_state.ms_seq;

    r->raw_flex_ail = g_state.adc_raw[ADC_IDX_FLEX_AIL];
    r->raw_aoa      = g_state.adc_raw[ADC_IDX_AOA];
    r->raw_flex_ele = g_state.adc_raw[ADC_IDX_FLEX_ELE];
    r->raw_flex_rud = g_state.adc_raw[ADC_IDX_FLEX_RUD];
    r->raw_vbat     = g_state.adc_raw[ADC_IDX_VBAT];
    r->raw_pitot    = g_state.adc_raw[ADC_IDX_PITOT];
    r->raw_mic      = g_state.adc_raw[ADC_IDX_MIC];

    r->pitot_dp_pa  = g_state.pitot_dp_pa;
    r->ias_mps      = g_state.ias_mps;
    r->tas_mps      = g_state.tas_mps;
    r->aoa_deg      = g_state.aoa_deg;
    r->defl_ail_deg = g_state.defl_deg[0];
    r->defl_ele_deg = g_state.defl_deg[1];
    r->defl_rud_deg = g_state.defl_deg[2];
    r->vbat_v       = g_state.vbat_v;
    r->rho_kgm3     = g_state.rho_kgm3;

    r->blade_hz = g_state.blade_hz;
    r->rpm      = g_state.rpm;
    r->thrust_n = g_state.thrust_n;
    r->rpm_snr  = g_state.rpm_snr;
    r->rpm_seq  = g_state.rpm_seq;

    r->gps_lat_1e7     = g_state.gps_lat_1e7;
    r->gps_lon_1e7     = g_state.gps_lon_1e7;
    r->gps_height_mm   = g_state.gps_height_mm;
    r->gps_hmsl_mm     = g_state.gps_hmsl_mm;
    r->gps_vel_n_mms   = g_state.gps_vel_n_mms;
    r->gps_vel_e_mms   = g_state.gps_vel_e_mms;
    r->gps_vel_d_mms   = g_state.gps_vel_d_mms;
    r->gps_gspeed_mms  = g_state.gps_gspeed_mms;
    r->gps_headmot_1e5 = g_state.gps_headmot_1e5;
    r->gps_hacc_mm     = g_state.gps_hacc_mm;
    r->gps_vacc_mm     = g_state.gps_vacc_mm;
    r->gps_year        = g_state.gps_year;
    r->gps_month       = g_state.gps_month;
    r->gps_day         = g_state.gps_day;
    r->gps_hour        = g_state.gps_hour;
    r->gps_min         = g_state.gps_min;
    r->gps_sec         = g_state.gps_sec;
    r->gps_fix_type    = g_state.gps_fix_type;
    r->gps_num_sv      = g_state.gps_num_sv;
    r->gps_seq         = g_state.gps_seq;

    dapu_state_unlock();

    r->crc16 = crc16_ccitt((const uint8_t *)r, LOG_RECORD_SIZE - 4u);
}

/* ==========================================================================
 * Producer
 * ========================================================================== */

void log_sample(void)
{
    log_record_t rec;

    if (!s_open)
    {
        return;
    }

    build_record(&rec);

    if (ring_free() < LOG_RECORD_SIZE)
    {
        s_dropped++;
        dapu_state_set_status(DAPU_ST_RING_OVERFLOW);
        return;
    }

    const uint8_t *src = (const uint8_t *)&rec;
    uint32_t head = s_head;
    uint32_t first = LOG_RING_SIZE - (head & RING_MASK);

    if (first > LOG_RECORD_SIZE)
    {
        first = LOG_RECORD_SIZE;
    }

    memcpy(&s_ring[head & RING_MASK], src, first);
    if (first < LOG_RECORD_SIZE)
    {
        memcpy(&s_ring[0], src + first, LOG_RECORD_SIZE - first);
    }

    s_head = (head + LOG_RECORD_SIZE) & RING_MASK;

    if (ring_used() >= LOG_WRITE_CHUNK)
    {
        osSemaphoreRelease(s_writer_sem);
    }
}

/* ==========================================================================
 * File open / close
 * ========================================================================== */

static void write_header(void)
{
    log_header_t hdr;
    UINT bw = 0;

    memset(&hdr, 0, sizeof(hdr));
    memcpy(hdr.magic, LOG_FILE_MAGIC, sizeof(LOG_FILE_MAGIC) - 1u);
    hdr.version          = LOG_FILE_VERSION;
    hdr.record_size      = LOG_RECORD_SIZE;
    hdr.base_rate_hz     = (float)DAPU_BASE_RATE_HZ;
    hdr.adc_scan_rate_hz = ADC_SCAN_RATE_HZ;

    snprintf(hdr.schema, sizeof(hdr.schema), "%s\n%s",
             LOG_RECORD_FORMAT, LOG_RECORD_NAMES);

    (void)f_write(&s_file, &hdr, sizeof(hdr), &bw);
    (void)f_sync(&s_file);
}

bool log_open(void)
{
    FRESULT res;

    if (s_open)
    {
        return true;
    }

    if (!s_mounted)
    {
        res = f_mount(&SDFatFS, SDPath, 1);
        if (res != FR_OK)
        {
            dapu_state_set_status(DAPU_ST_SD_ERROR);
            return false;
        }
        s_mounted = true;
    }

    /* Pick the first unused LOGnnnn.BIN so runs never overwrite each other.
     * Long file names are disabled in ffconf.h, so 8.3 it is. */
    FILINFO fno;
    uint32_t index = 1;

    for (; index < 10000u; index++)
    {
        snprintf(s_name, sizeof(s_name), "%s%04lu.BIN",
                 LOG_FILE_PREFIX, (unsigned long)index);
        if (f_stat(s_name, &fno) != FR_OK)
        {
            break;
        }
    }

    res = f_open(&s_file, s_name, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK)
    {
        s_name[0] = '\0';
        dapu_state_set_status(DAPU_ST_SD_ERROR);
        return false;
    }

    write_header();

    s_head = 0;
    s_tail = 0;
    s_last_sync_tick = HAL_GetTick();
    s_open = true;

    dapu_state_clear_status(DAPU_ST_SD_ERROR);
    dapu_state_set_status(DAPU_ST_SD_OK);
    return true;
}

void log_close(void)
{
    if (!s_open)
    {
        return;
    }

    s_open = false;             /* stop the producer first */

    /* Drain whatever is still queued before closing. */
    while (ring_used() > 0u)
    {
        uint32_t tail  = s_tail;
        uint32_t chunk = ring_used();
        uint32_t first = LOG_RING_SIZE - (tail & RING_MASK);
        UINT bw = 0;

        if (chunk > first)
        {
            chunk = first;
        }
        if (f_write(&s_file, &s_ring[tail & RING_MASK], chunk, &bw) != FR_OK)
        {
            break;
        }
        s_tail = (tail + bw) & RING_MASK;
        if (bw != chunk)
        {
            break;
        }
    }

    (void)f_close(&s_file);
    dapu_state_clear_status(DAPU_ST_SD_OK);
}

bool log_is_open(void)
{
    return s_open;
}

/* ==========================================================================
 * Consumer
 * ========================================================================== */

void log_service(void)
{
    (void)osSemaphoreAcquire(s_writer_sem, 50u);

    if (!s_open)
    {
        return;
    }

    while (ring_used() >= LOG_WRITE_CHUNK)
    {
        uint32_t tail  = s_tail;
        uint32_t chunk = ring_used();
        uint32_t first = LOG_RING_SIZE - (tail & RING_MASK);
        UINT bw = 0;

        if (chunk > first)
        {
            chunk = first;              /* stop at the wrap point */
        }
        if (chunk > LOG_WRITE_CHUNK)
        {
            chunk = LOG_WRITE_CHUNK;
        }

        if (f_write(&s_file, &s_ring[tail & RING_MASK], chunk, &bw) != FR_OK)
        {
            /* Close directly: log_close() would only retry the same failing
             * f_write() on the rest of the ring. */
            dapu_state_set_status(DAPU_ST_SD_ERROR);
            dapu_state_clear_status(DAPU_ST_SD_OK);
            s_open = false;
            (void)f_close(&s_file);
            return;
        }

        s_tail = (tail + bw) & RING_MASK;
        s_bytes_written += bw;

        HAL_GPIO_TogglePin(LED_SD_GPIO_Port, LED_SD_Pin);

        if (bw != chunk)
        {
            dapu_state_set_status(DAPU_ST_SD_ERROR);
            break;
        }
    }

    if ((HAL_GetTick() - s_last_sync_tick) >= LOG_SYNC_PERIOD_MS)
    {
        s_last_sync_tick = HAL_GetTick();
        if (f_sync(&s_file) != FR_OK)
        {
            dapu_state_set_status(DAPU_ST_SD_ERROR);
        }
    }
}

uint32_t log_dropped_records(void) { return s_dropped; }
uint32_t log_written_records(void) { return s_bytes_written / LOG_RECORD_SIZE; }
const char *log_filename(void)     { return s_name;    }
