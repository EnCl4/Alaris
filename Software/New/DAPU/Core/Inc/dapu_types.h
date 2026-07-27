/**
  ******************************************************************************
  * @file    dapu_types.h
  * @brief   Shared state block and on-disk log record of the DAPU.
  *
  *          The binary layout below is mirrored, field for field, by
  *          Tools/decode_log.py through the schema written into the file
  *          header, so the two can never silently drift apart.
  ******************************************************************************
  */

#ifndef __DAPU_TYPES_H
#define __DAPU_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * Status bits (log_record_t.status)
 * ========================================================================== */
#define DAPU_ST_BMP280_OK       (1u << 0)
#define DAPU_ST_MS5611_OK       (1u << 1)
#define DAPU_ST_ICM20948_OK     (1u << 2)
#define DAPU_ST_AK09916_OK      (1u << 3)
#define DAPU_ST_GPS_FIX_3D      (1u << 4)
#define DAPU_ST_SD_OK           (1u << 5)
#define DAPU_ST_RPM_VALID       (1u << 6)
#define DAPU_ST_CALIBRATED      (1u << 7)
#define DAPU_ST_RING_OVERFLOW   (1u << 8)
#define DAPU_ST_SD_ERROR        (1u << 9)
#define DAPU_ST_GPS_TIMEOUT     (1u << 10)
#define DAPU_ST_IMU_TIMEOUT     (1u << 11)
#define DAPU_ST_BARO_TIMEOUT    (1u << 12)

/* ==========================================================================
 * Live state, shared between the acquisition tasks and the logger.
 * Every access must be wrapped by dapu_state_lock()/dapu_state_unlock().
 * ==========================================================================
 * The *_seq counters increment once per successful sensor update.  Comparing
 * their slope against the nominal task rate is exactly the "configured rate
 * vs. measured rate" verification asked for in the V&V section.
 */
typedef struct
{
    /* --- IMU (ICM20948 + AK09916) ------------------------------------- */
    float    accel_g[3];        /* x, y, z  [g]    */
    float    gyro_dps[3];       /* p, q, r  [deg/s], gyro bias removed */
    float    gyro_bias_dps[3];  /* bias measured during boot calibration */
    float    mag_ut[3];         /* x, y, z  [uT]   */
    uint32_t imu_seq;
    uint32_t mag_seq;
    uint32_t imu_last_tick;

    /* --- BMP280 -------------------------------------------------------- */
    float    bmp_temp_c;
    float    bmp_press_pa;
    float    bmp_alt_m;         /* relative to the boot reference */
    float    bmp_press_ref_pa;
    uint32_t bmp_seq;
    uint32_t bmp_last_tick;

    /* --- MS5611 -------------------------------------------------------- */
    float    ms_temp_c;
    float    ms_press_pa;
    float    ms_alt_m;          /* relative to the boot reference */
    float    ms_press_ref_pa;
    uint32_t ms_seq;
    uint32_t ms_last_tick;

    /* --- Analog, raw counts (latest scan) ------------------------------ */
    uint16_t adc_raw[7];

    /* --- Analog, engineering units ------------------------------------- */
    float    pitot_dp_pa;       /* differential pressure  [Pa]   */
    float    ias_mps;           /* indicated airspeed     [m/s]  */
    float    tas_mps;           /* density corrected      [m/s]  */
    float    rho_kgm3;          /* air density used for the correction */
    float    aoa_deg;
    float    defl_deg[3];       /* aileron, elevator, rudder */
    float    vbat_v;
    float    pitot_zero_v;      /* measured at boot */
    uint32_t pitot_seq;
    uint32_t aoa_seq;
    uint32_t flex_seq;

    /* --- RPM / thrust (microphone FFT) --------------------------------- */
    float    blade_hz;
    float    rpm;
    float    thrust_n;
    float    rpm_snr;
    uint32_t rpm_seq;

    /* --- GPS (u-blox NAV-PVT) ------------------------------------------ */
    int32_t  gps_lat_1e7;
    int32_t  gps_lon_1e7;
    int32_t  gps_height_mm;
    int32_t  gps_hmsl_mm;
    int32_t  gps_vel_n_mms;
    int32_t  gps_vel_e_mms;
    int32_t  gps_vel_d_mms;
    int32_t  gps_gspeed_mms;
    int32_t  gps_headmot_1e5;
    uint32_t gps_hacc_mm;
    uint32_t gps_vacc_mm;
    uint16_t gps_year;
    uint8_t  gps_month, gps_day, gps_hour, gps_min, gps_sec;
    uint8_t  gps_fix_type;
    uint8_t  gps_num_sv;
    uint32_t gps_seq;
    uint32_t gps_last_tick;

    /* --- Global ---------------------------------------------------------*/
    uint16_t status;
} dapu_state_t;

/* ==========================================================================
 * On-disk record - 224 bytes, all fields naturally aligned, little endian.
 * ========================================================================== */
typedef struct __attribute__((packed))
{
    uint32_t magic;             /* LOG_RECORD_MAGIC, lets the decoder resync */
    uint32_t sample_id;         /* +1 per record written                     */
    uint32_t t_us;              /* microseconds since boot (DWT cycle count) */
    uint16_t status;
    uint16_t reserved0;

    float    ax, ay, az;        /* [g]     */
    float    gx, gy, gz;        /* [deg/s] */
    float    mx, my, mz;        /* [uT]    */
    uint32_t imu_seq;

    float    bmp_temp_c, bmp_press_pa, bmp_alt_m;
    uint32_t bmp_seq;

    float    ms_temp_c, ms_press_pa, ms_alt_m;
    uint32_t ms_seq;

    uint16_t raw_flex_ail, raw_aoa, raw_flex_ele, raw_flex_rud;
    uint16_t raw_vbat, raw_pitot, raw_mic;
    uint16_t reserved1;

    float    pitot_dp_pa, ias_mps, tas_mps;
    float    aoa_deg;
    float    defl_ail_deg, defl_ele_deg, defl_rud_deg;
    float    vbat_v, rho_kgm3;

    float    blade_hz, rpm, thrust_n, rpm_snr;
    uint32_t rpm_seq;

    int32_t  gps_lat_1e7, gps_lon_1e7, gps_height_mm, gps_hmsl_mm;
    int32_t  gps_vel_n_mms, gps_vel_e_mms, gps_vel_d_mms, gps_gspeed_mms;
    int32_t  gps_headmot_1e5;
    uint32_t gps_hacc_mm, gps_vacc_mm;
    uint16_t gps_year;
    uint8_t  gps_month, gps_day, gps_hour, gps_min, gps_sec;
    uint8_t  gps_fix_type, gps_num_sv, reserved2;
    uint16_t reserved3;
    uint32_t gps_seq;

    uint16_t crc16;             /* CRC-16/CCITT-FALSE over the first 220 B */
    uint16_t reserved4;
} log_record_t;

#define LOG_RECORD_MAGIC   0xD1A5D1A5u
#define LOG_RECORD_SIZE    224u

/* Fails to compile if a field is ever added without fixing the schema. */
_Static_assert(sizeof(log_record_t) == LOG_RECORD_SIZE,
               "log_record_t must stay 224 bytes - update LOG_RECORD_SCHEMA too");

/* Python struct format describing the record above, byte for byte. */
#define LOG_RECORD_FORMAT  "<IIIHH9fI3fI3fI7HH9f4fI9i2IH8BHIHH"

/* Field names, in the same order the format expands to (69 entries). */
#define LOG_RECORD_NAMES \
    "magic,sample_id,t_us,status,reserved0," \
    "ax,ay,az,gx,gy,gz,mx,my,mz,imu_seq," \
    "bmp_temp_c,bmp_press_pa,bmp_alt_m,bmp_seq," \
    "ms_temp_c,ms_press_pa,ms_alt_m,ms_seq," \
    "raw_flex_ail,raw_aoa,raw_flex_ele,raw_flex_rud,raw_vbat,raw_pitot,raw_mic,reserved1," \
    "pitot_dp_pa,ias_mps,tas_mps,aoa_deg,defl_ail_deg,defl_ele_deg,defl_rud_deg,vbat_v,rho_kgm3," \
    "blade_hz,rpm,thrust_n,rpm_snr,rpm_seq," \
    "gps_lat_1e7,gps_lon_1e7,gps_height_mm,gps_hmsl_mm," \
    "gps_vel_n_mms,gps_vel_e_mms,gps_vel_d_mms,gps_gspeed_mms,gps_headmot_1e5," \
    "gps_hacc_mm,gps_vacc_mm," \
    "gps_year,gps_month,gps_day,gps_hour,gps_min,gps_sec,gps_fix_type,gps_num_sv,reserved2," \
    "reserved3,gps_seq,crc16,reserved4"

/* ==========================================================================
 * File header - one 512 byte block at the very start of every log file.
 * ========================================================================== */
#define LOG_FILE_MAGIC     "ALARIS-DAPU-LOG"
#define LOG_FILE_VERSION   1u
/* 1 kB: the schema string alone is ~670 characters. */
#define LOG_HEADER_SIZE    1024u

typedef struct __attribute__((packed))
{
    char     magic[16];         /* LOG_FILE_MAGIC, NUL padded */
    uint32_t version;
    uint32_t record_size;
    float    base_rate_hz;      /* nominal logging rate  */
    float    adc_scan_rate_hz;  /* real microphone rate  */
    uint32_t reserved[4];
    char     schema[LOG_HEADER_SIZE - 48];  /* "<format>\n<names>" */
} log_header_t;

_Static_assert(sizeof(log_header_t) == LOG_HEADER_SIZE,
               "log_header_t size mismatch");

#endif /* __DAPU_TYPES_H */
