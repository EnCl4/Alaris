/**
  ******************************************************************************
  * @file    dapu_config.h
  * @brief   Single place for every tunable constant of the DAPU firmware.
  *
  *          Scope of this build: data acquisition + processing + SD logging.
  *          The GNCU link (SPI2) and the CGU link (USART3) are NOT implemented.
  ******************************************************************************
  */

#ifndef __DAPU_CONFIG_H
#define __DAPU_CONFIG_H

/* ==========================================================================
 * Base timing
 * ========================================================================== */

/* Base loop of the DAPU (Embarcado.tex, sec. DAPU): 5 ms = 200 Hz. */
#define DAPU_BASE_RATE_HZ            200u
#define DAPU_BASE_PERIOD_MS          (1000u / DAPU_BASE_RATE_HZ)   /* 5 ms  */

/* Per-sensor task periods, in ms (Tab. Freqsensores). */
#define TASK_PERIOD_IMU_MS           5u    /* 200 Hz */
#define TASK_PERIOD_PITOT_MS         5u    /* 200 Hz */
#define TASK_PERIOD_AOA_MS           5u    /* 200 Hz */
#define TASK_PERIOD_FLEX_MS          5u    /* 200 Hz */
#define TASK_PERIOD_BMP_MS           10u   /* 100 Hz */
#define TASK_PERIOD_MS5611_MS        10u   /* state machine -> 50 Hz pressure */
#define TASK_PERIOD_GPS_MS           10u   /* drains the UART DMA ring        */
#define TASK_PERIOD_LOG_MS           5u    /* 200 Hz record rate              */
#define TASK_PERIOD_PLOT_MS          50u   /* 20 Hz human-readable UART plot  */

/* ==========================================================================
 * ADC1 - all analog channels on one timer-triggered scan
 * ==========================================================================
 * A single 7-channel scan is triggered by TIM2 at ADC_SCAN_RATE_HZ and moved
 * to memory by DMA1_Stream0 in circular mode.  The microphone therefore gets
 * an exactly periodic, jitter-free sample clock (mandatory for the FFT), and
 * the 200 Hz tasks simply decimate the same buffer.
 */

/* Rank order inside one scan - must match MX_ADC1_Init(). */
#define ADC_IDX_FLEX_AIL             0u   /* PA0  ADC1_INP16 - aileron  flex */
#define ADC_IDX_AOA                  1u   /* PA1  ADC1_INP17 - AOA vane pot  */
#define ADC_IDX_FLEX_ELE             2u   /* PC0  ADC1_INP10 - elevator flex */
#define ADC_IDX_FLEX_RUD             3u   /* PC1  ADC1_INP11 - rudder   flex */
#define ADC_IDX_VBAT                 4u   /* PC4  ADC1_INP4  - battery  div  */
#define ADC_IDX_PITOT                5u   /* PC5  ADC1_INP8  - INA333 out    */
#define ADC_IDX_MIC                  6u   /* PB1  ADC1_INP5  - electret mic  */
#define ADC_N_CHANNELS               7u

/* TIM2 (32 bit, 100 MHz kernel): ARR+1 = 97656 -> 1024.0026 Hz.
 * 1024 Hz cannot be generated exactly from 100 MHz; the 2.6 ppm error is
 * carried into ADC_SCAN_RATE_HZ so the FFT bin scaling stays exact. */
#define ADC_TIM2_PRESCALER           0u
#define ADC_TIM2_PERIOD              97655u          /* ARR */
#define ADC_TIM2_KERNEL_HZ           100000000.0f
#define ADC_SCAN_RATE_HZ             (ADC_TIM2_KERNEL_HZ / (float)(ADC_TIM2_PERIOD + 1u))

/* One DMA half = one FFT block = 1 s of microphone signal. */
#define ADC_BLOCK_SCANS              1024u
#define ADC_DMA_LENGTH               (2u * ADC_BLOCK_SCANS * ADC_N_CHANNELS)

/* Number of consecutive scans averaged to produce one 200 Hz sample
 * (1024 / 200 = 5.12 -> boxcar over 5 scans). */
#define ADC_DECIMATION_SCANS         5u

#define ADC_FULL_SCALE               65535.0f        /* 16 bit */
#define ADC_VREF_V                   3.3f

/* ==========================================================================
 * Analog signal conditioning
 * ========================================================================== */

/* Battery: resistive divider ratio Vbat / Vadc.  MEASURE YOURS AND SET IT. */
#define VBAT_DIVIDER_RATIO           11.0f           /* e.g. 100k / 10k */

/* Pitot: MPS20N0040D bridge + INA333 (G = 1000) + buffer.
 * dp[Pa] = (Vadc - Vzero) / PITOT_V_PER_PA.  The zero is measured at boot.
 * PITOT_V_PER_PA comes from the design point in Embarcado.tex:
 * 0.3063 V at 245 Pa -> 1.25e-3 V/Pa.  Re-calibrate against a reference. */
#define PITOT_V_PER_PA               1.25e-3f
#define PITOT_DEADBAND_PA            0.5f            /* |dp| below this -> 0 */

/* Angle of attack vane potentiometer: aoa_deg = AOA_GAIN * Vadc + AOA_OFFSET */
#define AOA_GAIN_DEG_PER_V           1.0f
#define AOA_OFFSET_DEG               1.0f

/* Flex sensors: deflection_deg = FLEX_x_GAIN * Vadc + FLEX_x_OFFSET.
 * Placeholders (a = 1, b = 1) - replace after bench calibration. */
#define FLEX_AIL_GAIN_DEG_PER_V      1.0f
#define FLEX_AIL_OFFSET_DEG          1.0f
#define FLEX_ELE_GAIN_DEG_PER_V      1.0f
#define FLEX_ELE_OFFSET_DEG          1.0f
#define FLEX_RUD_GAIN_DEG_PER_V      1.0f
#define FLEX_RUD_OFFSET_DEG          1.0f

/* ==========================================================================
 * RPM / thrust estimation (microphone + FFT)
 * ========================================================================== */

#define RPM_FFT_SIZE                 1024u           /* = ADC_BLOCK_SCANS */
#define RPM_N_BLADES                 2.0f
#define RPM_SEARCH_FMIN_HZ           10.0f
#define RPM_SEARCH_FMAX_HZ           200.0f
/* Peak must exceed this multiple of the mean magnitude in the search band,
 * otherwise the estimate is rejected (motor off / only noise). */
#define RPM_PEAK_SNR_MIN             4.0f
/* Static-test thrust fit: T[N] = RPM_THRUST_K * rpm^2.  Placeholder. */
#define RPM_THRUST_K                 0.0f

/* Set to 0 to use the small built-in radix-2 FFT instead of CMSIS-DSP. */
#define DAPU_USE_CMSIS_DSP           1

/* ==========================================================================
 * Barometers
 * ========================================================================== */

#define BARO_SEA_LEVEL_PA            101325.0f
/* Boot reference averaging window (Embarcado.tex: ~2 s while stationary). */
#define BARO_REF_AVERAGE_MS          2000u

/* ==========================================================================
 * IMU (ICM20948)
 * ========================================================================== */

/* RA.04 asks for +/-3 g and the airframe is slow in yaw: 8 g / 1000 dps
 * keeps good resolution with margin. */
#define ICM_ACCEL_FULL_SCALE         _8g
#define ICM_GYRO_FULL_SCALE          _1000dps
/* Gyro bias window at boot, aircraft stationary. */
#define GYRO_BIAS_SAMPLES            400u            /* 400 @ 200 Hz = 2 s */

/* ==========================================================================
 * GPS (u-blox NEO-M8N)
 * ========================================================================== */

#define GPS_BAUD_DEFAULT             9600u           /* factory default   */
#define GPS_BAUD_TARGET              115200u         /* needed for 10 Hz  */
#define GPS_RATE_HZ                  10u
#define GPS_RX_DMA_SIZE              512u
/* CFG-NAV5 dynamic model. The F4 bench code used 3 (pedestrian); 6 is
 * "airborne < 1 g", which matches the aircraft and stops the receiver from
 * over-filtering the velocity solution. Set back to 3 for static tests. */
#define GPS_DYN_MODEL                6u

/* ==========================================================================
 * Logging
 * ========================================================================== */

/* Ring buffer between the 200 Hz sampler and the SD writer.  Must be a
 * power of two.  32 kB ~= 138 records of slack. */
#define LOG_RING_SIZE                32768u
/* Minimum contiguous payload handed to f_write(). */
#define LOG_WRITE_CHUNK              4096u
/* f_sync() period - bounds the data lost on a brutal power cut. */
#define LOG_SYNC_PERIOD_MS           1000u
#define LOG_FILE_PREFIX              "LOG"           /* LOG0001.BIN (8.3) */

/* ==========================================================================
 * UART console (USART1)
 * ========================================================================== */

/* The human-readable plot competes with the SD card for CPU time.  It can be
 * toggled at run time by sending 'p' on USART1; this is only its boot value. */
#define PLOT_DEFAULT_ENABLED         1

#endif /* __DAPU_CONFIG_H */
