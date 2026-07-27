/**
  ******************************************************************************
  * @file    dapu_analog.c
  * @brief   Timer triggered ADC1 scan + conversion to engineering units.
  ******************************************************************************
  * One TIM2 pulse at ADC_SCAN_RATE_HZ (~1024 Hz) starts a 7 channel regular
  * sequence; DMA1_Stream0 writes it to s_adc_dma in circular mode.  The buffer
  * holds two blocks of ADC_BLOCK_SCANS scans, so the half/complete callbacks
  * hand a full second of microphone signal to the RPM task while the 200 Hz
  * tasks decimate the same data.
  ******************************************************************************
  */

#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "tim.h"
#include "dapu_config.h"
#include "dapu_state.h"
#include "dapu_analog.h"
#include <math.h>
#include <string.h>

/* Dry air gas constant [J/(kg.K)] and ISA sea level density [kg/m^3]. */
#define AIR_R_SPECIFIC   287.05f
#define AIR_RHO_SL       1.225f

/* 32 byte aligned so the buffer stays usable if the D-cache is ever turned on */
__attribute__((aligned(32)))
static volatile uint16_t s_adc_dma[ADC_DMA_LENGTH];

static osSemaphoreId_t s_mic_sem;
static volatile uint8_t s_mic_block;      /* half that just completed */

/* ==========================================================================
 * Start / raw access
 * ========================================================================== */

void analog_init_rtos(void)
{
    s_mic_sem = osSemaphoreNew(1, 0, NULL);
}

void analog_start(void)
{
    memset((void *)s_adc_dma, 0, sizeof(s_adc_dma));

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_dma, ADC_DMA_LENGTH) != HAL_OK)
    {
        Error_Handler();
    }
    /* TIM2 only generates TRGO, no output pin and no interrupt. */
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
}

/** Index of the scan the DMA is about to write (i.e. one past the newest). */
static uint32_t current_scan_index(void)
{
    uint32_t remaining = __HAL_DMA_GET_COUNTER(hadc1.DMA_Handle);
    uint32_t written   = ADC_DMA_LENGTH - remaining;
    return written / ADC_N_CHANNELS;
}

uint16_t analog_latest(uint8_t ch)
{
    uint32_t scan = current_scan_index();
    /* Step back one full scan so we never read a half written sequence. */
    scan = (scan + (2u * ADC_BLOCK_SCANS) - 1u) % (2u * ADC_BLOCK_SCANS);
    return s_adc_dma[(scan * ADC_N_CHANNELS) + ch];
}

void analog_latest_scan(uint16_t out[7])
{
    uint32_t scan = current_scan_index();
    scan = (scan + (2u * ADC_BLOCK_SCANS) - 1u) % (2u * ADC_BLOCK_SCANS);

    const volatile uint16_t *p = &s_adc_dma[scan * ADC_N_CHANNELS];
    for (uint32_t i = 0; i < ADC_N_CHANNELS; i++)
    {
        out[i] = p[i];
    }
}

float analog_avg_counts(uint8_t ch, uint32_t n_scans)
{
    if (n_scans == 0u)
    {
        n_scans = 1u;
    }
    if (n_scans > ADC_BLOCK_SCANS)
    {
        n_scans = ADC_BLOCK_SCANS;
    }

    const uint32_t total_scans = 2u * ADC_BLOCK_SCANS;
    uint32_t scan = (current_scan_index() + total_scans - 1u) % total_scans;

    uint32_t sum = 0u;
    for (uint32_t i = 0; i < n_scans; i++)
    {
        sum += s_adc_dma[(scan * ADC_N_CHANNELS) + ch];
        scan = (scan + total_scans - 1u) % total_scans;
    }
    return (float)sum / (float)n_scans;
}

/* ==========================================================================
 * Engineering units
 * ========================================================================== */

/** Air density from the barometer, falling back to ISA sea level. */
static float air_density(void)
{
    float p = g_state.bmp_press_pa;
    float t = g_state.bmp_temp_c;

    if (p < 1000.0f)                       /* BMP280 absent or not read yet */
    {
        p = g_state.ms_press_pa;
        t = g_state.ms_temp_c;
    }
    if (p < 1000.0f || t < -80.0f || t > 120.0f)
    {
        return AIR_RHO_SL;
    }
    return p / (AIR_R_SPECIFIC * (t + 273.15f));
}

void analog_update_pitot(void)
{
    float counts = analog_avg_counts(ADC_IDX_PITOT, ADC_DECIMATION_SCANS);
    uint16_t raw = analog_latest(ADC_IDX_PITOT);

    dapu_state_lock();

    float volts = analog_counts_to_volts(counts);
    float dp    = (volts - g_state.pitot_zero_v) / PITOT_V_PER_PA;

    if (fabsf(dp) < PITOT_DEADBAND_PA)
    {
        dp = 0.0f;
    }

    float rho  = air_density();
    float sign = (dp < 0.0f) ? -1.0f : 1.0f;
    float mag  = fabsf(dp);

    g_state.pitot_dp_pa = dp;
    g_state.rho_kgm3    = rho;
    g_state.ias_mps     = sign * sqrtf(2.0f * mag / AIR_RHO_SL);
    g_state.tas_mps     = sign * sqrtf(2.0f * mag / rho);
    g_state.adc_raw[ADC_IDX_PITOT] = raw;
    g_state.pitot_seq++;

    dapu_state_unlock();
}

void analog_update_aoa(void)
{
    float counts = analog_avg_counts(ADC_IDX_AOA, ADC_DECIMATION_SCANS);
    uint16_t raw = analog_latest(ADC_IDX_AOA);
    float volts  = analog_counts_to_volts(counts);

    dapu_state_lock();
    g_state.aoa_deg = (AOA_GAIN_DEG_PER_V * volts) + AOA_OFFSET_DEG;
    g_state.adc_raw[ADC_IDX_AOA] = raw;
    g_state.aoa_seq++;
    dapu_state_unlock();
}

void analog_update_flex(void)
{
    float v_ail = analog_counts_to_volts(analog_avg_counts(ADC_IDX_FLEX_AIL, ADC_DECIMATION_SCANS));
    float v_ele = analog_counts_to_volts(analog_avg_counts(ADC_IDX_FLEX_ELE, ADC_DECIMATION_SCANS));
    float v_rud = analog_counts_to_volts(analog_avg_counts(ADC_IDX_FLEX_RUD, ADC_DECIMATION_SCANS));
    float v_bat = analog_counts_to_volts(analog_avg_counts(ADC_IDX_VBAT,     ADC_DECIMATION_SCANS));

    uint16_t scan[ADC_N_CHANNELS];
    analog_latest_scan(scan);

    dapu_state_lock();

    g_state.defl_deg[0] = (FLEX_AIL_GAIN_DEG_PER_V * v_ail) + FLEX_AIL_OFFSET_DEG;
    g_state.defl_deg[1] = (FLEX_ELE_GAIN_DEG_PER_V * v_ele) + FLEX_ELE_OFFSET_DEG;
    g_state.defl_deg[2] = (FLEX_RUD_GAIN_DEG_PER_V * v_rud) + FLEX_RUD_OFFSET_DEG;
    g_state.vbat_v      = v_bat * VBAT_DIVIDER_RATIO;

    g_state.adc_raw[ADC_IDX_FLEX_AIL] = scan[ADC_IDX_FLEX_AIL];
    g_state.adc_raw[ADC_IDX_FLEX_ELE] = scan[ADC_IDX_FLEX_ELE];
    g_state.adc_raw[ADC_IDX_FLEX_RUD] = scan[ADC_IDX_FLEX_RUD];
    g_state.adc_raw[ADC_IDX_VBAT]     = scan[ADC_IDX_VBAT];
    g_state.adc_raw[ADC_IDX_MIC]      = scan[ADC_IDX_MIC];
    g_state.flex_seq++;

    dapu_state_unlock();
}

/* ==========================================================================
 * Microphone block handoff
 * ========================================================================== */

bool analog_mic_wait_block(float *dst, uint32_t timeout_ms)
{
    if (osSemaphoreAcquire(s_mic_sem, timeout_ms) != osOK)
    {
        return false;
    }

    const uint32_t base = (uint32_t)s_mic_block * ADC_BLOCK_SCANS * ADC_N_CHANNELS;
    const volatile uint16_t *p = &s_adc_dma[base + ADC_IDX_MIC];

    for (uint32_t i = 0; i < RPM_FFT_SIZE; i++)
    {
        dst[i] = (float)(*p);
        p += ADC_N_CHANNELS;
    }
    return true;
}

/* The DMA runs from boot, well before osKernelStart(), so the callbacks must
 * not touch RTOS objects until the scheduler is actually running. */
static inline void mic_block_ready(uint8_t block)
{
    s_mic_block = block;
    if (s_mic_sem != NULL && osKernelGetState() == osKernelRunning)
    {
        osSemaphoreRelease(s_mic_sem);
    }
}

/* First half of the circular buffer complete. */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        mic_block_ready(0u);
    }
}

/* Second half complete. */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        mic_block_ready(1u);
    }
}
