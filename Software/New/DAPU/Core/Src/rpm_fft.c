/**
  ******************************************************************************
  * @file    rpm_fft.c
  * @brief   Propeller RPM / thrust estimation from the electret microphone.
  ******************************************************************************
  */

#include "dapu_config.h"
#include "rpm_fft.h"
#include <math.h>
#include <string.h>

#if DAPU_USE_CMSIS_DSP
#include "arm_math.h"
#endif

#define FFT_N       RPM_FFT_SIZE
#define FFT_N_BINS  (FFT_N / 2u)

static float s_window[FFT_N];       /* Hann coefficients            */
static float s_samples[FFT_N];      /* shared with the ADC handoff  */
static float s_mag[FFT_N_BINS];

#if DAPU_USE_CMSIS_DSP
static arm_rfft_fast_instance_f32 s_rfft;
static float s_spectrum[FFT_N];
#else
/* Compact in-place radix-2 FFT, used when CMSIS-DSP is not linked. */
static float s_re[FFT_N];
static float s_im[FFT_N];

static void fft_radix2(float *re, float *im, uint32_t n)
{
    /* Bit reversal permutation. */
    for (uint32_t i = 1u, j = 0u; i < n; i++)
    {
        uint32_t bit = n >> 1;
        for (; j & bit; bit >>= 1)
        {
            j ^= bit;
        }
        j ^= bit;

        if (i < j)
        {
            float t;
            t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }

    for (uint32_t len = 2u; len <= n; len <<= 1)
    {
        float ang = -2.0f * (float)M_PI / (float)len;
        float wr  = cosf(ang);
        float wi  = sinf(ang);

        for (uint32_t i = 0; i < n; i += len)
        {
            float cur_r = 1.0f, cur_i = 0.0f;

            for (uint32_t k = 0; k < (len >> 1); k++)
            {
                float ur = re[i + k];
                float ui = im[i + k];
                float vr = re[i + k + (len >> 1)] * cur_r - im[i + k + (len >> 1)] * cur_i;
                float vi = re[i + k + (len >> 1)] * cur_i + im[i + k + (len >> 1)] * cur_r;

                re[i + k] = ur + vr;
                im[i + k] = ui + vi;
                re[i + k + (len >> 1)] = ur - vr;
                im[i + k + (len >> 1)] = ui - vi;

                float nr = cur_r * wr - cur_i * wi;
                cur_i = cur_r * wi + cur_i * wr;
                cur_r = nr;
            }
        }
    }
}
#endif /* DAPU_USE_CMSIS_DSP */

void rpm_fft_init(void)
{
    for (uint32_t i = 0; i < FFT_N; i++)
    {
        s_window[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i / (float)(FFT_N - 1u)));
    }

#if DAPU_USE_CMSIS_DSP
    (void)arm_rfft_fast_init_f32(&s_rfft, FFT_N);
#endif
}

float *rpm_fft_sample_buffer(void)
{
    return s_samples;
}

bool rpm_fft_process(float *samples, float *blade_hz, float *rpm, float *snr)
{
    const float bin_hz = ADC_SCAN_RATE_HZ / (float)FFT_N;

    *blade_hz = 0.0f;
    *rpm      = 0.0f;
    *snr      = 0.0f;

    /* Remove the DC bias of the electret bias network before windowing:
     * a large DC term would leak across the low bins through the window. */
    float mean = 0.0f;
    for (uint32_t i = 0; i < FFT_N; i++)
    {
        mean += samples[i];
    }
    mean /= (float)FFT_N;

    for (uint32_t i = 0; i < FFT_N; i++)
    {
        samples[i] = (samples[i] - mean) * s_window[i];
    }

#if DAPU_USE_CMSIS_DSP
    arm_rfft_fast_f32(&s_rfft, samples, s_spectrum, 0);
    /* s_spectrum packs bin 0 as {DC, Nyquist}; harmless here because the
     * search band never reaches bin 0. */
    arm_cmplx_mag_f32(s_spectrum, s_mag, FFT_N_BINS);
#else
    memcpy(s_re, samples, sizeof(s_re));
    memset(s_im, 0, sizeof(s_im));
    fft_radix2(s_re, s_im, FFT_N);
    for (uint32_t i = 0; i < FFT_N_BINS; i++)
    {
        s_mag[i] = sqrtf((s_re[i] * s_re[i]) + (s_im[i] * s_im[i]));
    }
#endif

    uint32_t bin_lo = (uint32_t)(RPM_SEARCH_FMIN_HZ / bin_hz);
    uint32_t bin_hi = (uint32_t)(RPM_SEARCH_FMAX_HZ / bin_hz);

    if (bin_lo < 1u)             { bin_lo = 1u; }
    if (bin_hi > FFT_N_BINS - 2u){ bin_hi = FFT_N_BINS - 2u; }
    if (bin_lo >= bin_hi)        { return false; }

    uint32_t peak_bin = bin_lo;
    float    peak_mag = s_mag[bin_lo];
    float    band_sum = 0.0f;

    for (uint32_t i = bin_lo; i <= bin_hi; i++)
    {
        band_sum += s_mag[i];
        if (s_mag[i] > peak_mag)
        {
            peak_mag = s_mag[i];
            peak_bin = i;
        }
    }

    float band_mean = band_sum / (float)((bin_hi - bin_lo) + 1u);
    if (band_mean <= 0.0f)
    {
        return false;
    }

    float peak_snr = peak_mag / band_mean;
    *snr = peak_snr;

    if (peak_snr < RPM_PEAK_SNR_MIN)
    {
        return false;                   /* motor off, or just broadband noise */
    }

    /* Parabolic interpolation across the three bins around the peak pushes
     * the resolution below the 1 Hz bin width. */
    float ym1 = s_mag[peak_bin - 1u];
    float y0  = s_mag[peak_bin];
    float yp1 = s_mag[peak_bin + 1u];
    float denom = (ym1 - (2.0f * y0)) + yp1;
    float delta = (denom != 0.0f) ? (0.5f * (ym1 - yp1) / denom) : 0.0f;

    if (delta > 0.5f)  { delta =  0.5f; }
    if (delta < -0.5f) { delta = -0.5f; }

    *blade_hz = ((float)peak_bin + delta) * bin_hz;
    *rpm      = (*blade_hz) * 60.0f / RPM_N_BLADES;

    return true;
}

float rpm_to_thrust_n(float rpm)
{
    return RPM_THRUST_K * rpm * rpm;
}
