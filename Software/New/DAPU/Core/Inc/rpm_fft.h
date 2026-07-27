/**
  ******************************************************************************
  * @file    rpm_fft.h
  * @brief   Propeller RPM / thrust estimation from the electret microphone.
  ******************************************************************************
  * N = 1024 points at ADC_SCAN_RATE_HZ (~1024 Hz) gives a 1 s window and a
  * 1 Hz bin, i.e. ~30 RPM per bin for a two bladed propeller, as derived in
  * Embarcado.tex. A Hann window limits spectral leakage and the peak search is
  * restricted to RPM_SEARCH_FMIN_HZ..RPM_SEARCH_FMAX_HZ.
  ******************************************************************************
  */

#ifndef __RPM_FFT_H
#define __RPM_FFT_H

#include <stdint.h>
#include <stdbool.h>

/** Builds the Hann window and the FFT tables. Call once at boot. */
void   rpm_fft_init(void);

/** Scratch buffer of RPM_FFT_SIZE floats owned by this module; hand it to
 *  analog_mic_wait_block() and then to rpm_fft_process(). */
float *rpm_fft_sample_buffer(void);

/** Runs the FFT and extracts the blade passing frequency.
 *  @param samples   RPM_FFT_SIZE raw ADC counts (destroyed by the call)
 *  @param blade_hz  blade passing frequency [Hz]
 *  @param rpm       propeller speed [RPM]
 *  @param snr       peak magnitude divided by the in-band mean
 *  @return false when the peak is not significant (motor off / only noise) */
bool   rpm_fft_process(float *samples, float *blade_hz, float *rpm, float *snr);

/** Static thrust estimate from the fitted T = k * rpm^2 curve. */
float  rpm_to_thrust_n(float rpm);

#endif /* __RPM_FFT_H */
