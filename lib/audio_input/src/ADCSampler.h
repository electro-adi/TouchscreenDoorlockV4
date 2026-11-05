#pragma once

#include <driver/adc.h>
#include "I2SSampler.h"

// Software gain (original project: 15). Lower it if you hear clipping.
#ifndef ADC_MIC_GAIN
#define ADC_MIC_GAIN 12.0f
#endif

// Gentle high-pass (Hz) that only removes DC drift / handling thumps. Set to 0 to disable.
#ifndef ADC_MIC_HPF_HZ
#define ADC_MIC_HPF_HZ 80.0f
#endif

// Rate the rest of the pipeline expects (must equal SAMPLE_RATE).
#ifndef ADC_ACTUAL_RATE
#define ADC_ACTUAL_RATE 16000.0f
#endif

// The legacy I2S driver in ADC mode derives its clock as  mclk = rate * 2 * 16  and needs
//   160 MHz / mclk < 256   ->   rate >= ~19.6 kHz.
// Below that it prints "sample rate is too small ... ADC/DAC clock calculate failed" and the ADC runs on
// a leftover clock at the WRONG speed (that is what produced the chopped / sped-up audio).
// So configure the ADC at 32 kHz (= 2 x SAMPLE_RATE) and let read() average pairs down to 16 kHz.
#define ADC_MIN_I2S_RATE 19700.0f

// Safety net: read() also MEASURES the real ADC rate right after start (a quick check at ~0.45 s that only
// reacts to gross errors, a precise one at ~1.2 s) and trusts the measurement if it disagrees with the
// configured rate. The result is remembered, so the next transmission starts with the right value.

// Uncomment to print 48 raw 12-bit ADC values once per second while transmitting
// #define ADC_DEBUG_RAW

struct Biquad
{
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    float z1 = 0, z2 = 0;
    // transposed direct form II
    inline float process(float x)
    {
        float y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
};

class ADCSampler : public I2SSampler
{
private:
    adc_unit_t m_adcUnit;
    adc1_channel_t m_adcChannel;

    static const int IN_BLOCK = 512;   // raw ADC samples read per call at most
    int16_t m_in[IN_BLOCK];

    float m_native = 32000.0f;   // raw ADC rate the resampler assumes (starts as the configured rate)
    uint8_t m_cal_stage = 0;     // 0 = waiting for window 1, 1 = measuring window 1, 2 = waiting for window 2, 3 = measuring window 2, 4 = done
    uint32_t m_cal_t0 = 0;       // micros() when the current measuring window opened
    uint32_t m_cal_n = 0;        // raw samples counted inside the window
    uint32_t m_start_us = 0;     // micros() at start

    float m_acc = 0.0f;      // resampler: running sum of the current output bin
    float m_fill = 0.0f;     // resampler: how much of the current bin is filled (in input samples)
    float m_dc = -1.0f;      // running estimate of the mic's DC bias (raw ADC counts)
    int m_lastValid = 2048;  // last sample that was not a full-scale glitch
    Biquad m_hpf[2];         // 4th-order Butterworth high-pass (two cascaded biquads)
    int m_warmup = 0;        // output samples still to mute after start (ADC start-up transient)

    void designHighpass(Biquad &f, float fc, float fs, float q);
    void calibrate(int n_in);

protected:
    void configureI2S();
    void unConfigureI2S();

public:
    ADCSampler(adc_unit_t adc_unit, adc1_channel_t adc_channel, const i2s_config_t &i2s_config);
    static void forgetLearnedRate();   // drop a rate that was measured earlier (normally never needed)
    // 'samples' must have room for 'count' samples; returns the number written (about count-1 per call)
    virtual int read(int16_t *samples, int count);
};