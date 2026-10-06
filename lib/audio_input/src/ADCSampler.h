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

// Rate the ESP32's ADC REALLY delivers samples at, measured: ~43.9 kHz no matter what
// sample_rate is configured. read() resamples this down to ADC_ACTUAL_RATE.
// If the "Sent ... = P Hz" printout is not ~16000, set this to  (current value * P / 16000).
#ifndef ADC_NATIVE_RATE
#define ADC_NATIVE_RATE 43880.0f
#endif

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

    static const int IN_BLOCK = 256;   // raw ADC samples read per call (-> at most 94 output samples)
    int16_t m_in[IN_BLOCK];

    float m_acc = 0.0f;      // resampler: running sum of the current output bin
    float m_fill = 0.0f;     // resampler: how much of the current bin is filled (in input samples)
    float m_dc = -1.0f;      // running estimate of the mic's DC bias (raw ADC counts)
    int m_lastValid = 2048;  // last sample that was not a full-scale glitch
    Biquad m_hpf[2];         // 4th-order Butterworth high-pass (two cascaded biquads)
    int m_warmup = 0;        // output samples still to mute after start (ADC start-up transient)

    void designHighpass(Biquad &f, float fc, float fs, float q);

protected:
    void configureI2S();
    void unConfigureI2S();

public:
    ADCSampler(adc_unit_t adc_unit, adc1_channel_t adc_channel, const i2s_config_t &i2s_config);
    // 'samples' must have room for at least 96 samples; returns the number written (~93 per call)
    virtual int read(int16_t *samples, int count);
};