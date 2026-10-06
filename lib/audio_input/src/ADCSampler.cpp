#include <Arduino.h>
#include <math.h>
#include "ADCSampler.h"

#if CONFIG_IDF_TARGET_ESP32

ADCSampler::ADCSampler(adc_unit_t adcUnit, adc1_channel_t adcChannel, const i2s_config_t &i2s_config) : I2SSampler(I2S_NUM_0, i2s_config)
{
    m_adcUnit = adcUnit;
    m_adcChannel = adcChannel;
}

void ADCSampler::designHighpass(Biquad &f, float fc, float fs, float q)
{
    float w0 = 2.0f * (float)M_PI * fc / fs;
    float c = cosf(w0);
    float alpha = sinf(w0) / (2.0f * q);
    float a0 = 1.0f + alpha;
    f.b0 = ((1.0f + c) / 2.0f) / a0;
    f.b1 = (-(1.0f + c)) / a0;
    f.b2 = f.b0;
    f.a1 = (-2.0f * c) / a0;
    f.a2 = (1.0f - alpha) / a0;
    f.z1 = 0;
    f.z2 = 0;
}

void ADCSampler::configureI2S()
{
    // reset all state from the previous transmission
    m_acc = 0.0f;
    m_fill = 0.0f;
    m_dc = -1.0f;
    m_lastValid = 2048;
    // filters run AFTER resampling, i.e. at the 16 kHz output rate
    designHighpass(m_hpf[0], ADC_MIC_HPF_HZ, ADC_ACTUAL_RATE, 0.5412f);  // 4th-order Butterworth
    designHighpass(m_hpf[1], ADC_MIC_HPF_HZ, ADC_ACTUAL_RATE, 1.3066f);
    m_warmup = 1024;  // mute the first ~64 ms: the ADC ramps up from 0 when it starts

    //init ADC pad
    i2s_set_adc_mode(m_adcUnit, m_adcChannel);
    // enable the adc
    i2s_adc_enable(m_i2sPort);
}

void ADCSampler::unConfigureI2S()
{
    // make sure ot do this or the ADC is locked
    i2s_adc_disable(m_i2sPort);
}

int ADCSampler::read(int16_t *samples, int count)
{
    // read a fixed block of raw samples (the ADC runs at ~ADC_NATIVE_RATE)
    size_t bytes_read = 0;
    i2s_read(m_i2sPort, m_in, sizeof(m_in), &bytes_read, portMAX_DELAY);
    int n_in = bytes_read / sizeof(int16_t);

#ifdef ADC_DEBUG_RAW
    static uint32_t lastDump = 0;
    if (millis() - lastDump > 1000)
    {
        lastDump = millis();
        Serial.print("RAW12:");
        for (int i = 0; i < n_in && i < 48; i++)
            Serial.printf(" %u", (unsigned)(uint16_t(m_in[i]) & 0xfff));
        Serial.println();
    }
#endif

    const float binWidth = ADC_NATIVE_RATE / ADC_ACTUAL_RATE;   // input samples per output sample (~2.74)
    int n_out = 0;

    for (int i = 0; i < n_in; i++)
    {
        int raw = uint16_t(m_in[i]) & 0xfff;

        // 1) glitch rejection: 0 and 4095 are the ADC rails, hold the last good value
        if (raw == 0 || raw == 4095)
            raw = m_lastValid;
        else
            m_lastValid = raw;

        // 2) resample ~43.9 kHz -> 16 kHz by averaging (box filter doubles as anti-alias filter)
        float remaining = 1.0f;
        while (remaining > 1e-6f)
        {
            float space = binWidth - m_fill;
            float take = remaining < space ? remaining : space;
            m_acc += raw * take;
            m_fill += take;
            remaining -= take;

            if (m_fill >= binWidth - 1e-6f)
            {
                float avg = m_acc / binWidth;
                m_acc = 0.0f;
                m_fill = 0.0f;

                // 3) slow DC tracker so the signal is centred on zero
                if (m_dc < 0)
                    m_dc = avg;
                m_dc += (avg - m_dc) * 0.002f;
                float v = m_dc - avg;                 // same polarity as the original code

                // 4) gentle high-pass (optional)
                if (ADC_MIC_HPF_HZ > 0.0f)
                    v = m_hpf[1].process(m_hpf[0].process(v));

                // 5) gain, start-up mute, clamp
                v *= ADC_MIC_GAIN;
                if (m_warmup > 0)
                {
                    m_warmup--;
                    v = 0;
                }
                if (v > 32767.0f)  v = 32767.0f;
                if (v < -32767.0f) v = -32767.0f;

                if (n_out < count)
                    samples[n_out++] = (int16_t)v;
            }
        }
    }
    return n_out;
}

#endif