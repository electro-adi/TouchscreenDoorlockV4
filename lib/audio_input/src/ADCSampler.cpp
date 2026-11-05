#include <Arduino.h>
#include <math.h>
#include "ADCSampler.h"

#if CONFIG_IDF_TARGET_ESP32

// remembered across transmissions (0 = nothing learned yet)
static float s_native_learned = 0.0f;

void ADCSampler::forgetLearnedRate() { s_native_learned = 0.0f; }

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
    m_cal_stage = 0;
    m_cal_t0 = 0;
    m_cal_n = 0;
    m_start_us = micros();

    // With a valid configuration the legacy driver delivers exactly the configured rate.
    // (If an earlier transmission measured something different, start from that.)
    m_native = (s_native_learned > 0) ? s_native_learned : (float)m_i2s_config.sample_rate;
    if ((float)m_i2s_config.sample_rate < ADC_MIN_I2S_RATE)
        Serial.printf("ADCSampler - WARNING: I2S ADC rate %u Hz is below the driver minimum (~%.0f Hz); "
                      "increase sample_rate in i2s_adc_config (ADC_RATE_DIVIDER)\n", (unsigned)m_i2s_config.sample_rate, ADC_MIN_I2S_RATE);

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

// Measure the real raw sample rate in two windows after start. Each window begins after the DMA backlog that
// is already waiting at the first read has been drained, so it is not counted.
//   window 1: 0.15 s .. 0.45 s  - coarse (+-3 %); only acts if the rate is more than 25 % off what we assume
//   window 2: 0.45 s .. 1.20 s  - precise (+-1 %); makes the FINAL decision: keep the configured rate if the
//                                 measurement agrees with it within 4 %, otherwise use (and remember) the measurement
void ADCSampler::calibrate(int n_in)
{
    if (m_cal_stage >= 4) return;
    uint32_t now = micros();
    uint32_t since = now - m_start_us;

    static const uint32_t begin_us[2] = {150000, 450000};
    static const uint32_t length_us[2] = {300000, 750000};
    int w = m_cal_stage / 2;                       // which window we are in

    if ((m_cal_stage & 1) == 0)                    // waiting for the window to open
    {
        if (since >= begin_us[w]) { m_cal_t0 = now; m_cal_n = 0; m_cal_stage++; }
        return;
    }
    m_cal_n += n_in;                               // measuring
    uint32_t dt = now - m_cal_t0;
    if (dt < length_us[w]) return;

    float measured = m_cal_n * 1e6f / (float)dt;
    float configured = (float)m_i2s_config.sample_rate;
    if (w == 0)
    {
        if (fabsf(measured - m_native) > 0.25f * m_native)
        {
            Serial.printf("ADCSampler - measured ADC rate %.0f Hz differs from the assumed %.0f Hz, using the measurement\n", measured, m_native);
            m_native = measured;                   // provisional
        }
    }
    else
    {
        if (fabsf(measured - configured) <= 0.04f * configured)
        {
            m_native = configured;                 // the driver does what we asked
            s_native_learned = 0.0f;
        }
        else if (measured > 0.2f * configured && measured < 4.0f * configured)
        {
            if (fabsf(measured - m_native) > 0.04f * m_native)
                Serial.printf("ADCSampler - ADC really runs at %.0f Hz (configured %.0f Hz), using %.0f Hz\n", measured, configured, measured);
            m_native = measured;
            s_native_learned = measured;           // start the next transmission with this value
        }
    }
    m_cal_stage++;
}

int ADCSampler::read(int16_t *samples, int count)
{
    const float binWidth = m_native / ADC_ACTUAL_RATE;         // raw samples per output sample

    // read just enough raw samples to produce at most 'count' outputs
    int want = (int)((count - 1) * binWidth);
    if (want > IN_BLOCK) want = IN_BLOCK;
    if (want < 16) want = 16;

    size_t bytes_read = 0;
    i2s_read(m_i2sPort, m_in, sizeof(int16_t) * want, &bytes_read, portMAX_DELAY);
    int n_in = bytes_read / sizeof(int16_t);
    calibrate(n_in);

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

    int n_out = 0;
    for (int i = 0; i < n_in; i++)
    {
        int raw = uint16_t(m_in[i]) & 0xfff;

        // 1) glitch rejection: 0 and 4095 are the ADC rails, hold the last good value
        if (raw == 0 || raw == 4095)
            raw = m_lastValid;
        else
            m_lastValid = raw;

        // 2) resample (box average = cheap anti-alias filter)
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