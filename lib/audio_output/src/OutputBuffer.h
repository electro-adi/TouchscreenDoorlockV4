#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

// Output gain applied to the 8-bit samples (original project: 32). Raise for more volume.
#ifndef SPEAKER_GAIN
#define SPEAKER_GAIN 128
#endif

// After an UNDERRUN (the buffer ran dry) playback resumes once 1/OUTPUT_REBUFFER_DIVISOR of the normal
// start-up pre-buffer has refilled. 1 = original behaviour. (A simulation of bursty WiFi delivery showed
// no benefit from larger values, so the default is unchanged; the point of this file is the counters.)
#ifndef OUTPUT_REBUFFER_DIVISOR
#define OUTPUT_REBUFFER_DIVISOR 1
#endif

/**
 * @brief Circular buffer for 8 bit unsigned PCM samples
 *
 * Instrumented: underruns() / dropped() tell you whether choppy playback comes from the network
 * (packets arriving late = underruns) or from overflow (dropped).
 */
class OutputBuffer
{
private:
  // how many samples should we buffer before outputting data?
  int m_number_samples_to_buffer;
  // how many are needed right now to (re)start playing: full pre-buffer at the start, less after an underrun
  int m_threshold;
  int m_read_head;
  int m_write_head;
  int m_available_samples;
  int m_buffer_size;
  bool m_buffering;
  uint8_t *m_buffer;
  SemaphoreHandle_t m_semaphore;
  volatile uint32_t m_underruns = 0;
  volatile uint32_t m_dropped = 0;

public:
  OutputBuffer(int number_samples_to_buffer) : m_number_samples_to_buffer(number_samples_to_buffer)
  {
    m_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(m_semaphore);
    m_read_head = 0;
    m_write_head = 0;
    m_available_samples = 0;
    m_threshold = number_samples_to_buffer;
    m_buffering = true;
    m_buffer_size = 3 * number_samples_to_buffer;
    m_buffer = (uint8_t *)malloc(m_buffer_size);
    if (!m_buffer)
    {
      Serial.println("Failed to allocate buffer");
      m_buffer_size = 0;
      return;
    }
    memset(m_buffer, 0, m_buffer_size);
  }

  // we're adding samples that are 8 bit as they are coming from the transport
  void add_samples(const uint8_t *samples, int count)
  {
    xSemaphoreTake(m_semaphore, portMAX_DELAY);
    if (m_available_samples + count <= m_buffer_size)
    {
      for (int i = 0; i < count; i++)
      {
        m_buffer[m_write_head] = samples[i];
        m_write_head = (m_write_head + 1) % m_buffer_size;
      }
      m_available_samples += count;
    }
    else
    {
      m_dropped++;                       // no room: this packet is lost
    }
    xSemaphoreGive(m_semaphore);
  }

  // convert the samples to 16 bit as they are going to the output
  void remove_samples(int16_t *samples, int count)
  {
    xSemaphoreTake(m_semaphore, portMAX_DELAY);
    for (int i = 0; i < count; i++)
    {
      samples[i] = 0;
      // buffer ran dry while playing: start re-buffering (shorter than the initial pre-buffer)
      if (m_available_samples == 0 && !m_buffering)
      {
        m_buffering = true;
        m_underruns++;
        m_threshold = m_number_samples_to_buffer / OUTPUT_REBUFFER_DIVISOR;
        if (m_threshold < 1) m_threshold = 1;
      }
      if (m_buffering && m_available_samples < m_threshold)
      {
        samples[i] = 0;                  // not enough yet: silence
      }
      else
      {
        m_buffering = false;
        int32_t sample = m_buffer[m_read_head];
        int32_t v = (sample - 128) * SPEAKER_GAIN;
        samples[i] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
        m_read_head = (m_read_head + 1) % m_buffer_size;
        m_available_samples--;
      }
    }
    xSemaphoreGive(m_semaphore);
  }

  void flush()
  {
    // drop everything and start the next call with the full pre-buffer again
    xSemaphoreTake(m_semaphore, portMAX_DELAY);
    m_read_head = 0;
    m_write_head = 0;
    m_available_samples = 0;
    m_buffering = true;
    m_threshold = m_number_samples_to_buffer;
    xSemaphoreGive(m_semaphore);
  }

  // read-and-reset the counters (call from the intercom task when a receive session ends)
  uint32_t underruns() { uint32_t v = m_underruns; m_underruns = 0; return v; }
  uint32_t dropped()   { uint32_t v = m_dropped;   m_dropped = 0;   return v; }
};