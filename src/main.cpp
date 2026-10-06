
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*





                                                  
                                                  Adi's Doorlock





*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <TFT_eSPI.h>
#include <SD.h>
#include <Preferences.h>
#include <vector>
#include <math.h>

#include <Adafruit_NeoPixel.h>
#include <SparkFunSX1509.h>
#include "Keypad_SX1509.h"

#include "DFRobot_VL53L0X.h"
#include <Thinary_AHT10.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoHA.h>

#include <driver/i2s.h>
#include <driver/gpio.h>
#include "ADCSampler.h"
#include "I2SOutput.h"
#include "UdpTransport.h"
#include "OutputBuffer.h"

#include "credentials.h"
#include "definitions.h"
#include "AdiWiFiManager.h"

// I2S Config for using the Internal ADC
i2s_config_t i2s_adc_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
  .sample_rate = (uint32_t)(SAMPLE_RATE / ADC_RATE_DIVIDER),
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_STAND_MSB,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 8,
  .dma_buf_len = 256,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0
};

// I2S Speaker Pins
i2s_pin_config_t i2s_speaker_pins = {
  .bck_io_num = I2S_BCLK,
  .ws_io_num = I2S_LRC,
  .data_out_num = I2S_DOUT,
  .data_in_num = I2S_PIN_NO_CHANGE
};

uint32_t miredsToColor(uint16_t mireds);
bool matrixAlertBusy();

void LIDAR_Handler();
void PIR_Handler(); 
void AHT10_Handler();
void Reed_Handler();

void GetEEPROM();
void UpdateEEPROM();
void wifiDebugLog(const char *message);
void printLog(String log);
void setup();
void loop();

void HA_MQTT_Init();
void onMqttDisconnected();
void HAButtonsHandler(HAButton* sender);
void HASwitchHandler(bool state, HASwitch* sender);

uint16_t matrixXY(uint8_t x, uint8_t y);
void matrixClear();
uint8_t matrixMaxRing();
uint8_t matrixStepCount(MatrixEffectType type);
void matrixAdvanceStep();
bool matrixHasGlow();
void matrixRenderFrame();
void matrixStartEffect(MatrixEffectType type, uint16_t hue, uint16_t speedMs);
void matrixStopEffect();
void matrixUpdate();

void DoorlockRGB_State(bool state, HALight* sender);
void DoorlockRGB_Brightness(uint8_t brightness, HALight* sender);
void DoorlockRGB_Temp(uint16_t temperature, HALight* sender);
void DoorlockRGB_Color(HALight::RGBColor color, HALight* sender);
void DoorlockRGB_Effect(int8_t index, HASelect* sender);
void MatrixRGB_State(bool state, HALight* sender);
void MatrixRGB_Brightness(uint8_t brightness, HALight* sender);
void MatrixRGB_Temp(uint16_t temperature, HALight* sender);
void MatrixRGB_Color(HALight::RGBColor color, HALight* sender);
void MatrixRGB_Effect(int8_t index, HASelect* sender);

void Gate_Opened_Alert(bool state, HASwitch* sender);
void LStair_Presence_Alert(bool state, HASwitch* sender);
void LStair_Cbell_Alert(bool state, HASwitch* sender);
void UStair_Presence_Alert(bool state, HASwitch* sender);
void UStair_Lidar_Alert(bool state, HASwitch* sender);
void Hall_PIR1_Alert(bool state, HASwitch* sender);
void Hall_PIR2_Alert(bool state);
void Door_Lidar_Alert(bool state);
void LockHandler();

void keyEventListener(KeypadEvent key, KeyState kpadState);

void Intercom_Init();
void Intercom_Task(void *param);

Preferences preferences;
AdiWiFiManager WiFiManager;
TFT_eSPI tft = TFT_eSPI();
SPIClass sdSPI(VSPI);
Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);
AHT10Class AHT10;
SX1509 ExtraIO;
DFRobot_VL53L0X LIDAR;

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);

Keypad_SX1509 keypad(makeKeymap(keyMap), rowPins, colPins, KEY_ROWS, KEY_COLS, ExtraIO);

Output *m_output;
I2SSampler *m_input;
Transport *m_transport;
OutputBuffer *m_output_buffer;

//------------------------------------------- Home Assistant Stuff

HAButton DevModeCMD("dev_mode_btn");
HAButton RebootCMD("reboot_btn");
HAButton UnlockDoorCMD("unlock_door_btn");
HAButton StopSoundsCMD("stop_sounds");

HABinarySensor PIR1("hallway_presence");
HABinarySensor PIR2("adi_room_presence");
HABinarySensor Reed("adi_room_door_status");
HABinarySensor Lidar("doorlock_lidar_presence");
HABinarySensor Button("button");

HASensorNumber Temp("hall_temp");
HASensorNumber Humd("hall_humd");

HASwitch gate_opened_alert("gate_opened_alert");
HASwitch lstair_pres_alert("lstair_pres_alert");
HASwitch lstair_cbell_alert("lstair_cbell_alert");
HASwitch ustair_pres_alert("ustair_pres_alert");
HASwitch ustair_lidar_alert("ustair_lidar_alert");
HASwitch hall_pir1_alert("hall_pir1_alert");

HASwitch Buzzer1CMD("buzzer1");
HASwitch Buzzer2CMD("buzzer2");
HASwitch MuteCMD("mute");
HASwitch DisMAlertsCMD("disable_m_alerts");
HASwitch DisDAlertsCMD("disable_d_alerts");

HALight DoorlockRGBCMD("doorlock_rgb", HALight::BrightnessFeature | HALight::ColorTemperatureFeature | HALight::RGBFeature);
HALight MatrixRGBCMD("matrix_rgb", HALight::BrightnessFeature | HALight::ColorTemperatureFeature | HALight::RGBFeature);
HASelect DoorlockRGB_Eff("doorlock_rgb_eff");
HASelect MatrixRGB_Eff("matrix_rgb_eff");

// ========================================================================================================================================
// ======================= RGB Effects Engine (Doorlock strip + Matrix) ========================
// ========================================================================================================================================

class StripEffects {
  public:
    void begin(const int* leds, uint8_t count) 
    {
      _leds = leds;
      _count = count;
    }

    void begin2D(const int* leds, uint8_t count, uint8_t w, uint8_t h, uint16_t (*xyFunc)(uint8_t, uint8_t)) 
    {
      begin(leds, count);
      _width = w;
      _height = h;
      _xyFunc = xyFunc;
    }

    void setPower(bool on) 
    {
      _on = on;
      if(!on) 
      {
        for(uint8_t i = 0; i < _count; i++) pixels.setPixelColor(_leds[i], 0);
        pixels.show();
      } 
      else if(_effect == 0) renderSolid();
    }

    void setBrightness(uint8_t percent) 
    {
      _brightnessPct = percent;
      if(_on && _effect == 0) renderSolid();
    }

    void setColor(uint32_t rgb) 
    {
      _color = rgb;
      if(_on && _effect == 0) renderSolid();
    }

    void setEffect(uint8_t idx) 
    {
      _effect = idx;
      _step = 0;
      _dir = 1;
      for(uint8_t i = 0; i < _count; i++) _glow[i] = 0;
      _lastStep = millis();
      _lastFrame = 0;
      if(_on && _effect == 0) renderSolid();
    }

    void update() 
    {
      if(!_on) return;
      if(_effect == 0) return;

      unsigned long now = millis();
      if(now - _lastFrame < FRAME_MS) return;
      _lastFrame = now;

      switch(_effect) 
      {
        case 1:  effectBreathe(); break;
        case 2:  effectRainbow(); break;
        case 3:  effectChase(); break;
        case 4:  effectTheater(); break;
        case 5:  effectComet(); break;
        case 6:  effectTwinkle(); break;
        case 7:  effectSparkle(); break;
        case 8:  effectWipe(); break;
        case 9:  effectBounce(); break;
        case 10: effectWheel(); break;
        case 11: effectFire(); break;
        case 12: effectStrobe(); break;
        case 13: effectWave(); break;
        case 14: if(_xyFunc) effectCircularSpatial(); break;
        case 15: if(_xyFunc) effectTopDownSpatial(); break;
        case 16: if(_xyFunc) effectLeftRightSpatial(); break;
        default: break;
      }
      pixels.show();
    }

  private:
    static const uint16_t FRAME_MS = 25;

    const int* _leds = nullptr;
    uint8_t _count = 0;
    uint8_t _width = 0, _height = 0;
    uint16_t (*_xyFunc)(uint8_t, uint8_t) = nullptr;

    bool _on = false;
    uint8_t _brightnessPct = 100;
    uint32_t _color = 0xFFFFFF;
    uint8_t _effect = 0;

    uint16_t _step = 0;
    int8_t _dir = 1;
    unsigned long _lastStep = 0;
    unsigned long _lastFrame = 0;
    float _glow[26] = {0};

    uint32_t applyBrightness(uint32_t rgb, float factor) 
    {
      if(factor < 0) factor = 0;
      if(factor > 1) factor = 1;
      uint8_t r = (uint8_t)(((rgb >> 16) & 0xFF) * factor);
      uint8_t g = (uint8_t)(((rgb >> 8) & 0xFF) * factor);
      uint8_t b = (uint8_t)((rgb & 0xFF) * factor);
      return pixels.Color(r, g, b);
    }

    void renderSolid() 
    {
      uint32_t c = applyBrightness(_color, _brightnessPct / 100.0f);
      for(uint8_t i = 0; i < _count; i++) pixels.setPixelColor(_leds[i], c);
      pixels.show();
    }

    bool advanceStepMod(uint16_t intervalMs, uint16_t mod) 
    {
      if(millis() - _lastStep >= intervalMs) 
      {
        _lastStep = millis();
        _step = (_step + 1) % mod;
        return true;
      }
      return false;
    }

    uint8_t spatialMaxRing() 
    {
      uint8_t cx = (_width - 1) / 2;
      uint8_t cy = (_height - 1) / 2;
      uint8_t rx = max(cx, (uint8_t)(_width - 1 - cx));
      uint8_t ry = max(cy, (uint8_t)(_height - 1 - cy));
      return max(rx, ry);
    }

    void decayAndRenderSpatial() 
    {
      for(uint8_t y = 0; y < _height; y++) 
      {
        for(uint8_t x = 0; x < _width; x++) 
        {
          uint8_t idx = y * _width + x;
          float b = _glow[idx];
          if(b > 0) 
          {
            b *= 0.80f;
            if(b < 0.004f) b = 0;
            _glow[idx] = b;
          }
          pixels.setPixelColor(_xyFunc(x, y), applyBrightness(_color, b * (_brightnessPct / 100.0f)));
        }
      }
    }

    void effectBreathe() 
    {
      float phase = (millis() % 4000) / 4000.0f;
      float s = (sinf(phase * 2 * PI) + 1.0f) / 2.0f;
      float factor = (0.15f + 0.85f * s) * (_brightnessPct / 100.0f);
      uint32_t c = applyBrightness(_color, factor);
      for(uint8_t i = 0; i < _count; i++) pixels.setPixelColor(_leds[i], c);
    }

    void effectRainbow() 
    {
      unsigned long t = millis();
      uint8_t val = (uint8_t)(255 * (_brightnessPct / 100.0f));
      for(uint8_t i = 0; i < _count; i++) 
      {
        uint16_t hue = (uint16_t)(((uint32_t)i * 65536UL / _count) + t * 20);
        pixels.setPixelColor(_leds[i], pixels.gamma32(pixels.ColorHSV(hue, 255, val)));
      }
    }

    void effectChase() 
    {
      advanceStepMod(80, _count);
      uint32_t c = applyBrightness(_color, _brightnessPct / 100.0f);
      for(uint8_t i = 0; i < _count; i++) pixels.setPixelColor(_leds[i], (i == _step) ? c : 0);
    }

    void effectTheater() 
    {
      advanceStepMod(150, 3);
      uint32_t c = applyBrightness(_color, _brightnessPct / 100.0f);
      for(uint8_t i = 0; i < _count; i++) pixels.setPixelColor(_leds[i], ((i % 3) == _step) ? c : 0);
    }

    void effectComet() 
    {
      advanceStepMod(60, _count);
      _glow[_step] = 1.0f;
      for(uint8_t i = 0; i < _count; i++) 
      {
        if(_glow[i] > 0) { _glow[i] *= 0.80f; if(_glow[i] < 0.01f) _glow[i] = 0; }
        pixels.setPixelColor(_leds[i], applyBrightness(_color, _glow[i] * (_brightnessPct / 100.0f)));
      }
    }

    void effectTwinkle() {
      if(millis() - _lastStep > 90) 
      {
        _lastStep = millis();
        _glow[random(_count)] = 1.0f;
      }
      for(uint8_t i = 0; i < _count; i++) 
      {
        if(_glow[i] > 0) { _glow[i] *= 0.90f; if(_glow[i] < 0.02f) _glow[i] = 0; }
        pixels.setPixelColor(_leds[i], applyBrightness(_color, _glow[i] * (_brightnessPct / 100.0f)));
      }
    }

    void effectSparkle() {
      if(millis() - _lastStep > 50) 
      {
        _lastStep = millis();
        if(random(100) < 40) _glow[random(_count)] = 1.0f;
      }
      for(uint8_t i = 0; i < _count; i++) 
      {
        if(_glow[i] > 0) { _glow[i] *= 0.65f; if(_glow[i] < 0.03f) _glow[i] = 0; }
        pixels.setPixelColor(_leds[i], applyBrightness(_color, _glow[i] * (_brightnessPct / 100.0f)));
      }
    }

    void effectWipe() 
    {
      advanceStepMod(80, _count * 2);
      uint32_t c = applyBrightness(_color, _brightnessPct / 100.0f);
      bool filling = _step < _count;
      int litUpTo = filling ? _step : (2 * (int)_count - 1 - _step);
      for(uint8_t i = 0; i < _count; i++) pixels.setPixelColor(_leds[i], ((int)i <= litUpTo) ? c : 0);
    }

    void effectBounce() 
    {
      if(millis() - _lastStep > 60) 
      {
        _lastStep = millis();
        _step += _dir;
        if(_step >= _count - 1) { _step = _count - 1; _dir = -1; }
        if(_step <= 0) { _step = 0; _dir = 1; }
      }
      uint32_t c = applyBrightness(_color, _brightnessPct / 100.0f);
      for(uint8_t i = 0; i < _count; i++) pixels.setPixelColor(_leds[i], (i == _step) ? c : 0);
    }

    void effectWheel() 
    {
      uint16_t hue = (uint16_t)((millis() / 20) % 65536);
      uint32_t c = pixels.gamma32(pixels.ColorHSV(hue, 255, (uint8_t)(255 * (_brightnessPct / 100.0f))));
      for(uint8_t i = 0; i < _count; i++) pixels.setPixelColor(_leds[i], c);
    }

    void effectFire() 
    {
      if(millis() - _lastStep > 60) 
      {
        _lastStep = millis();
        for(uint8_t i = 0; i < _count; i++) 
        {
          int cooldown = random(0, 80);
          int heat255 = (int)(_glow[i] * 255) - cooldown;
          _glow[i] = (heat255 < 0 ? 0 : heat255) / 255.0f;
        }
        if(random(100) < 60) 
        {
          uint8_t idx = random(_count);
          float spark = random(60, 100) / 100.0f;
          _glow[idx] = min(1.0f, _glow[idx] + spark);
        }
      }
      for(uint8_t i = 0; i < _count; i++) 
      {
        float heat = _glow[i];
        uint16_t hue = (uint16_t)(1500 + heat * 3500); // deep red -> orange/yellow
        uint8_t val = (uint8_t)(heat * 255 * (_brightnessPct / 100.0f));
        pixels.setPixelColor(_leds[i], val == 0 ? 0 : pixels.gamma32(pixels.ColorHSV(hue, 255, val)));
      }
    }

    void effectStrobe() 
    {
      bool on = (millis() % 300) < 40;
      uint32_t c = on ? applyBrightness(_color, _brightnessPct / 100.0f) : 0;
      for(uint8_t i = 0; i < _count; i++) pixels.setPixelColor(_leds[i], c);
    }

    void effectWave() 
    {
      float t = millis() / 1000.0f;
      for(uint8_t i = 0; i < _count; i++) 
      {
        float phase = (float)i / _count * 2 * PI + t * 2.5f;
        float s = (sinf(phase) + 1.0f) / 2.0f;
        pixels.setPixelColor(_leds[i], applyBrightness(_color, s * (_brightnessPct / 100.0f)));
      }
    }

    void effectCircularSpatial() 
    {
      uint8_t maxRing = spatialMaxRing();
      if(advanceStepMod(220, maxRing + 1)) 
      {
        uint8_t cx = (_width - 1) / 2, cy = (_height - 1) / 2;
        for(uint8_t y = 0; y < _height; y++) 
        {
          for(uint8_t x = 0; x < _width; x++)
          {
            uint8_t dist = max(abs((int)x - (int)cx), abs((int)y - (int)cy));
            if(dist == _step) _glow[y * _width + x] = 1.0f;
          }
        }
      }
      decayAndRenderSpatial();
    }

    void effectTopDownSpatial() 
    {
      if(advanceStepMod(200, _height))
      {
        uint8_t y = _height - 1 - _step;
        for(uint8_t x = 0; x < _width; x++) _glow[y * _width + x] = 1.0f;
      }
      decayAndRenderSpatial();
    }

    void effectLeftRightSpatial() 
    {
      if(advanceStepMod(200, _width)) 
      {
        uint8_t x = _step;
        for(uint8_t y = 0; y < _height; y++) _glow[y * _width + x] = 1.0f;
      }
      decayAndRenderSpatial();
    }
};

StripEffects doorlockFX;
StripEffects matrixFX;

uint32_t miredsToColor(uint16_t mireds) {

  float kelvin = (1000000.0f / mireds) / 100.0f;
  float r, g, b;

  if(kelvin <= 66) 
  {
    r = 255;
    g = 99.47f * logf(kelvin) - 161.12f;
  } 
  else 
  {
    r = 329.7f * powf(kelvin - 60, -0.1332f);
    g = 288.12f * powf(kelvin - 60, -0.0755f);
  }

  if(kelvin >= 66) b = 255;
  else if(kelvin <= 19) b = 0;
  else b = 138.52f * logf(kelvin - 10) - 305.04f;

  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);

  return ((uint32_t)(uint8_t)r << 16) | ((uint32_t)(uint8_t)g << 8) | (uint8_t)b;
}

bool matrixAlertBusy() {
  return (matrixActiveEffect != EFFECT_NONE) || matrixHasGlow();
}

//========================================================================================================================================
//======================= I2C + Sensors Reading Functions ========================
//========================================================================================================================================

int lidar_median_read()
{
  int samples[LIDAR_SAMPLES];
  int valid = 0;

  for (int i = 0; i < LIDAR_SAMPLES; i++) {
    int d = LIDAR.getDistance();
    if (d > 0 && d < 8000) samples[valid++] = d;
    delay(10); // small gap between samples — keeps I2C happy
  }

  if (valid == 0) return -1;
  if (valid == 1) return samples[0];

  // Simple sort for small N
  for (int i = 0; i < valid - 1; i++)
    for (int j = i + 1; j < valid; j++)
      if (samples[j] < samples[i]) { int t = samples[i]; samples[i] = samples[j]; samples[j] = t; }

  return samples[valid / 2];
}

void LIDAR_Handler()
{
  if(millis() - last_lidar_read < lidar_read_interval) return;
  last_lidar_read = millis();

  int dist = lidar_median_read();
  if(dist < 0) return;

  // Bootstrap
  if(lidar_baseline < 0) 
  {
    lidar_baseline     = dist;
    lidar_last_reading = dist;
    DEBUG_PRINT("LIDAR_Handler - Baseline initialised: ");
    DEBUG_PRINT(lidar_baseline);
    DEBUG_PRINTLN("mm");
    return;
  }

  int delta_from_baseline = abs(dist - lidar_baseline);
  bool reading_is_normal  = (delta_from_baseline < LIDAR_BASELINE_MARGIN);

  // ── Only update baseline when stable AND no presence active
  // Prevents the baseline from chasing the environment during an event
  if(reading_is_normal && !LIDAR_presence) 
  {
    lidar_baseline = (int)(LIDAR_BASELINE_ALPHA * dist + (1.0f - LIDAR_BASELINE_ALPHA) * lidar_baseline);
  }

  bool suspicious = (delta_from_baseline > LIDAR_CHANGE_THRESHOLD);

  if(suspicious) 
  {
    lidar_last_trigger_ms = millis();
    if(!LIDAR_presence) 
    {
      LIDAR_presence = true;
      Lidar.setState(true);
      DEBUG_PRINTLN("LIDAR_Handler - Presence detected.");
    }
  } 
  else 
  {
    if(LIDAR_presence && (millis() - lidar_last_trigger_ms > LIDAR_PRESENCE_HOLD_MS)) {
      LIDAR_presence = false;
      Lidar.setState(false);
      DEBUG_PRINTLN("LIDAR_Handler - Presence cleared.");
    }
  }

  lidar_last_reading = dist;

  #if SERIAL_DEBUG != 0
    //Serial.printf("LIDAR_Handler - dist=%d mm  baseline=%d mm  delta=%d  present=%s\n", dist, lidar_baseline, delta_from_baseline, LIDAR_presence ? "YES" : "no");
  #endif
}

void PIR_Handler() {

  //PIR Sensor 1, Facing the hall

  if(millis() - last_pir_read1 < pir_read_interval) return;
  last_pir_read1 = millis();

  bool hall_motion = ExtraIO.digitalRead(SX1509_PIR1_PIN);

  if(hall_motion) 
  {
    pir_last_trigger_ms1 = millis();   // extend hold window
    if(!Hall_presence) 
    {
      Hall_presence = true;
      PIR1.setState(true);
      DEBUG_PRINTLN("PIR_Handler - Hall Presence detected");
    }
  } 
  else 
  {
    // Only clear after hold period with no new triggers
    if(Hall_presence && (millis() - pir_last_trigger_ms1 > PIR_PRESENCE_HOLD_MS)) 
    {
      Hall_presence = false;
      PIR1.setState(false);
      DEBUG_PRINTLN("PIR_Handler - Hall Presence cleared.");
    }
  }

  //PIR Sensor 2, Facing the room

  if(millis() - last_pir_read2 < pir_read_interval) return;
  last_pir_read2 = millis();

  bool room_motion = ExtraIO.digitalRead(SX1509_PIR2_PIN);

  if(room_motion) 
  {
    pir_last_trigger_ms2 = millis();   // extend hold window
    if(!Room_presence) 
    {
      Room_presence = true;
      PIR2.setState(true);
      DEBUG_PRINTLN("PIR_Handler - Room Presence detected");
    }
  } 
  else 
  {
    // Only clear after hold period with no new triggers
    if(Room_presence && (millis() - pir_last_trigger_ms2 > PIR_PRESENCE_HOLD_MS)) 
    {
      Room_presence = false;
      PIR2.setState(false);
      DEBUG_PRINTLN("PIR_Handler - Room Presence cleared.");
    }
  }
}

void AHT10_Handler() {

  if(millis() - last_aht10_read < aht10_read_interval) return;
  last_aht10_read = millis();

  float temperature = AHT10.GetTemperature();
  float humidity = AHT10.GetHumidity();

  Temp.setValue(temperature);
  Humd.setValue(humidity);
}

void Reed_Handler() {

  if(millis() - last_reed_read < reed_read_interval) return;
  last_reed_read = millis();

  if(ExtraIO.digitalRead(SX1509_REED_PIN) && !Door_open)
  {
    Door_open = true;
    Reed.setState(true);
    DEBUG_PRINTLN("Reed_Handler - Door Opened.");
  }
  else if(!ExtraIO.digitalRead(SX1509_REED_PIN) && Door_open)
  {
    Door_open = false;
    Reed.setState(false);
    DEBUG_PRINTLN("Reed_Handler - Door Closed.");
  }
}

void Button_Handler() {

  bool reading = !ExtraIO.digitalRead(SX1509_BUTTON_PIN);

  if(reading != last_btn_state) last_btn_press_time = millis();
  
  if((millis() - last_btn_press_time) > 50) 
  {
    if(reading != cur_btn_state) 
    {
      cur_btn_state = reading;
      if(cur_btn_state == HIGH) 
      {
        DEBUG_PRINTLN("Button_Handler - BTN PRESS!");
        UnlockDoor_Now = true;
      }
    }
  }

  last_btn_state = reading;
}

//========================================================================================================================================
//======================= Loops, Buttons and EEPROM Functions ========================
//========================================================================================================================================

void GetEEPROM() {

  preferences.begin("variables", false);
  reboot_devmode = preferences.getBool("reboot_devmode", reboot_devmode);
  preferences.end();
}

void UpdateEEPROM() {

  preferences.begin("variables", false);
  preferences.putBool("reboot_devmode", reboot_devmode); 
  preferences.end();
}

void wifiDebugLog(const char *message) {
  DEBUG_PRINT("WiFiDebugLog - ");
  DEBUG_PRINTLN(message);
  if(!setup_complete || dev_mode) printLog(message);
}

std::vector<String> wrapLog(String text, int maxWidth) {
  std::vector<String> lines;
  String current = "";

  int start = 0;
  while (start < (int)text.length()) 
  {
    int spaceIdx = text.indexOf(' ', start);
    String word = (spaceIdx == -1) ? text.substring(start) : text.substring(start, spaceIdx);
    start = (spaceIdx == -1) ? text.length() : spaceIdx + 1;

    String test = current.length() ? current + " " + word : word;

    if(tft.textWidth(test) <= maxWidth) 
    {
      current = test;
      continue;
    }

    if(current.length()) 
    {
      lines.push_back(current);
      current = "";
    }

    while (tft.textWidth(word) > maxWidth) 
    {
      int cut = word.length();
      while (cut > 1 && tft.textWidth(word.substring(0, cut)) > maxWidth) cut--;
      lines.push_back(word.substring(0, cut));
      word = word.substring(cut);
    }
    current = word;
  }

  if(current.length()) lines.push_back(current);
  return lines;
}

void printLog(String log) {
  log = "> " + log;

  int maxWidth = tft.width() - LOG_LEFT_MARGIN - LOG_RIGHT_MARGIN;
  std::vector<String> wrapped = wrapLog(log, maxWidth);
  for(String &line : wrapped) logLines.push_back(line);

  int maxLines = (tft.height() - LOG_TOP_MARGIN) / LOG_LINE_HEIGHT;
  while((int)logLines.size() > maxLines) logLines.erase(logLines.begin());

  tft.fillScreen(TFT_BLACK);
  for (size_t i = 0; i < logLines.size(); i++) tft.drawString(logLines[i], LOG_LEFT_MARGIN, LOG_TOP_MARGIN + (LOG_LINE_HEIGHT * i));
}

void setup() {

  #if SERIAL_DEBUG != 0
    Serial.begin(115200);
  #endif

  pinMode(DevBTN, INPUT);

  GetEEPROM();

  DEBUG_PRINTLN("setup - Build:" + String(BUILD));

  tft.init();
  tft.setRotation(0);
  tft.setTextSize(1);
  tft.fillScreen(TFT_BLACK);
  printLog("Build:" + String(BUILD));
  printLog("Hold dev button to enter dev mode");
  delay(1000);

  if(reboot_devmode) 
  {
    dev_mode = true;
    reboot_devmode = false;
    UpdateEEPROM();
  }

  if(digitalRead(DevBTN))
  {
    DEBUG_PRINTLN("setup - Booting into Dev Mode");
    printLog("Booting into Dev Mode");
    dev_mode = true;
  }

  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if(!SD.begin(SD_CS, sdSPI)) 
  {
    DEBUG_PRINTLN("setup - SD Card Mount Failed");
    printLog("SD Card Mount Failed!");
    sd_available = false;
  }
  else
  {
    DEBUG_PRINTLN("setup - SD Card Mount Successful");
    printLog("SD Card Mount Successful");
  }

  WiFiManager.setDebugCallback(wifiDebugLog);
  WiFiManager.WB_StaysActive(false);
  WiFiManager.connectToWiFi(true, "", "");

  if(dev_mode) WiFiManager.StartWebserver();
  
  if(!dev_mode && WiFiManager.getWiFiStatus() == WL_CONNECTED)
  {
    HA_MQTT_Init();

    Wire.end();
    delay(50);
    Wire.setTimeOut(10);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);

    delay(200);

    if(!ExtraIO.begin(SX1509_ADDRESS))
    {
      DEBUG_PRINTLN("setup - SX1509 Init Failed!");
      printLog("SX1509 Init Failed!");

      DEBUG_PRINTLN("setup - CRITICAL ERROR!");
      printLog("CRITICAL ERROR!");

      while(true)
      {
        //do nothing
      }
    }
    else
    {
      DEBUG_PRINTLN("setup - SX1509 Init Successful");
      printLog("SX1509 Init Successful");

      ExtraIO.pinMode(SX1509_PIR1_PIN, INPUT);
      ExtraIO.pinMode(SX1509_BUZZER1_PIN, OUTPUT);
      ExtraIO.pinMode(SX1509_BUZZER2_PIN, OUTPUT);
      ExtraIO.pinMode(SX1509_LOCK_PIN, OUTPUT, HIGH);
      ExtraIO.pinMode(SX1509_BUTTON_PIN, INPUT_PULLUP);
      ExtraIO.pinMode(SX1509_REED_PIN, INPUT_PULLUP);
      ExtraIO.pinMode(SX1509_PIR2_PIN, INPUT);

      ExtraIO.digitalWrite(SX1509_LOCK_PIN, HIGH);

      keypad.addStatedEventListener(keyEventListener);
    }

    delay(200);

    LIDAR.begin(0x50);
    LIDAR.setMode(LIDAR.eContinuous,LIDAR.eHigh);
    LIDAR.start();

    delay(200);

    if(!AHT10.begin(eAHT10Address_default)) 
    {
      DEBUG_PRINTLN("setup - AHT Init Failed!");
      printLog("AHT Init Failed!");
    }
    else
    {
      DEBUG_PRINTLN("setup - AHT Init Successful");
      printLog("AHT Init Successful");
    }

    pixels.begin();
    pixels.clear();
    for(int i = 0; i < DOORLOCKLEDS_NUM; i++) 
    {
      pixels.setPixelColor(Doorlock_LEDS[i], pixels.Color(0, 0, 100));
      pixels.show();
      delay(100);
    }
    for(int i = 0; i < MATRIX_LEDS_NUM; i++) 
    {
      pixels.setPixelColor(Matrix_LEDS[i], pixels.Color(100, 0, 100));
      pixels.show();
      delay(100);
    }
    delay(2000);
    pixels.clear();
    pixels.show();

    matrixClear();

    doorlockFX.begin(Doorlock_LEDS, DOORLOCKLEDS_NUM);
    matrixFX.begin2D(Matrix_LEDS, MATRIX_LEDS_NUM, MATRIX_W, MATRIX_H, matrixXY);

    DEBUG_PRINTLN("setup - Initializing Intercom..");
    printLog("Initializing Intercom..");
    Intercom_Init();

    DEBUG_PRINTLN("setup - Device Setup Complete");
    DEBUG_PRINTLN("");
    printLog("Device Setup Complete");

    delay(1000);
    
    tft.fillScreen(TFT_BLACK);
  }
  setup_complete = true;
}

void loop() {
  if(!dev_mode && WiFiManager.getWiFiStatus() == WL_CONNECTED)
  {
    mqtt.loop();

    LIDAR_Handler();
    PIR_Handler(); 
    AHT10_Handler();
    Reed_Handler();
    Button_Handler();
    LockHandler();

    keypad.getKeys();

    doorlockFX.update();
    if(!matrixAlertBusy()) matrixFX.update();
    matrixUpdate();
  }
  else
  {
    WiFiManager.loop();
  }
}

//========================================================================================================================================
//======================= Home Assistant Integration Functions ========================
//========================================================================================================================================

void HA_MQTT_Init() {

  byte mac[MAC_ADDR_LENGTH];
  WiFi.macAddress(mac);
  device.setUniqueId(mac, sizeof(mac));

  device.enableSharedAvailability();
  device.setAvailability(false);
  device.enableLastWill();

  static String configUrl = "http://" + WiFi.localIP().toString();

  device.setName("Adi's Doorlock");
  device.setManufacturer("Adi's Lab Inc");
  device.setModel("3000 PRO MAX");
  device.setSoftwareVersion(BUILD);
  device.setConfigurationUrl(configUrl.c_str());

  //----------------------------Buttons

  DevModeCMD.setName("Reboot into Dev Mode");
  DevModeCMD.setIcon("mdi:microsoft-visual-studio-code");
  DevModeCMD.onCommand(HAButtonsHandler);

  RebootCMD.setName("Reboot");
  RebootCMD.setIcon("mdi:restart");
  RebootCMD.onCommand(HAButtonsHandler);

  UnlockDoorCMD.setName("Unlock Door");
  UnlockDoorCMD.setIcon("mdi:lock-open-variant");
  UnlockDoorCMD.onCommand(HAButtonsHandler);

  StopSoundsCMD.setName("Stop Sounds");
  StopSoundsCMD.setIcon("mdi:volume-low");
  StopSoundsCMD.onCommand(HAButtonsHandler);

  //----------------------------Sensors

  PIR1.setName("Hallway Presence");
  PIR1.setDeviceClass("motion");
  PIR1.setIcon("mdi:walk");

  PIR2.setName("Adi's Room Presence");
  PIR2.setDeviceClass("motion");
  PIR2.setIcon("mdi:walk");

  Reed.setName("Door Status");
  Reed.setDeviceClass("opening");
  Reed.setIcon("mdi:door");

  Lidar.setName("LIDAR Presence");
  Lidar.setDeviceClass("motion");
  Lidar.setIcon("mdi:location-enter");

  Temp.setName("Hall Temperature");
  Temp.setIcon("mdi:temperature-celsius");
  Temp.setUnitOfMeasurement("°C");

  Button.setName("Unlock Button");
  Button.setIcon("mdi:button-pointer");

  Humd.setName("Hall Humidity");
  Humd.setIcon("mdi:water-percent");
  Humd.setUnitOfMeasurement("%");

  //----------------------------Alert Buttons

  gate_opened_alert.setName("Gate Opened Alert");
  gate_opened_alert.setIcon("mdi:alert-box");
  gate_opened_alert.onCommand(Gate_Opened_Alert);

  lstair_pres_alert.setName("Lower Stair Presence Alert");
  lstair_pres_alert.setIcon("mdi:alert-box");
  lstair_pres_alert.onCommand(LStair_Presence_Alert);

  lstair_cbell_alert.setName("Lower Stair Calling Bell Alert");
  lstair_cbell_alert.setIcon("mdi:alert-box");
  lstair_cbell_alert.onCommand(LStair_Cbell_Alert);

  ustair_pres_alert.setName("Upper Stair Presence Alert");
  ustair_pres_alert.setIcon("mdi:alert-box");
  ustair_pres_alert.onCommand(UStair_Presence_Alert);

  ustair_lidar_alert.setName("Upper Stair LIDAR Alert");
  ustair_lidar_alert.setIcon("mdi:alert-box");
  ustair_lidar_alert.onCommand(UStair_Lidar_Alert);

  hall_pir1_alert.setName("Hallway PIR Alert");
  hall_pir1_alert.setIcon("mdi:alert-box");
  hall_pir1_alert.onCommand(Hall_PIR1_Alert);

  //----------------------------Switches

  Buzzer1CMD.setName("Buzzer 1");
  Buzzer1CMD.setIcon("mdi:bell");
  Buzzer1CMD.onCommand(HASwitchHandler);

  Buzzer2CMD.setName("Buzzer 2");
  Buzzer2CMD.setIcon("mdi:bell");
  Buzzer2CMD.onCommand(HASwitchHandler);

  MuteCMD.setName("Mute");
  MuteCMD.setIcon("mdi:volume-mute");
  MuteCMD.onCommand(HASwitchHandler);

  DisMAlertsCMD.setName("Disable Matrix Alerts");
  DisMAlertsCMD.setIcon("mdi:blur-off");
  DisMAlertsCMD.onCommand(HASwitchHandler);

  DisDAlertsCMD.setName("Disable Doorlock Alerts");
  DisDAlertsCMD.setIcon("mdi:square-off-outline");
  DisDAlertsCMD.onCommand(HASwitchHandler);

  //----------------------------Lights

  DoorlockRGBCMD.setName("Doorlock RGB Strip");
  DoorlockRGBCMD.setBrightnessScale(100);
  DoorlockRGBCMD.setMinMireds(153);
  DoorlockRGBCMD.setMaxMireds(500);
  DoorlockRGBCMD.onStateCommand(DoorlockRGB_State);
  DoorlockRGBCMD.onBrightnessCommand(DoorlockRGB_Brightness);
  DoorlockRGBCMD.onColorTemperatureCommand(DoorlockRGB_Temp);
  DoorlockRGBCMD.onRGBColorCommand(DoorlockRGB_Color);

  MatrixRGBCMD.setName("Matrix RGB");
  MatrixRGBCMD.setBrightnessScale(100);
  MatrixRGBCMD.setMinMireds(153);
  MatrixRGBCMD.setMaxMireds(500);
  MatrixRGBCMD.onStateCommand(MatrixRGB_State);
  MatrixRGBCMD.onBrightnessCommand(MatrixRGB_Brightness);
  MatrixRGBCMD.onColorTemperatureCommand(MatrixRGB_Temp);
  MatrixRGBCMD.onRGBColorCommand(MatrixRGB_Color);

  DoorlockRGB_Eff.setName("Doorlock RGB Effects");
  DoorlockRGB_Eff.setIcon("mdi:led-strip-variant");
  DoorlockRGB_Eff.setOptions("None;Breathe;Rainbow;Chase;Theater;Comet;Twinkle;Sparkle;Wipe;Bounce;Wheel;Fire;Strobe;Wave");
  DoorlockRGB_Eff.onCommand(DoorlockRGB_Effect);

  MatrixRGB_Eff.setName("Matrix RGB Effects");
  MatrixRGB_Eff.setIcon("mdi:led-strip-variant");
  MatrixRGB_Eff.setOptions("None;Breathe;Rainbow;Chase;Theater;Comet;Twinkle;Sparkle;Wipe;Bounce;Wheel;Fire;Strobe;Wave;Circular;TopDown;LeftRight");  
  MatrixRGB_Eff.onCommand(MatrixRGB_Effect);

  mqtt.onDisconnected(onMqttDisconnected);

  if(mqtt.begin(MQTT_BROKER_ADDR, MQTT_USERNAME, MQTT_PASSWORD)) 
  {
    DEBUG_PRINTLN("HA_MQTT_Init - MQTT Init Success");
    printLog("MQTT Init Successful");
  }
  else 
  {
    DEBUG_PRINTLN("HA_MQTT_Init - MQTT Init Failed");
    printLog("MQTT Init Failed");

    DEBUG_PRINTLN("HA_MQTT_Init - CRITICAL ERROR!");
    printLog("CRITICAL ERROR!");

    while(true)
    {
      //do nothing
    }
  }

  device.setAvailability(true);
}

void onMqttDisconnected() {
  DEBUG_PRINTLN("onMqttDisconnected - Disconnected from the broker!");
}

void HAButtonsHandler(HAButton* sender)
{
  if(sender == &DevModeCMD) 
  {
    reboot_devmode = true;
    UpdateEEPROM();
    delay(1000);
    ESP.restart();
  }
  if(sender == &RebootCMD) 
  {
    ESP.restart();
  } 
  else if(sender == &UnlockDoorCMD) 
  {
    DEBUG_PRINTLN("HAButtonsHandler - Unlock Command Received");
    UnlockDoor_Now = true;
  } 
  if(sender == &StopSoundsCMD) 
  {
    //todo
  }
}

void HASwitchHandler(bool state, HASwitch* sender)
{
  if(sender == &Buzzer1CMD) 
  {
    if(state) 
    {
      ExtraIO.digitalWrite(SX1509_BUZZER1_PIN, HIGH);
    } 
    else 
    {
      ExtraIO.digitalWrite(SX1509_BUZZER1_PIN, LOW);
    }
  }
  else if(sender == &Buzzer2CMD) 
  {
    if(state) 
    {
      ExtraIO.digitalWrite(SX1509_BUZZER2_PIN, HIGH);
    } 
    else 
    {
      ExtraIO.digitalWrite(SX1509_BUZZER2_PIN, LOW);
    }
  }
  else if(sender == &MuteCMD) 
  {
    if(state) 
    {
      //todo
    } 
    else 
    {
      //todo
    }
  }
  else if(sender == &DisMAlertsCMD) 
  {
    if(state) 
    {
      //todo
    } 
    else 
    {
      //todo
    }
  }
  else if(sender == &DisDAlertsCMD) 
  {
    if(state) 
    {
      //todo
    } 
    else 
    {
      //todo
    }
  }

  sender->setState(state); // report state back to the Home Assistant
}

//========================================================================================================================================
//======================= Matrix Led Helper Functions ========================
//========================================================================================================================================

uint16_t matrixXY(uint8_t x, uint8_t y) {
  if(MATRIX_SERPENTINE && (y % 2 == 1)) x = (MATRIX_W - 1) - x;
  uint8_t localIndex = y * MATRIX_W + x;
  return Matrix_LEDS[localIndex];
}
 
void matrixClear() {
  for(int i = 0; i < MATRIX_LEDS_NUM; i++) 
  {
    matrixBrightness[i] = 0.0f;
    pixels.setPixelColor(Matrix_LEDS[i], 0);
  }
  pixels.show();
}
 
uint8_t matrixMaxRing() {
  uint8_t cx = (MATRIX_W - 1) / 2;
  uint8_t cy = (MATRIX_H - 1) / 2;
  uint8_t rx = max(cx, (uint8_t)(MATRIX_W - 1 - cx));
  uint8_t ry = max(cy, (uint8_t)(MATRIX_H - 1 - cy));
  return max(rx, ry);
}
 
uint8_t matrixStepCount(MatrixEffectType type) {
  switch(type) 
  {
    case EFFECT_BOTTOM_TOP: return MATRIX_H;
    case EFFECT_LEFT_RIGHT: return MATRIX_W;
    case EFFECT_CIRCULAR:   return matrixMaxRing() + 1;
    default: return 0;
  }
}

void matrixAdvanceStep() {
  uint8_t steps = matrixStepCount(matrixActiveEffect);
  if(steps == 0) return;
 
  switch(matrixActiveEffect) 
  {
    case EFFECT_BOTTOM_TOP:
      for(uint8_t x = 0; x < MATRIX_W; x++) matrixBrightness[matrixStep * MATRIX_W + x] = 1.0f;
      break;
    case EFFECT_LEFT_RIGHT:
      for(uint8_t y = 0; y < MATRIX_H; y++) matrixBrightness[y * MATRIX_W + matrixStep] = 1.0f;
      break;
    case EFFECT_CIRCULAR: 
    {
      uint8_t cx = (MATRIX_W - 1) / 2;
      uint8_t cy = (MATRIX_H - 1) / 2;
      for(uint8_t y = 0; y < MATRIX_H; y++) 
      {
        for(uint8_t x = 0; x < MATRIX_W; x++) 
        {
          uint8_t dist = max(abs((int)x - (int)cx), abs((int)y - (int)cy));
          if(dist == matrixStep) matrixBrightness[y * MATRIX_W + x] = 1.0f;
        }
      }
      break;
    }
    default: break;
  }
  matrixStep = (matrixStep + 1) % steps;
}
 
bool matrixHasGlow() {
  for(int i = 0; i < MATRIX_LEDS_NUM; i++) if(matrixBrightness[i] > 0.0f) return true;
  return false;
}
 
void matrixRenderFrame() {
  for(uint8_t y = 0; y < MATRIX_H; y++) 
  {
    for(uint8_t x = 0; x < MATRIX_W; x++) 
    {
      uint8_t idx = y * MATRIX_W + x;
      float b = matrixBrightness[idx];
 
      if(b > 0.0f) 
      {
        b *= MATRIX_FADE;
        if(b < 0.004f) b = 0.0f;
        matrixBrightness[idx] = b;
      }
 
      uint8_t val = (uint8_t)(b * MATRIX_MAX_VAL);
      uint32_t color = (val == 0) ? 0 : pixels.gamma32(pixels.ColorHSV(matrixDisplayHue, MATRIX_SATURATION, val));
      pixels.setPixelColor(matrixXY(x, y), color);
    }
  }
  pixels.show();
}
 
void matrixStartEffect(MatrixEffectType type, uint16_t hue, uint16_t speedMs = MATRIX_WAVE_SPEED_MS) {
  matrixActiveEffect = type;
  matrixDisplayHue = hue;
  matrixStepDelay = speedMs;
  matrixStep = 0;
  matrixLastStepTime = millis();
}
 
void matrixStopEffect() {
  matrixActiveEffect = EFFECT_NONE;
}

void matrixUpdate() {
  unsigned long now = millis();
  bool active = (matrixActiveEffect != EFFECT_NONE);
 
  if(active && (now - matrixLastStepTime >= matrixStepDelay)) 
  {
    matrixLastStepTime = now;
    matrixAdvanceStep();
  }
 
  if(!active && !matrixHasGlow()) return;
 
  if(now - matrixLastFrameTime >= MATRIX_FRAME_MS) 
  {
    matrixLastFrameTime = now;
    matrixRenderFrame();
  }
}
 
// ========================================================================================================================================
// ======================= HA Light + Select Callbacks ========================
// ========================================================================================================================================

void DoorlockRGB_State(bool state, HALight* sender) {
  doorlockFX.setPower(state);
  sender->setState(state);
}

void DoorlockRGB_Brightness(uint8_t brightness, HALight* sender) {
  doorlockFX.setBrightness(brightness);
  sender->setBrightness(brightness);
}

void DoorlockRGB_Temp(uint16_t temperature, HALight* sender) {
  doorlockFX.setColor(miredsToColor(temperature));
  sender->setColorTemperature(temperature);
}

void DoorlockRGB_Color(HALight::RGBColor color, HALight* sender) {
  doorlockFX.setColor(((uint32_t)color.red << 16) | ((uint32_t)color.green << 8) | color.blue);
  sender->setRGBColor(color);
}

void DoorlockRGB_Effect(int8_t index, HASelect* sender) {
  if(index < 0) return;
  doorlockFX.setEffect((uint8_t)index);
  sender->setState(index);
}

void MatrixRGB_State(bool state, HALight* sender) {
  matrixFX.setPower(state);
  sender->setState(state);
}

void MatrixRGB_Brightness(uint8_t brightness, HALight* sender) {
  matrixFX.setBrightness(brightness);
  sender->setBrightness(brightness);
}

void MatrixRGB_Temp(uint16_t temperature, HALight* sender) {
  matrixFX.setColor(miredsToColor(temperature));
  sender->setColorTemperature(temperature);
}

void MatrixRGB_Color(HALight::RGBColor color, HALight* sender) {
  matrixFX.setColor(((uint32_t)color.red << 16) | ((uint32_t)color.green << 8) | color.blue);
  sender->setRGBColor(color);
}

void MatrixRGB_Effect(int8_t index, HASelect* sender) {
  if(index < 0) return;
  matrixFX.setEffect((uint8_t)index);
  sender->setState(index);
}

// ========================================================================================================================================
// ======================= Matrix Alert Functions ========================
//
//    Led matrix alert patterns
//
//    Gate opened               : bottom to top green wave
//    Lower stair node presence : bottom to top yellow wave
//    Lower stair node cbell    : bottom to top blue wave
//    Upper stair node presence : left to right orange wave
//    Upper stair node lidar    : left to right red wave
//    Hallway node 1 PIR        : circular yellow wave
//    Hallway node 2 PIR        : circular orange wave
//    Room door LIDAR           : circular red wave
//
//========================================================================================================================================

void Gate_Opened_Alert(bool state, HASwitch* sender) {
  //bottom to top green wave
  if(state)
  {
    matrixStartEffect(EFFECT_BOTTOM_TOP, HUE_GREEN);
  }
  else
  {
    matrixStopEffect();
  }
  sender->setState(state);

  lstair_pres_alert.setState(false);
  lstair_cbell_alert.setState(false);
  ustair_pres_alert.setState(false);
  ustair_lidar_alert.setState(false);
  hall_pir1_alert.setState(false);
}
 
void LStair_Presence_Alert(bool state, HASwitch* sender) {
  //bottom to top yellow wave
  if(state)
  {
    matrixStartEffect(EFFECT_BOTTOM_TOP, HUE_YELLOW);
  }
  else
  {
    matrixStopEffect();
  }
  sender->setState(state);

  gate_opened_alert.setState(false);
  lstair_cbell_alert.setState(false);
  ustair_pres_alert.setState(false);
  ustair_lidar_alert.setState(false);
  hall_pir1_alert.setState(false);
}
 
void LStair_Cbell_Alert(bool state, HASwitch* sender) {
  //blue 
  if(state)
  {
    matrixStartEffect(EFFECT_BOTTOM_TOP, HUE_BLUE);
  }
  else
  {
    matrixStopEffect();
  }
  sender->setState(state);

  gate_opened_alert.setState(false);
  ustair_pres_alert.setState(false);
  ustair_lidar_alert.setState(false);
  hall_pir1_alert.setState(false);
}

void UStair_Presence_Alert(bool state, HASwitch* sender) {
  //left to right orange wave
  if(state)
  {
    matrixStartEffect(EFFECT_LEFT_RIGHT, HUE_ORANGE);
  }
  else
  {
    matrixStopEffect();
  }
  sender->setState(state);

  gate_opened_alert.setState(false);
  lstair_pres_alert.setState(false);
  lstair_cbell_alert.setState(false);
  ustair_lidar_alert.setState(false);
  hall_pir1_alert.setState(false);
}
 
void UStair_Lidar_Alert(bool state, HASwitch* sender) {
  //left to right red wave
  if(state)
  {
    matrixStartEffect(EFFECT_LEFT_RIGHT, HUE_RED);
  }
  else
  {
    matrixStopEffect();
  }
  sender->setState(state);

  gate_opened_alert.setState(false);
  lstair_pres_alert.setState(false);
  lstair_cbell_alert.setState(false);
  ustair_pres_alert.setState(false);
  hall_pir1_alert.setState(false);
}
 
void Hall_PIR1_Alert(bool state, HASwitch* sender) {
  //circular yellow wave
  if(state)
  {
    matrixStartEffect(EFFECT_CIRCULAR, HUE_YELLOW);
  }
  else
  {
    matrixStopEffect();
  }
  sender->setState(state);

  gate_opened_alert.setState(false);
  lstair_pres_alert.setState(false);
  lstair_cbell_alert.setState(false);
  ustair_pres_alert.setState(false);
  ustair_lidar_alert.setState(false);
}
 
void Hall_PIR2_Alert(bool state) {
  //circular orange wave
  if(state)
  {
    matrixStartEffect(EFFECT_CIRCULAR, HUE_ORANGE);
  }
  else
  {
    matrixStopEffect();
  }

  gate_opened_alert.setState(false);
  lstair_pres_alert.setState(false);
  lstair_cbell_alert.setState(false);
  ustair_pres_alert.setState(false);
  ustair_lidar_alert.setState(false);
  hall_pir1_alert.setState(false);
}
 
void Door_Lidar_Alert(bool state) {
  //circular red wave
  if(state)
  {
    matrixStartEffect(EFFECT_CIRCULAR, HUE_RED);
  }
  else
  {
    matrixStopEffect();
  }
  
  gate_opened_alert.setState(false);
  lstair_pres_alert.setState(false);
  lstair_cbell_alert.setState(false);
  ustair_pres_alert.setState(false);
  ustair_lidar_alert.setState(false);
  hall_pir1_alert.setState(false);
}

// ========================================================================================================================================
// ======================= Doorlock Alert Functions ========================
// ========================================================================================================================================
 
//todo add gate opened, calling bell, lower stair presence, upper stair presence animations for doorlock leds

// ========================================================================================================================================
// ======================= Doorlock Core Functions ========================
// ========================================================================================================================================

void LockHandler() {

  if(UnlockDoor_Now)
  {
    ExtraIO.digitalWrite(SX1509_BUZZER1_PIN, HIGH);
    ExtraIO.digitalWrite(SX1509_BUZZER2_PIN, HIGH);
    ExtraIO.digitalWrite(SX1509_LOCK_PIN, LOW);
    
    UnlockDoor_Now = false;
    UnlockingStarted = true;
    UnlockingStarted_Time = millis();

    DEBUG_PRINTLN("LockHandler - Unlocked!");
  }
  else if(UnlockingStarted && (millis() - UnlockingStarted_Time) > 3000)
  {
    ExtraIO.digitalWrite(SX1509_LOCK_PIN, HIGH);
    UnlockingStarted = false;

    DEBUG_PRINTLN("LockHandler - Locked!");
  }
  else if(UnlockingStarted && (millis() - UnlockingStarted_Time) > 500)
  {
    ExtraIO.digitalWrite(SX1509_BUZZER1_PIN, LOW);
    ExtraIO.digitalWrite(SX1509_BUZZER2_PIN, LOW);

    DEBUG_PRINTLN("LockHandler - Buzzer off!");
  }
}

//========================================================================================================================================
//======================= GUI and Input Functions ========================
//========================================================================================================================================

void keyEventListener(KeypadEvent key, KeyState kpadState) {

  if(!TransmitAudioBTNHeld && kpadState == HOLD && key == '*')
  {
    ExtraIO.digitalWrite(SX1509_BUZZER1_PIN, HIGH);
    delay(100);
    ExtraIO.digitalWrite(SX1509_BUZZER1_PIN, LOW);
    TransmitAudioBTNHeld = true;
    DEBUG_PRINTLN("keyEventListener - TransmitAudioBTNHeld = true");
  }
  else if(TransmitAudioBTNHeld && kpadState == IDLE && key == '*')
  {
    ExtraIO.digitalWrite(SX1509_BUZZER1_PIN, HIGH);
    delay(100);
    ExtraIO.digitalWrite(SX1509_BUZZER1_PIN, LOW);
    TransmitAudioBTNHeld = false;
    DEBUG_PRINTLN("keyEventListener - TransmitAudioBTNHeld = false");
  }

  DEBUG_PRINT("keyEventListener - Key ");
  DEBUG_PRINT(key);
  switch(kpadState)
  {
    case PRESSED:    
      DEBUG_PRINTLN(" PRESSED.");
      break;
    case HOLD:
      DEBUG_PRINTLN(" HOLD.");
      break;
    case RELEASED:
      DEBUG_PRINTLN(" RELEASED.");
      break;
    case IDLE:
      DEBUG_PRINTLN(" IDLE.");
      break;
  }
}

//========================================================================================================================================
//======================= Intercom Functions ========================
//========================================================================================================================================

void Intercom_Init() {
  m_output_buffer = new OutputBuffer(300 * 16);
  m_input = new ADCSampler(ADC_UNIT_1, ADC_MIC_CHANNEL, i2s_adc_config);
  m_output = new I2SOutput(I2S_NUM_1, i2s_speaker_pins);
  m_transport = new UdpTransport(m_output_buffer);
  m_transport->set_header(TRANSPORT_HEADER_SIZE, transport_header);

  m_transport->begin();
  m_output->start(SAMPLE_RATE);
  m_output_buffer->flush();

  xTaskCreatePinnedToCore(Intercom_Task,  "Intercom_Task", 8192, NULL, 1, NULL, 0);
}

void Intercom_Task(void *param) {

  int16_t *samples = reinterpret_cast<int16_t *>(malloc(sizeof(int16_t) * 128));

  while(true)
  {
    if(digitalRead(DevBTN) || TransmitAudioBTNHeld)
    {
      DEBUG_PRINTLN("Intercom_Task - Started transmitting");

      // transmit for at least 1 second or while the button is pushed
      m_output->stop();
      m_input->start();

      unsigned long start_time = millis();
      uint32_t total = 0;
      while(millis() - start_time < 1000 || TransmitAudioBTNHeld)
      {
        int n = m_input->read(samples, 128);
        for(int i = 0; i < n; i++) m_transport->add_sample(samples[i]);
        total += n;
      }
      unsigned long ms = millis() - start_time;

      m_transport->flush();
      m_input->stop();
      m_output->start(SAMPLE_RATE);
      Serial.printf("Sent %u samples in %lu ms = %.0f Hz (want %d)\n", (unsigned)total, ms, total * 1000.0 / ms, SAMPLE_RATE);
      TransmitAudioNow = false;
      DEBUG_PRINTLN("Intercom_Task - Finished transmitting");
    }

    DEBUG_PRINTLN("Intercom_Task - Started Receiving");
    while(!digitalRead(DevBTN) && !TransmitAudioBTNHeld)
    {
      m_output_buffer->remove_samples(samples, 128); // read from the output buffer (which should be getting filled by the transport)
      m_output->write(samples, 128); // and send the samples to the speaker
    }
    DEBUG_PRINTLN("Intercom_Task - Finished Receiving");
  }
}