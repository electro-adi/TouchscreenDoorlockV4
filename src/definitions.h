#pragma once
#pragma GCC optimize("Os")

//-----------------------------------------EEPROM Variables

bool boot_animation = true;
bool enable_buzzer = true;
bool locked = false;

//-----------------------------------------Misc

#define NUMPIXELS 16

const char BUILD[] = __DATE__ " " __TIME__;
uint8_t log_counter = 0;
bool sd_available = true;

#if SERIAL_DEBUG != 0
  #define DEBUG_PRINT(x) Serial.print(x);
#else
  #define DEBUG_PRINT(x)
#endif

#if SERIAL_DEBUG != 0
  #define DEBUG_PRINTLN(x) Serial.println(x);
#else
  #define DEBUG_PRINTLN(x)
#endif

//----------------------------------------- Wifi stuff

#define NUM_SLOTS 5               //number of slots in eeprom to save wifi networks

uint8_t wifi_status = 0;
unsigned long last_scan;

String _ssid;
String _pass;

bool scan_now = false;
bool scan_complete = false;
bool webserver_mode = false;
bool connect_to_new_network = false;

String _update_error_str = "";
unsigned long _current_progress_size;

class WiFiResult
{
  public:
    bool duplicate;
    String SSID;
    uint8_t encryptionType;
    int32_t RSSI;
    uint8_t *BSSID;
    int32_t channel;
    bool isHidden;

  WiFiResult()
  {
  }
};

typedef int16_t wifi_ssid_count_t;
WiFiResult *wifiSSIDs;
wifi_ssid_count_t wifiSSIDCount;  //number of scanned ssids

//----------------------------------------- SD Card stuff

typedef struct
{
  String filename;
  String ftype;
  String fsize;
} fileinfo;

#define MAX_FILES 100
fileinfo Filenames[MAX_FILES]__attribute__((section(".ext_ram.bss")));
int numfiles = 0;

//----------------------------------------- Webserver stuff

#include <memory>
#include <rom/rtc.h>
// fix crash on ESP32 (see https://github.com/alanswx/ESPAsyncWiFiManager/issues/44)
int chosen_background;
int chosen_icon;

//-----------------------------------------IO Expander Stuff

const byte SX1509_ADDRESS = 0x3E; // SX1509 I2C address

// SX1509 pin definitions:

const byte MatrixPin1 = 0;
const byte MatrixPin2 = 1;
const byte MatrixPin3 = 2;
const byte MatrixPin4 = 3;
const byte MatrixPin5 = 4;
const byte MatrixPin6 = 5;
const byte MatrixPin7 = 6;

//-----------------------------------------Pins

#define SD_SCK    32
#define SD_MISO   35
#define SD_MOSI   33
#define SD_CS     0

#define I2S_MCK   23
#define I2S_BCLK  16
#define I2S_LRCK  17
#define I2S_DOUT  5
#define I2S_DIN   39

#define I2C_SDA   21
#define I2C_SCL   22

#define RGB_Leds  2

#define DevBTN    36


/*
  ESP32 DEVKIT Pinout:

  GPIO 36 : Webserver Button          GPIO 23 : I2S_MCK
  GPIO 39 : I2S_DIN                   GPIO 22 : I2C_SCL
  GPIO 34 :                           GPIO 1  : UART MAIN
  GPIO 35 : SD_MISO                   GPIO 3  : UART MAIN
  GPIO 32 : SD_SCK                    GPIO 21 : I2C_SDA
  GPIO 33 : SD_MOSI                   GPIO 19 : TFT_D5 
  GPIO 25 : TFT_D3                    GPIO 18 : TFT_D4
  GPIO 26 : TFT_D2                    GPIO 5  : I2S_DOUT
  GPIO 27 : TFT_D6                    GPIO 17 : I2S_LRCK
  GPIO 14 : TFT_D7                    GPIO 16 : I2S_BCLK
  GPIO 12 : TFT_D0                    GPIO 4  : TFT_WR
  GPIO 13 : TFT_D1                    GPIO 2  : Addressible Leds
  GPIO 9  : CAN'T USE                 GPIO 15 : TFT_DC
  GPIO 10 : CAN'T USE                 GPIO 0  : SD_CS (not connected)
  GPIO 11 : CAN'T USE                 GPIO 8  : CAN'T USE
  GND PIN                             GPIO 7  : CAN'T USE
  VIN PIN                             GPIO 6  : CAN'T USE
*/