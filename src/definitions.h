#pragma once
#pragma GCC optimize("Os")

//-----------------------------------------EEPROM Variables

bool reboot_devmode = false;
bool mute = false;
bool disable_keypad_unlocking = false;
bool disable_doorlock_functionality = false;

//-----------------------------------------Misc

const char BUILD[] = __DATE__ " " __TIME__;
uint8_t log_counter = 0;
bool sd_available = true;
bool dev_mode = false;
bool setup_complete = false;
unsigned long lastUptimeUpdateAt = 0;

int8_t CMD_fromHA = 0;
//int8_t CMD_toHA = 0; not needed

/*
  CMD Numbers and their meaning

  ---------From HA---------

  0   - no command, default state
  1   - gate opened
  2   - gate closed
  3   - calling bell rang
  4   - lower stair node LIDAR presence detected
  5   - upper stair node PIR presence detected
  6   - upper stair node LIDAR presence detected
  7   - hallway presence detected
  8   - stop all sounds
  9   - disable keypad unlocking
  10  - enable keypad unlocking
  11  - disable doorlock functionality
  12  - enable doorlock functionality

  ---------To HA-----------

  0   - no command, default state
  1   - pin correct
  2   - pin incorrect
  3   - turn on room lights
  4   - turn off room lights
  5   - turn on room fairy lights
  6   - turn off room fairy lights
  7   - turn off all room devices and lights
  8   - open windows
  9   - close windows
  10  - unlock button pressed
*/


#define MAC_ADDR_LENGTH 6

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

//----------------------------------------- Display Stuff

const int LOG_LEFT_MARGIN  = 5;
const int LOG_RIGHT_MARGIN = 5;
const int LOG_TOP_MARGIN   = 8;
const int LOG_LINE_HEIGHT  = 10;

std::vector<String> logLines;

#define RED           0xF800
#define ORANGE        0xFAC0
#define FAINT_ORANGE  0x2060
#define BLUE          0x1F
#define LIGHT_BLUE    0x55F
#define FAINT_BLUE    0x10A5
#define GREEN         0x7E0
#define FAINT_GREEN   0x1985
#define YELLOW        0xFD00
#define LIGHT_YELLOW  0x9B00

//----------------------------------------- Sensors Stuff

unsigned long last_sync = 0;

//0x38 is AHT
//0x3E is SX1509
//0x50 is LIDAR

// Intervals (ms)
#define lidar_read_interval  1000
#define aht10_read_interval  30000

// Timestamps
unsigned long last_lidar_read = 0;
unsigned long last_aht10_read = 0;

// Presence flags 
bool LIDAR_presence = false;
bool Hall_presence  = false;
bool Room_presence  = false;

// Lidar Specific
int lidar_baseline                  = -1;
int lidar_last_reading              = -1;
unsigned long lidar_last_trigger_ms = 0;

const int LIDAR_PRESENCE_HOLD_MS    = 60000;
const int LIDAR_CHANGE_THRESHOLD    = 400;
const int LIDAR_BASELINE_MARGIN     = 60;
const float LIDAR_BASELINE_ALPHA    = 0.08f;
const int LIDAR_SAMPLES             = 3;

// PIR Specific
bool pir1_level = false, pir2_level = false;
unsigned long pir_last_trigger_ms1        = 0;
unsigned long pir_last_trigger_ms2        = 0;
const unsigned long PIR_PRESENCE_HOLD_MS  = 60000;

// Reed Specific
bool Door_open = false;

//------------------------- Led Stuff

#define NUMPIXELS 35
#define DOORLOCKLEDS_NUM 26
#define MATRIX_LEDS_NUM 9

int Doorlock_LEDS[DOORLOCKLEDS_NUM] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25};
int Matrix_LEDS[MATRIX_LEDS_NUM] = {26, 27, 28, 29, 30, 31, 32, 33, 34};

#define MATRIX_W 3
#define MATRIX_H 3
#define MATRIX_SERPENTINE true

#define HUE_RED         0
#define HUE_ORANGE      5461
#define HUE_YELLOW      9929
#define HUE_GREEN       21845
#define HUE_BLUE        43690
 
//-----------------------------------------IO Expander Stuff

const byte SX1509_ADDRESS = 0x3E; // SX1509 I2C address

const byte SX1509_PIR1_PIN = 7;
const byte SX1509_BUZZER1_PIN = 8;   // Buzzer inside the doorlock (hall area)
const byte SX1509_LOCK_PIN = 15;     // Blue Wire
const byte SX1509_BUZZER2_PIN = 14;  // Yellow Wire (inside the room)
const byte SX1509_BUTTON_PIN = 13;   // Orange Wire
const byte SX1509_REED_PIN = 12;     // Brown Wire
const byte SX1509_PIR2_PIN = 11;     // Black Wire (PIR inside the room)

#define KEY_ROWS 4
#define KEY_COLS 3

byte rowPins[KEY_ROWS] = {5, 0, 1, 3};
byte colPins[KEY_COLS] = {4, 6, 2};

char keyMap[KEY_ROWS][KEY_COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

#define ROW_MASK ((1u<<5)|(1u<<0)|(1u<<1)|(1u<<3))
#define COL_MASK ((1u<<4)|(1u<<6)|(1u<<2))
#define IO_INT_MASK ((1u<<SX1509_PIR1_PIN)|(1u<<SX1509_PIR2_PIN)|(1u<<SX1509_REED_PIN)|(1u<<SX1509_BUTTON_PIN)|ROW_MASK)

enum KeypadIRQState : uint8_t { KP_IDLE_WATCH, KP_ACTIVE_POLL, KP_RELEASE_WATCH };
KeypadIRQState kp_state = KP_IDLE_WATCH;
unsigned long kp_last_poll = 0;
unsigned long kp_active_since = 0;   // when ACTIVE_POLL was entered
#define KP_SETTLE_MS 60

#define IO_OUT_MASK ((1u << SX1509_BUZZER1_PIN) | (1u << SX1509_BUZZER2_PIN) | (1u << SX1509_LOCK_PIN))

bool io_healthy = true;

//pin6 - col1, pin5 - row0, pin4 - col0, pin3 - row3, pin2 - col2, pin1 - row2, pin0 - row1

//-----------------------------------------Lock Variables

bool UnlockDoor_Now = false;
bool UnlockingStarted = false;
unsigned long UnlockingStarted_Time;

int PIN[5] = {-1, -1, -1, -1, -1};

bool incorrect_pin = false;
bool correct_pin = false;
unsigned long reset_pin_at = 0;

//-----------------------------------------Menu Variables

uint8_t MENU = 1;

/*
  MENU 1: Lock Home Menu
  MENU 2: HA Menu
  MENU 3: Alert Menu
*/
bool Generate_Screen = true;
bool Update_Screen = false;

//-----------------------------------------Buzzer Variables

unsigned long buzzer1_off_at = 0;
unsigned long buzzer2_off_at = 0;

//-----------------------------------------Intercom Stuff

#define SAMPLE_RATE 16000
#define ADC_RATE_DIVIDER 0.5f

#define ADC_MIC_CHANNEL ADC1_CHANNEL_6

// In case all transport packets need a header (to avoid interference with other applications or walkie talkie sets), 
// specify TRANSPORT_HEADER_SIZE (the length in bytes of the header) in the next line, and define the transport header
#define TRANSPORT_HEADER_SIZE 3
uint8_t transport_header[TRANSPORT_HEADER_SIZE] = {0x1F, 0xCD, 0x01};

volatile bool TransmitAudioBTNHeld = false;
volatile bool intercom_receiving = false;
volatile unsigned long intercom_last_received = 0;
unsigned long intercom_calling_off_at = 0;

//-----------------------------------------Pins

#define SD_SCK    32
#define SD_MISO   35
#define SD_MOSI   33
#define SD_CS     0

#define SX1509_INT_PIN 2

#define I2S_BCLK  17 // Mic SCK + Amp BCLK
#define I2S_LRC   16 // Mic WS + Amp LRC
#define I2S_DIN   4  // Mic SD
#define I2S_DOUT  23 // Amp DIN

#define I2C_SDA   21
#define I2C_SCL   22

#define RGB_PIN   5

#define DevBTN    36


/*

  SX1509 IO Expander Pinout:
    
  PIN 0   : Matrix Keypad             PIN 15  : Solenoid Lock
  PIN 1   : Matrix Keypad             PIN 14  : Buzzer (Room)
  PIN 2   : Matrix Keypad             PIN 13  : Button (Room)
  PIN 3   : Matrix Keypad             PIN 12  : Reed Door Sensor
  PIN 4   : Matrix Keypad             PIN 11  : PIR Sensor (Room)
  PIN 5   : Matrix Keypad             PIN 10  :
  PIN 6   : Matrix Keypad             PIN 9   :
  PIN 7   : PIR Sensor (Hall)         PIN 8   : Buzzer (Hall)

  Connections going through the door: 

  White   : 5v +
  Grey    : GND
  Orange  : Button
  Yellow  : Buzzer
  Green   : Addressible LEDs
  Brown   : Reed Door Sensor
  Blue    : Solenoid Lock
  Black   : PIR Sensor

  
  ESP32 DEVKIT Pinout:

  GPIO 36 : Webserver Button          GPIO 23 : I2S_DOUT
  GPIO 39 :                           GPIO 22 : I2C_SCL
  GPIO 34 : ADC MIC                   GPIO 1  : UART MAIN
  GPIO 35 : SD_MISO                   GPIO 3  : UART MAIN
  GPIO 32 : SD_SCK                    GPIO 21 : I2C_SDA
  GPIO 33 : SD_MOSI                   GPIO 19 : TFT_D5 
  GPIO 25 : TFT_D3                    GPIO 18 : TFT_D4
  GPIO 26 : TFT_D2                    GPIO 5  : Addressible Leds
  GPIO 27 : TFT_D6                    GPIO 17 : I2S_BCLK
  GPIO 14 : TFT_D7                    GPIO 16 : I2S_LRC
  GPIO 12 : TFT_D0                    GPIO 4  : TFT_WR
  GPIO 13 : TFT_D1                    GPIO 2  : SX1509 Interrupt
  GPIO 9  : CAN'T USE                 GPIO 15 : TFT_DC
  GPIO 10 : CAN'T USE                 GPIO 0  : SD_CS (not connected)
  GPIO 11 : CAN'T USE                 GPIO 8  : CAN'T USE
  GND PIN                             GPIO 7  : CAN'T USE
  VIN PIN                             GPIO 6  : CAN'T USE
*/