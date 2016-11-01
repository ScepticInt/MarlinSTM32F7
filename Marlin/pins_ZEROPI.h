/**
 * Board ZeroPi pin assignments
 */

#define X_STEP_PIN          4
#define X_DIR_PIN           3
#define X_ENABLE_PIN        44
#define X_STOP_PIN          31//Rx

#define Y_STEP_PIN          6
#define Y_DIR_PIN           5
#define Y_ENABLE_PIN       48
#define Y_STOP_PIN          30//Tx

#define Z_STEP_PIN          12
#define Z_DIR_PIN           10
#define Z_ENABLE_PIN         52
#define Z_STOP_PIN          58//SWCLK


#if ENABLED(Z_MIN_PROBE_ENDSTOP)
  // Define a pin to use as the signal pin on Arduino for the Z_PROBE endstop.
  #define Z_MIN_PROBE_PIN  59//SWDIO
#endif


#define E0_STEP_PIN         8
#define E0_DIR_PIN          9
#define E0_ENABLE_PIN      11

#define SDPOWER            -1
#define SDSS               A3
#define LED_PIN            -1
#define FAN_PIN            56//-1
#define PS_ON_PIN          -1
#define KILL_PIN           -1

#define HEATER_0_PIN       57
#define HEATER_1_PIN       -1
#define HEATER_2_PIN       -1
#define TEMP_0_PIN          4   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define TEMP_1_PIN         -1   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define TEMP_2_PIN         -1   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define HEATER_BED_PIN     -1//56
#define TEMP_BED_PIN       5


#define LCD_PINS_RS        1
#define LCD_PINS_ENABLE    0
#define LCD_PINS_D4        PIN_WIRE_SCL
#define LCD_PINS_D5        PIN_WIRE_SDA
#define LCD_PINS_D6        SCK
#define LCD_PINS_D7        MOSI

#define BTN_EN1       15
#define BTN_EN2       14
#define BTN_ENC       16

#define SD_DETECT_PIN  -1


