#ifndef _PINS_h
#define _PINS_h

#include "Arduino.h"
#include "Display.h"

// LCD (Not changable, driven by I2C)

#define PIN_LCD_SDA 20
#define PIN_LCD_SCL 21

// Limit Switch Inputs

#define PIN_LIMIT_FOCUS_LEFT 43
#define PIN_LIMIT_FOCUS_RIGHT 51
#define PIN_LIMIT_ELEV 47

// Button Inputs

#define PIN_E_STOP 52
#define PIN_NAV_SELECT 26
#define PIN_NAV_UP 24
#define PIN_NAV_DOWN 24
#define PIN_ELEV_LEFT 32
#define PIN_ELEV_RIGHT 34
#define PIN_FOCUS_LEFT 28
#define PIN_FOCUS_RIGHT 30
#define PIN_AUTO_ELEV 50
#define PIN_AUTO_FOCUS 46
#define PIN_TEMP_UP 42
#define PIN_TEMP_DOWN 38

// Steppers

#define PIN_STEPPER_ELEV_ENABLE 2
#define PIN_STEPPER_ELEV_STEP 3
#define PIN_STEPPER_ELEV_DIR 4

#define PIN_STEPPER_FOCUS_LEFT_ENABLE 5
#define PIN_STEPPER_FOCUS_LEFT_STEP 6
#define PIN_STEPPER_FOCUS_LEFT_DIR 7

#define PIN_STEPPER_FOCUS_RIGHT_ENABLE 8
#define PIN_STEPPER_FOCUS_RIGHT_STEP 9
#define PIN_STEPPER_FOCUS_RIGHT_DIR 10

// GPS - (Not changable, driven by Serial2)

#define PIN_GPS_RX 17
#define PIN_GPS_TX 16

// LEDs

#define PIN_LED_NAV_SELECT 0
#define PIN_LED_NAV_UP 0
#define PIN_LED_NAV_DOWN 0
#define PIN_LED_AUTO_ELEV 48
#define PIN_LED_AUTO_FOCUS 44
#define PIN_LED_TEMP_UP 40
#define PIN_LED_TEMP_DOWN 36

// Thermocouples

#define PIN_THERMOCOUPLE_A_GND 41
#define PIN_THERMOCOUPLE_A_VCC 39
#define PIN_THERMOCOUPLE_A_DO 37
#define PIN_THERMOCOUPLE_A_CS 35
#define PIN_THERMOCOUPLE_A_CLK 33

#define LIMIT_DEBOUNCE_COUNT 8

#define PIN_DEBOUNCE_COUNT 20
#define PIN_LONG_PRESS_COUNT 600
#define PIN_REPEAT_DELAY 300
#define PIN_REPEAT_INTERVAL 50

void pins_init();
void pins_test(Display disp);
bool getInput(byte pin);
bool getInputDebounced(byte pin);
bool getLimitDebounced(byte pin);
bool getInputMomentary(byte pin, byte pinOff);
bool getInputMomentaryLong(byte pin, byte pinOff);
bool getInputMomentaryRepeating(byte pin);
void setPinOverride(byte pin, bool value);
bool getDoubleLong(byte pinA, byte pinB);

#endif

