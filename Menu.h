#ifndef _MENU_h
#define _MENU_h

class Menu;

#include "Arduino.h"
#include "Display.h"
#include "Stepper.h"
#include "GPS.h"
#include "Pins.h"
#include "SolarController.h"
#include <EEPROM.h>

#ifndef MAX6675_H
#define MAX6675_H
#include <max6675.h>
#endif


#define MENU_LENGTH 11

#define OPTION_EXIT_MENU 0
#define OPTION_TEMP_SOURCE 1
#define OPTION_HOME_ALL 2
#define OPTION_IO_TEST 3
#define OPTION_GPS_TEST 4
#define OPTION_TIMEZONE_SETTING 5
#define OPTION_PID_P 6
#define OPTION_PID_I 7
#define OPTION_PID_D 8
#define OPTION_PID_INPUT_CUTOFF 9
#define OPTION_PID_D_CUTOFF 10

class Menu {

  public:
    void setup(Display *_displaypanel, Stepper *_stepper_elev, Stepper *_stepper_focus_left, Stepper *_stepper_focus_right, GPS *_gps, MAX6675 *_thermocouple, SolarController *_solarController);
    int show();
    
  private:
    Display *displaypanel;
    Stepper *stepper_elev;
    Stepper *stepper_focus_left;
    Stepper *stepper_focus_right;
    GPS *gps;
    MAX6675 *thermocouple;
    SolarController *solarcontroller;
    
    int runMain();
    void setTimezone();
    void setTempSource();
    void setPIDP();
    void setPIDI();
    void setPIDD();
    void setPIDInputCutoff();
    void setPIDDCutoff();
    float adjustNumber(String label, float initialValue, float stepSize, float minValue, float maxValue);
};

#endif

