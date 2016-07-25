#ifndef _MENU_h
#define _MENU_h

#include "Arduino.h"
#include "Display.h"
#include "Stepper.h"
#include "GPS.h"
#include "Pins.h"
#include <EEPROM.h>

#ifndef MAX6675_H
#define MAX6675_H
#include <max6675.h>
#endif


#define MENU_LENGTH 6

#define OPTION_EXIT_MENU 0
#define OPTION_HOME_ALL 1
#define OPTION_IO_TEST 2
#define OPTION_GPS_TEST 3
#define OPTION_TIMEZONE_SETTING 4
#define OPTION_TEMP_SOURCE 5

class Menu {

  public:
    void setup(Display *_displaypanel, Stepper *_stepper_elev, Stepper *_stepper_focus_left, Stepper *_stepper_focus_right, GPS *_gps, MAX6675 *_thermocouple);
    int show();
    
  private:
    Display *displaypanel;
    Stepper *stepper_elev;
    Stepper *stepper_focus_left;
    Stepper *stepper_focus_right;
    GPS *gps;
    MAX6675 *thermocouple;
    
    int runMain();
    void setTimezone();
    void setTempSource();
};

#endif

