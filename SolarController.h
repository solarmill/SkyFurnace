#ifndef _SOLARCONTROLLER_h
#define _SOLARCONTROLLER_h

#include "Arduino.h"
#include "Display.h"
#include "Stepper.h"
#include "GPS.h"
#include "Pins.h"
#include "Menu.h"
#include "Melexis.h"
#include "DataLink.h"
#include <EEPROM.h>

#ifndef MAX6675_H
#define MAX6675_H
#include <max6675.h>
#endif

#define ELEV_CALCULATION_INTERVAL 1000

#define DISPLAY_REFRESH_INTERVAL_MS 500

#define FOCUS_SOFT_LIMIT_LOW_INCHES 0.0
#define FOCUS_SOFT_LIMIT_HIGH_INCHES 22.25
#define FOCUS_SOFT_LIMIT_HIGH_INCHES_NOT_HOMED 0.5
#define FOCUS_SOFT_LIMIT_BUFFER_INCHES 0.10
#define FOCUS_LIMIT_HOME_OFFSET -0.15

#define ELEV_SOFT_LIMIT_LOW_DEGREES 0.0
#define ELEV_SOFT_LIMIT_HIGH_DEGREES 75.0
#define ELEV_SOFT_LIMIT_HIGH_DEGREES_NOT_HOMED 20.0
#define ELEV_SOFT_LIMIT_BUFFER_DEGREES 0.5
#define ELEV_LIMIT_HOME_OFFSET -3.0
#define ELEV_HOME_ANGLE 88.6

#define THERMOCOUPLE_BUFFER_COUNT 10

// settings for raw temperature probe
//#define FOCUS_P_COEFF 10.0
//#define FOCUS_I_COEFF 0.0
//#define FOCUS_D_COEFF 15.0

// settings for iron kettle
#define FOCUS_P_COEFF 4.0
#define FOCUS_I_COEFF 0.0
#define FOCUS_D_COEFF 15.0

class SolarController {

public:
	void setup(Display *_displaypanel, Stepper *_stepper_elev, Stepper *_stepper_focus_left, Stepper *_stepper_focus_right, GPS *_gps, MAX6675 *_thermocouple, MelexisTempProbe *_irProbe, DataLink *_datalink);
	void run();
	void khz();
  void setHomeElev();
  void setHomeFocus();
  void setIRTempExists(bool value);
  
private:
  Menu *menu;
	Display *displaypanel;
	Stepper *stepper_elev;
	Stepper *stepper_focus_left;
	Stepper *stepper_focus_right;
	GPS *gps;
  MAX6675 *thermocouple;
  MelexisTempProbe *irProbe;
  DataLink *datalink;
  
	bool isSetup = false;

	bool autoFocus = false;
	bool autoElev = false;
	bool autoFocusHomed = false;
	bool autoElevHomed = false;
	unsigned long lastDisplayTime = 0;
	bool displayResetNeeded = true;
	float tempSetPoint = 150.0;
	float tempThermocouple = 0;
  float tempCurrent = 0;
  float tempIR = 0;
  float focusSetMaxExtension = 99.9;
  float autoElevOffset = 0;
  bool homingFocus = 0, homingElev = 0;
  
	void applyLimitsAndEStops();
	void updateDisplay();
	void resetDisplay();
  void updateGPS();
  float DEGREES(float rad);
  float RADIANS(float deg);
  void errorDisplay(String message);
  int timezoneOffset;
  int tempSource;
  
  float ISum = 0;
  float lastD = 0;
  float setLat, setLon;

  float calcSunElev();
  void doTempSetPointAdjustments();
  void doModeSwitches();
  bool doThermocouple();
  bool doIRThermometer();
  void doTemperatureControl(bool gotSample);
  void doElevationControl();
  void enableAutoElev();
};

#endif

