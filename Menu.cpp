#include "Menu.h"
#include "Arduino.h"
#include "Display.h"
#include "Stepper.h"
#include "GPS.h"
#include "Pins.h"
#include "Settings.h"
#include <EEPROM.h>

void Menu::setup(Display *_displaypanel, Stepper *_stepper_elev, Stepper *_stepper_focus_left, Stepper *_stepper_focus_right, GPS *_gps, MAX6675 *_thermocouple, SolarController *_solarController) {
  displaypanel = _displaypanel;
  stepper_elev = _stepper_elev;
  stepper_focus_left = _stepper_focus_left;
  stepper_focus_right = _stepper_focus_right;
  gps = _gps;
  thermocouple = _thermocouple;
  solarcontroller = _solarController;
}

int Menu::show() {
  int option;

  do {
    option = runMain();

    switch (option) {
      case OPTION_HOME_ALL:
        // will be handled by return value
        break;

      case OPTION_IO_TEST:
        pins_test(*displaypanel);
        break;

      case OPTION_GPS_TEST:
        gps_test(*gps, *displaypanel);
        break;

      case OPTION_TIMEZONE_SETTING:
        setTimezone();
        break;

      case OPTION_TEMP_SOURCE:
        setTempSource();
        break;

      case OPTION_PID_P:
        setPIDP();
        break;

      case OPTION_PID_I:
        setPIDI();
        break;

      case OPTION_PID_D:
        setPIDD();
        break;

      case OPTION_PID_INPUT_CUTOFF:
        setPIDInputCutoff();
        break;

      case OPTION_PID_D_CUTOFF:
        setPIDDCutoff();
        break;
    }
  }
  while (option != OPTION_EXIT_MENU && option != OPTION_HOME_ALL);

  return option;
}

int Menu::runMain() {
  String options[MENU_LENGTH];
  options[OPTION_EXIT_MENU] = "Exit Menu";
  options[OPTION_TEMP_SOURCE] = "Temperature Input";
  options[OPTION_HOME_ALL] = "Home All";
  options[OPTION_IO_TEST] = "IO Test";
  options[OPTION_GPS_TEST] = "GPS Test";
  options[OPTION_TIMEZONE_SETTING] = "Timezone Setting";
  options[OPTION_PID_P] = "PID P Gain";
  options[OPTION_PID_I] = "PID I Gain";
  options[OPTION_PID_D] = "PID D Gain";
  options[OPTION_PID_INPUT_CUTOFF] = "PID Input Lowpass";
  options[OPTION_PID_D_CUTOFF] = "PID D Lowpass";

  int currentOption = 0;


  while (1) {
    displaypanel->clear();
    displaypanel->setCursor(0, 0);
    displaypanel->print("MENU");

    while (getInput(PIN_AUTO_FOCUS) || getInput(PIN_AUTO_ELEV));
    
    int start = 0;
    if (currentOption > 1) start = currentOption - 1;
    if (currentOption == MENU_LENGTH - 1) start = currentOption - 2;

    for (int i=start; i<=start+2; i++) {
      if (i == currentOption) {
        displaypanel->setCursor(1, i-start+1); 
        displaypanel->print("->");
      }
      displaypanel->setCursor(3, i-start+1);
      displaypanel->print(options[i]);
    }

    int gotEnter = 0, gotUp = 0, gotDown = 0;
    while (!gotEnter && !gotUp && !gotDown) {
      gotEnter = getInputMomentary(PIN_AUTO_FOCUS, 0) || getInputMomentary(PIN_AUTO_ELEV, 0);
      gotUp = getInputMomentary(PIN_TEMP_UP, 0);
      gotDown = getInputMomentary(PIN_TEMP_DOWN, 0);
      delay(1);
    }

    if (gotUp && currentOption > 0) currentOption--;
    if (gotDown && currentOption < MENU_LENGTH-1) currentOption++;
    if (gotEnter) {
      return currentOption;
    }
  }
}

void Menu::setTempSource() {
  int gotEnter = 0, gotUp = 0, gotDown = 0;
  int source = loadTempSource();
  
  do {
      displaypanel->clear();
      displaypanel->setCursor(0, 0);
      displaypanel->print("TEMPERATURE SOURCE");
      displaypanel->setCursor(2, 2);
      if (source == 0) displaypanel->print("Thermocouple");
      if (source == 1) displaypanel->print("IR Probe");
      if (source == 2) displaypanel->print("Both (averaged)");
      
      gotEnter = 0; gotUp = 0; gotDown = 0;
      
      while (!gotEnter && !gotUp && !gotDown) {
        gotEnter = getInputMomentary(PIN_AUTO_FOCUS, 0) || getInputMomentary(PIN_AUTO_ELEV, 0);
        gotUp = getInputMomentary(PIN_TEMP_UP, 0);
        gotDown = getInputMomentary(PIN_TEMP_DOWN, 0);
        delay(1);

        if (gotUp) source--;
        if (gotDown) source++;
        if (source < 0) source += 3;
        if (source > 2) source -= 3;
      }
      
  } while (!gotEnter);

  saveTempSource(source);
}

void Menu::setTimezone() {
    int offset = loadTimezoneOffset();
    char buffer[20];
    int gotEnter = 0, gotUp = 0, gotDown = 0;

    do {
      displaypanel->clear();
      displaypanel->setCursor(0, 0);
      displaypanel->print("SET TIMEZONE OFFSET");
      displaypanel->setCursor(6, 2);
      dtostrf(offset, 3, 0, buffer);
      displaypanel->print(buffer);

      gotEnter = 0; gotUp = 0; gotDown = 0;
      while (!gotEnter && !gotUp && !gotDown) {
        gotEnter = getInputMomentary(PIN_AUTO_FOCUS, 0) || getInputMomentary(PIN_AUTO_ELEV, 0);
        gotUp = getInputMomentary(PIN_TEMP_UP, 0);
        gotDown = getInputMomentary(PIN_TEMP_DOWN, 0);
        delay(1);
      }

      if (gotUp && offset < 12) offset++;
      if (gotDown && offset > -12) offset--;
      
    } while (!gotEnter);

    saveTimezoneOffset(offset);
}

void Menu::setPIDP() {
  solarcontroller->savePIDP(adjustNumber("SET PID P", solarcontroller->loadPIDP(), 0.1, 0, 50));
}

void Menu::setPIDI() {
  solarcontroller->savePIDI(adjustNumber("SET PID I", solarcontroller->loadPIDI(), 0.1, 0, 50));
}

void Menu::setPIDD() {
  solarcontroller->savePIDD(adjustNumber("SET PID D", solarcontroller->loadPIDD(), 0.1, 0, 50));
}

void Menu::setPIDInputCutoff() {
  solarcontroller->savePIDInputCutoff(adjustNumber("SET PID INPUT CUTOFF", solarcontroller->loadPIDInputCutoff(), 0.1, 0, 50));
}

void Menu::setPIDDCutoff() {
  solarcontroller->savePIDDCutoff(adjustNumber("SET PID D CUTOFF", solarcontroller->loadPIDDCutoff(), 0.01, 0, 50));
}

float Menu::adjustNumber(String label, float initialValue, float stepSize, float minValue, float maxValue) {
  float value = initialValue;
  
  char buffer[20];
  int gotEnter = 0, gotUp = 0, gotDown = 0;

      do {
      displaypanel->clear();
      displaypanel->setCursor(0, 0);
      displaypanel->print(label);
      displaypanel->setCursor(6, 2);
      dtostrf(value, 3, stepSize == 0.1 ? 1 : 2, buffer);
      displaypanel->print(buffer);

      gotEnter = 0; gotUp = 0; gotDown = 0;
      while (!gotEnter && !gotUp && !gotDown) {
        gotEnter = getInputMomentary(PIN_AUTO_FOCUS, 0) || getInputMomentary(PIN_AUTO_ELEV, 0);
        gotUp = getInputMomentary(PIN_TEMP_UP, 0);
        gotDown = getInputMomentary(PIN_TEMP_DOWN, 0);
        delay(1);
      }

      if (gotUp && value < maxValue) value += stepSize;
      if (gotDown && value > minValue) value -= stepSize;
      //value = ((int)(value * 10.0)) / 10.0;
      
    } while (!gotEnter);

    return value;
}

