#include "Menu.h"
#include "Arduino.h"
#include "Display.h"
#include "Stepper.h"
#include "GPS.h"
#include "Pins.h"
#include "Settings.h"
#include <EEPROM.h>

void Menu::setup(Display *_displaypanel, Stepper *_stepper_elev, Stepper *_stepper_focus_left, Stepper *_stepper_focus_right, GPS *_gps, MAX6675 *_thermocouple) {
  displaypanel = _displaypanel;
  stepper_elev = _stepper_elev;
  stepper_focus_left = _stepper_focus_left;
  stepper_focus_right = _stepper_focus_right;
  gps = _gps;
  thermocouple = _thermocouple;
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
    }
  }
  while (option != OPTION_EXIT_MENU && option != OPTION_HOME_ALL);

  return option;
}

int Menu::runMain() {
  String options[MENU_LENGTH];
  options[OPTION_EXIT_MENU] = "Exit Menu";
  options[OPTION_HOME_ALL] = "Home All";
  options[OPTION_IO_TEST] = "IO Test";
  options[OPTION_GPS_TEST] = "GPS Test";
  options[OPTION_TIMEZONE_SETTING] = "Timezone Setting";
  options[OPTION_TEMP_SOURCE] = "Temperature Input";

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

