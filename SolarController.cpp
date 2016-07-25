#include "SolarController.h"
#include "Display.h"
#include "Menu.h"
#include "Settings.h"
#include "DataLink.h"

void SolarController::setup(Display *_displaypanel, Stepper *_stepper_elev, Stepper *_stepper_focus_left, Stepper *_stepper_focus_right, GPS *_gps, MAX6675 *_thermocouple,  MelexisTempProbe *_irProbe, DataLink *_datalink) {

  displaypanel = _displaypanel;
  menu = new Menu();
  menu->setup(_displaypanel, stepper_elev, stepper_focus_left, stepper_focus_right, _gps, thermocouple);

	stepper_elev = _stepper_elev;
	stepper_focus_left = _stepper_focus_left;
	stepper_focus_right = _stepper_focus_right;
	gps = _gps;
  thermocouple = _thermocouple;
  irProbe = _irProbe;
  datalink = _datalink;
	isSetup = true;
  timezoneOffset = loadTimezoneOffset();
  tempSource = loadTempSource();
  
  if (!getInput(PIN_TEMP_UP)) {
    stepper_elev->setCurrentAngle(loadElevation());
    stepper_focus_left->setCurrentAngle(loadFocus());
    stepper_focus_right->setCurrentAngle(loadFocus());
  }
}

void SolarController::khz() {
	if (isSetup) {
		stepper_elev->run(0.001);
		stepper_focus_left->run(0.001);
		stepper_focus_right->run(0.001);
	}
}

void SolarController::setHomeElev() {
  homingElev = 1;
  autoElev = 0;
}

void SolarController::setHomeFocus() {
  homingFocus = 1;
  autoFocus = 0;
}

void SolarController::run() {
  static unsigned long runcount = 0;
  static unsigned long lastruncount = 0;
  static bool gotThermalSample;
    
	while (true) {

    setPinOverride(PIN_FOCUS_LEFT, homingFocus);
    setPinOverride(PIN_ELEV_LEFT, homingElev);

    updateGPS();

		applyLimitsAndEStops();
    
    gotThermalSample = doThermocouple();
    doIRThermometer();
    switch (tempSource) {
      case 0: tempCurrent = tempThermocouple; break;
      case 1: tempCurrent = tempIR; break;
      case 2: tempCurrent = (tempThermocouple + tempIR) / 2.0; break;
    }
    doTempSetPointAdjustments();

    doModeSwitches();

    doTemperatureControl(gotThermalSample);

    doElevationControl();

    // other values set in PID controller
    datalink->setValue(5, stepper_focus_left->getAngle() * 100);
    datalink->setValue(6, tempCurrent * 100);
    datalink->setValue(7, stepper_elev->getAngle() * 100);
    
    
    runcount++;
		if (millis() - lastDisplayTime > DISPLAY_REFRESH_INTERVAL_MS / 4) {
		datalink->send();
    
			updateDisplay();    
			lastDisplayTime = millis();
      //Serial.print("Loops per interval: ");
      //Serial.println(runcount - lastruncount);
      lastruncount = runcount;
		}
	}
}

void SolarController::updateGPS() {
  while (Serial2.available()) {
    gps->handleByte(Serial2.read());
  }
}

void SolarController::doElevationControl() {
  static unsigned long lastElevTime = 0;

  if (autoElev) {
    if (gps->hasHadFix()) {
      if (millis() - lastElevTime > ELEV_CALCULATION_INTERVAL) {
        Serial.print("AutoElevationOffset=");
        Serial.println(autoElevOffset);
        Serial.print("CalcSunElev=");
        Serial.println(calcSunElev());
        float newTargetAngle = ELEV_HOME_ANGLE - calcSunElev() + autoElevOffset;
        Serial.print("newTargetAngle=");
        Serial.println(newTargetAngle);
        stepper_elev->gotoAngle(ELEV_HOME_ANGLE - newTargetAngle);
        lastElevTime = millis();
      }
    }
    if (getInputDebounced(PIN_ELEV_LEFT) || getInputDebounced(PIN_ELEV_RIGHT)) {
      autoElev = false;
      float angle = stepper_elev->getAngle();
      stepper_elev->gotoAngle(angle);
    }
  }

  else /* !autoElev */ {
    if (getInput(PIN_ELEV_LEFT)) {
      stepper_elev->goBackward();
    }
    else if (getInput(PIN_ELEV_RIGHT)) {
      stepper_elev->goForward();
    }
  }
}

void SolarController::doTemperatureControl(bool gotSample) {
  float err = (tempSetPoint - tempCurrent) * 0.01 /* dt */;
  digitalWrite(PIN_LED_TEMP_DOWN, err > -0.01);
  digitalWrite(PIN_LED_TEMP_UP, err < 0.01);

  if (autoFocus) {
    if (gotSample) {
      float P = err * FOCUS_P_COEFF;
      if (err != NAN) ISum += err;
      float I = ISum * FOCUS_I_COEFF;
      float D = (err - lastD) * FOCUS_D_COEFF;
      lastD = err;
      float PID = P + I + D;
      datalink->setValue(0, P * 100);
      datalink->setValue(1, I * 100);
      datalink->setValue(2, D * 100);
      datalink->setValue(3, err * 100);
      datalink->setValue(4, PID * 100);
//      Serial.print(" err="); Serial.print(err);
//      Serial.print(" p="); Serial.print(P);
//      Serial.print(" i="); Serial.print(I);
//      Serial.print(" d="); Serial.print(D);
//      Serial.print(" pid="); Serial.print(PID);
//      Serial.print(" derr="); Serial.print(err - lastD);
//      Serial.print(" isum="); Serial.println(ISum);
      
      float target = stepper_focus_left->getAngle() + PID;
        target = min(min(focusSetMaxExtension, FOCUS_SOFT_LIMIT_HIGH_INCHES), max(0, target));
      stepper_focus_left->gotoAngle(target);
      stepper_focus_right->gotoAngle(target);
    }

    if (getInputDebounced(PIN_FOCUS_LEFT) || getInputDebounced(PIN_FOCUS_RIGHT)) {
      autoFocus = false;
      float angle = stepper_focus_left->getAngle();
      stepper_focus_left->gotoAngle(angle);
      stepper_focus_right->gotoAngle(angle);
    }
  }

  else /* !autoFocus */ {
    if (getInput(PIN_FOCUS_LEFT)) {
      stepper_focus_left->goBackward();
      stepper_focus_right->goBackward();
    }
    else if (getInput(PIN_FOCUS_RIGHT)) {
      stepper_focus_left->goForward();
      stepper_focus_right->goForward();
    }
  }
}

bool SolarController::doIRThermometer() {
    if (irProbe->sensorExists()) {
      tempIR = irProbe->getTargetTemp();
      return true;  
    }
    else {
      return false;
    }
}

bool SolarController::doThermocouple() {
  static byte thermocoupleClock;
  static int thermocoupleReadings[THERMOCOUPLE_BUFFER_COUNT];
  static byte thermocoupleIndex = 0;
  unsigned int thermocoupleSum = 0;

  if (thermocoupleClock-- == 0) {
    // clock divider because the thermocouple boards seems to lock on a temperature if we query it too quickly.
    
    thermocoupleReadings[thermocoupleIndex] = thermocouple->readFahrenheit();
    thermocoupleIndex = (thermocoupleIndex + 1) % THERMOCOUPLE_BUFFER_COUNT;
    thermocoupleClock = 250;
    for (byte i=0; i<THERMOCOUPLE_BUFFER_COUNT; i++) {
      thermocoupleSum += thermocoupleReadings[i];
    }
    tempThermocouple = (float)thermocoupleSum / (float)THERMOCOUPLE_BUFFER_COUNT;
    return true;
  }
  else {
    return false;
  }
}

void SolarController::doTempSetPointAdjustments() {
  if (getInputMomentaryRepeating(PIN_TEMP_UP)) tempSetPoint++;
  if (getInputMomentaryRepeating(PIN_TEMP_DOWN)) tempSetPoint--;
}

void SolarController::enableAutoElev() {
  if (autoElevHomed) {
    if (gps->hasHadFix()) {
      setLon = gps->getLon();
      autoElev = 1;
    }
    else {
      errorDisplay("Need GPS Fix");
    }
  }
  else {
    errorDisplay("Auto Elev Not Homed");
  }
}

void SolarController::doModeSwitches() {

  // If both temperature buttons held, home all
  if (getDoubleLong(PIN_TEMP_UP, PIN_TEMP_DOWN)) {
    Serial.println("Homing all");
    setHomeElev();
    setHomeFocus();  
  }

  // If both focus buttons held, menu
  if (getDoubleLong(PIN_AUTO_FOCUS, PIN_AUTO_ELEV)) {
    Serial.println("Showing menu");
    int option = menu->show();
    displayResetNeeded = 1;
    if (option == OPTION_HOME_ALL) {
      setHomeElev();
      setHomeFocus();  
    }
    timezoneOffset = loadTimezoneOffset();
    tempSource = loadTempSource();
    displayResetNeeded = true;
  }

  // If AutoFocus pressed, toggle AutoFocus
  if (getInputMomentary(PIN_AUTO_FOCUS, PIN_AUTO_ELEV)) {
    if (autoFocusHomed) {
      autoFocus = !autoFocus;
      if (!autoFocus) {
        float angle = stepper_focus_left->getAngle();
        stepper_focus_left->gotoAngle(angle);
        stepper_focus_right->gotoAngle(angle);
      }
    }
    else {
      errorDisplay("AutoFocus Not Homed");
    }
  }

  // If AutoFocus held and not in autofocus mode, set autofocus extent limit
  if (getInputMomentaryLong(PIN_AUTO_FOCUS, PIN_AUTO_ELEV) && !autoFocus) {
    focusSetMaxExtension = stepper_focus_left->getAngle();
    for (int i=0; i<8; i++) {
      digitalWrite(PIN_LED_AUTO_FOCUS, 0);
      delay(100);
      digitalWrite(PIN_LED_AUTO_FOCUS, 1);
      delay(100);
    }
  }

  // If AutoElev pressed, toggle AutoElev
  if (getInputMomentary(PIN_AUTO_ELEV, PIN_AUTO_FOCUS)) {
    if (autoElev) {
      autoElev = false;
      float angle = stepper_elev->getAngle();
      stepper_elev->gotoAngle(angle);
    }
    else {
      enableAutoElev();
    }
  }

  // If AutoElev held and not in autoelev mode, set elev offset
  if (getInputMomentaryLong(PIN_AUTO_ELEV, PIN_AUTO_FOCUS) && !autoElev) {
    if (autoElevHomed) {
      if (gps->hasHadFix()) {
        setLat = gps->getLat();
        setLon = gps->getLon();
        Serial.print("Angle Offset: ");
        Serial.println(ELEV_HOME_ANGLE - calcSunElev());
        Serial.println(stepper_elev->getAngle());
        Serial.println(ELEV_HOME_ANGLE - stepper_elev->getAngle());
        Serial.println(calcSunElev());
        autoElevOffset = calcSunElev() - stepper_elev->getAngle();
        Serial.println(autoElevOffset);
        Serial.println(ELEV_HOME_ANGLE - calcSunElev() + autoElevOffset);
        for (int i=0; i<8; i++) {
          digitalWrite(PIN_LED_AUTO_ELEV, 1);
          delay(100);
          digitalWrite(PIN_LED_AUTO_ELEV, 0);
          delay(100);
        }
        enableAutoElev();
      }
      else {
        errorDisplay("Need GPS Fix");
      }
    }
    else {
      errorDisplay("Auto Elev Not Homed");
    }
  }
}

float SolarController::calcSunElev() {
  int dayOfYear = gps->getDayOfYear();
  int minuteOfDay = gps->getMinuteOfDayGMT() + timezoneOffset * 60;
  
  float x = (float)((dayOfYear - 1.0) + (minuteOfDay / 1440.0));
  //Serial.print("x=");Serial.println(x);
  float gamma = 0.017214 / x;
  //Serial.print("gamma="); Serial.println(gamma);
  float eqTime = 229.18 * (0.000075 +0.001868 * cos(gamma) - 0.032077 * sin(gamma) - 0.014615 * cos(2.0 * gamma) - 0.040849 * sin(2.0 * gamma));
  //Serial.print("eqTime="); Serial.println(eqTime);
  float declAngle = 0.006918 - (0.399912 * cos(gamma))+ 0.070257 * sin(gamma)-0.006758 * cos(2.0 * gamma) + 0.000907 * sin(2.0 * gamma) - 0.002697 * cos(3.0 * gamma) + 0.00148 * sin(3.0 * gamma);
  //Serial.print("declAngle="); Serial.println(declAngle);
  float timeOffset = eqTime - 4.0 * setLon - 60.0 * timezoneOffset;
  //Serial.print("timeOffset="); Serial.println(timeOffset);
  float trueSolarTime = minuteOfDay + timeOffset;
  //Serial.print("trueSolarTime="); Serial.println(trueSolarTime);
  float solarHourAngle = (trueSolarTime / 4.0) - 180.0;
  //Serial.print("solarHourAngle="); Serial.println(solarHourAngle);
  float solarZenith = DEGREES(acos(((sin(RADIANS(setLat))*sin(declAngle))+(cos(RADIANS(setLat))*cos(declAngle)*cos(RADIANS(solarHourAngle))))));
  //Serial.print("solarZenith="); Serial.println(solarZenith);
  float solarElevation = 90.0 - solarZenith;
  //Serial.print("solarElevation="); Serial.println(solarElevation);
  return solarElevation;  
}

float SolarController::DEGREES(float rad) {
  return rad * 180.0 / 3.14159;
}

float SolarController::RADIANS(float deg) {
  return deg * 3.14159 / 180.0;
}

void SolarController::applyLimitsAndEStops() {

	// logic to drive past limits safely

	static bool limitElevOverridePossible = true;
	static bool limitFocusOverridePossible = true;
	static bool limitElevOverrideActive = false;
	static bool limitFocusOverrideActive = false;

	if (stepper_focus_left->getAngle() > 0.1 && stepper_focus_right->getAngle() > 0.1) {
		limitFocusOverrideActive = false;
		limitFocusOverridePossible = false;
	}
	else if (getInput(PIN_FOCUS_LEFT) == false) {
		limitFocusOverridePossible = true;
    limitFocusOverrideActive = false;
	}
	else if (limitFocusOverridePossible && getInput(PIN_FOCUS_LEFT)) {
		limitFocusOverrideActive = true;
	}

	if (stepper_elev->getAngle() > 0.1) {
		limitElevOverrideActive = false;
		limitElevOverridePossible = false;
	}
	else if (getInput(PIN_ELEV_LEFT) == false) {
		limitElevOverridePossible = true;
    limitElevOverrideActive = false;
	}
	else if (limitElevOverridePossible && getInput(PIN_ELEV_LEFT)) {
		limitElevOverrideActive = true;
	}

  if (homingElev) {
    limitElevOverridePossible = true;
    limitElevOverrideActive = true;
  }

  if (homingFocus) {
    limitFocusOverridePossible = true;
    limitFocusOverrideActive = true;
  }
  
	// soft limits

	float softVelocityPercentForward = 1.0;
	float softVelocityPercentBackward = 1.0;
  
	float effective_focus_limit_high_inches = autoFocusHomed ? FOCUS_SOFT_LIMIT_HIGH_INCHES : FOCUS_SOFT_LIMIT_HIGH_INCHES_NOT_HOMED;
  if (autoFocus) effective_focus_limit_high_inches = min(effective_focus_limit_high_inches, focusSetMaxExtension);

	if (stepper_focus_left->getAngle() < FOCUS_SOFT_LIMIT_LOW_INCHES + FOCUS_SOFT_LIMIT_BUFFER_INCHES && stepper_focus_left->getAngle() >= FOCUS_SOFT_LIMIT_LOW_INCHES) {
		softVelocityPercentBackward = (stepper_focus_left->getAngle() - FOCUS_SOFT_LIMIT_LOW_INCHES) / FOCUS_SOFT_LIMIT_BUFFER_INCHES;
	}
	if (stepper_focus_left->getAngle() > effective_focus_limit_high_inches - FOCUS_SOFT_LIMIT_BUFFER_INCHES) {
		softVelocityPercentForward = (effective_focus_limit_high_inches - stepper_focus_left->getAngle()) / FOCUS_SOFT_LIMIT_BUFFER_INCHES;
	}

	softVelocityPercentForward = softVelocityPercentForward * 0.9 + 0.2;
	softVelocityPercentBackward = softVelocityPercentBackward * 0.9 + 0.2;

	if (stepper_focus_left->getAngle() <= FOCUS_SOFT_LIMIT_LOW_INCHES && !limitFocusOverrideActive) softVelocityPercentBackward = 0;

	stepper_focus_left->setSoftLimitVelocities(softVelocityPercentForward, softVelocityPercentBackward);
	stepper_focus_right->setSoftLimitVelocities(softVelocityPercentForward, softVelocityPercentBackward);

	softVelocityPercentBackward = 1.0;
	softVelocityPercentForward = 1.0;

	float effective_elev_limit_high_degrees = autoElevHomed ? ELEV_SOFT_LIMIT_HIGH_DEGREES : ELEV_SOFT_LIMIT_HIGH_DEGREES_NOT_HOMED;

	if (stepper_elev->getAngle() < ELEV_SOFT_LIMIT_LOW_DEGREES + ELEV_SOFT_LIMIT_BUFFER_DEGREES && stepper_elev->getAngle() >= ELEV_SOFT_LIMIT_LOW_DEGREES) {
		softVelocityPercentBackward = (stepper_elev->getAngle() - ELEV_SOFT_LIMIT_LOW_DEGREES) / ELEV_SOFT_LIMIT_BUFFER_DEGREES;
	}
	if (stepper_elev->getAngle() > effective_elev_limit_high_degrees - ELEV_SOFT_LIMIT_BUFFER_DEGREES) {
		softVelocityPercentForward = (effective_elev_limit_high_degrees - stepper_elev->getAngle()) / ELEV_SOFT_LIMIT_BUFFER_DEGREES;
	}

	softVelocityPercentForward = softVelocityPercentForward * 0.9 + 0.2;
	softVelocityPercentBackward = softVelocityPercentBackward * 0.9 + 0.2;

	if (stepper_elev->getAngle() <= ELEV_SOFT_LIMIT_LOW_DEGREES && !limitElevOverrideActive) softVelocityPercentBackward = 0;

	stepper_elev->setSoftLimitVelocities(softVelocityPercentForward, softVelocityPercentBackward);

	// E-STOP

	if (getInput(PIN_E_STOP)) {
		stepper_focus_left->setAllowedMotion(false, false);
		stepper_focus_right->setAllowedMotion(false, false);
		stepper_elev->setAllowedMotion(false, false);
	}
	else {
		stepper_focus_left->setAllowedMotion(true, true);
		stepper_focus_right->setAllowedMotion(true, true);
		stepper_elev->setAllowedMotion(true, true);
	}
  if (getInputMomentary(PIN_E_STOP, 0)) {
    saveFocus(stepper_focus_left->getAngle());
    saveElevation(stepper_elev->getAngle());
  }

	// Limit Switches

	if (getLimitDebounced(PIN_LIMIT_ELEV)) {
    Serial.println("Elev Limit");
		stepper_elev->setAllowedMotion(false, true);
		stepper_elev->setCurrentAngle(ELEV_LIMIT_HOME_OFFSET);
    homingElev = false;
    autoElev = false;
    Serial.println("Elev Homed");
		autoElevHomed = true;
		limitElevOverrideActive = false;
		limitElevOverridePossible = false;
    if (getInput(PIN_ELEV_LEFT)) {
		  stepper_elev->gotoAngle(0);
		  delay(2000); // time to let rebound finish without trigging limits
    }
	}
	if (getLimitDebounced(PIN_LIMIT_FOCUS_RIGHT)) {
    Serial.println("Focus Right Limit");
    stepper_focus_right->setAllowedMotion(false, true);
		stepper_focus_right->setCurrentAngle(FOCUS_LIMIT_HOME_OFFSET);
    if (autoFocus) {
      autoFocus = false;
      stepper_focus_left->stop();
      autoFocusHomed = false;
    }
	}
	if (getLimitDebounced(PIN_LIMIT_FOCUS_LEFT)) {
    Serial.println("Focus Left Limit");
		stepper_focus_left->setAllowedMotion(false, true);
		stepper_focus_left->setCurrentAngle(FOCUS_LIMIT_HOME_OFFSET);
    if (autoFocus) {
      autoFocus = false;
      autoFocusHomed = false;
      stepper_focus_right->stop();
    }
	}
	if (getLimitDebounced(PIN_LIMIT_FOCUS_LEFT) && getLimitDebounced(PIN_LIMIT_FOCUS_RIGHT)) {
    Serial.println("Focus Both Limit");
    stepper_focus_right->setCurrentAngle(FOCUS_LIMIT_HOME_OFFSET);
    stepper_focus_left->setCurrentAngle(FOCUS_LIMIT_HOME_OFFSET);
    homingFocus = false;
    Serial.println("Focus Homed");
		autoFocusHomed = true;
		limitFocusOverrideActive = false;
		limitFocusOverridePossible = false;
    if (getInput(PIN_FOCUS_LEFT)) {
		  stepper_focus_left->gotoAngle(0);
		  stepper_focus_right->gotoAngle(0);
		  delay(1000);
    }
	}
}

void SolarController::errorDisplay(String message) {
  displayResetNeeded = true;
  displaypanel->alert("NOPE", message);
  delay(3000); 
}

void SolarController::updateDisplay() {

	static bool messageDisplayed = false;
  static byte displaySection = 0;
  static bool flasher = 0;
	char buffer[20];

  flasher = !flasher;
	digitalWrite(PIN_LED_AUTO_FOCUS, autoFocus || (homingFocus && flasher));
	digitalWrite(PIN_LED_AUTO_ELEV, autoElev || (homingElev && flasher));
 
	resetDisplay();   

  displaySection = (displaySection + 1) % 4;

  switch (displaySection) {

  case 0:
  	displaypanel->setCursor(0, 0);
	  if (gps->hasHadFix()) {
      String timeString = "";
      int GPSMinutes = gps->getMinuteOfDayGMT() + timezoneOffset * 60;
      if (GPSMinutes < 0) GPSMinutes += 60 * 24;
      timeString = String((int)(GPSMinutes / 60)) + ":";
      GPSMinutes = GPSMinutes % 60;
      timeString += (GPSMinutes < 10 ? "0" : "") + String(GPSMinutes) + " ";
	  	displaypanel->print(timeString);
	  }
  	else {
	  	displaypanel->print("NO GPS");
	  }
    break;

  case 1:
  	displaypanel->setCursor(0, 1);
	  displaypanel->print(autoElevHomed ? "H" : "-");
	  displaypanel->setCursor(8, 1);
	  dtostrf(ELEV_HOME_ANGLE - stepper_elev->getAngle(), 6, 2, buffer);
	  displaypanel->print(buffer);
    break;

  case 2:
  	displaypanel->setCursor(0, 2);
	  displaypanel->print(autoFocusHomed ? "H" : "-");
	  displaypanel->setCursor(8, 2);
	  dtostrf(stepper_focus_left->getAngle(), 6, 2, buffer);
	  displaypanel->print(buffer);
    break;

  case 3:
  	displaypanel->setCursor(0, 3);
	  if (getInput(PIN_E_STOP)) {
		  displaypanel->print("EMERGENCY STOP      ");
		  messageDisplayed = true;
	  }
	  else if (getInput(PIN_LIMIT_ELEV) || getInput(PIN_LIMIT_FOCUS_LEFT) || getInput(PIN_LIMIT_FOCUS_RIGHT)) {
		  displaypanel->print("LIMIT SWITCH        ");
		  messageDisplayed = true;
	  }
	  else if (messageDisplayed) {
		  displayResetNeeded = true;
		  messageDisplayed = false;
	  }
	  else {
		  displaypanel->setCursor(8, 3);
		  dtostrf(tempCurrent, 3, 0, buffer);
		  displaypanel->print(buffer);
		  displaypanel->print("/");
		  dtostrf(tempSetPoint, 3, 0, buffer);
		  displaypanel->print(buffer);
	  }
    break;
  }
}

void SolarController::resetDisplay() {
	if (displayResetNeeded) {
		displaypanel->clear();
		displaypanel->setCursor(0, 1);
		displaypanel->print("  Elev         deg");
		displaypanel->setCursor(0, 2);
		displaypanel->print("  Focus        in");
		displaypanel->setCursor(0, 3);
    switch (tempSource) {
      case 0: displaypanel->print("K "); break;
      case 1: displaypanel->print("IR"); break;
      case 2: displaypanel->print("B "); break;
    }
		displaypanel->print("Temp            F");
		displayResetNeeded = false;
	}
}


