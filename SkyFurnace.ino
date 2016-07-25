#include <LiquidCrystal_I2C.h>

#include <Wire.h>
#include <EEPROM.h>
#include "Pins.h"
#include "Display.h"
#include "Stepper.h"
#include "GPS.h"
#include "SolarController.h"
#include "Menu.h"
#include "Settings.h"
#include "Melexis.h"
#include "DataLink.h"

#ifndef MAX6675_H
#define MAX6675_H
#include <max6675.h>
#endif

// Note: this is using a modified MAX6675 library.  The .h file has been modified to include the typical #ifndef...#endif wrapper

#define FIRMWARE_VERSION "0.01"

// stepper steps per revolution = (motor's intrinsic steps per revolution) * (microstepping)
#define STEPPER_ELEV_STEPS_PER_REV 200.0 * 1.0

// stepper elevation revolutions per degree = (drive train ratio) * (timing belt gear ratio) * (rack and pinion gear ratio) / 360
#define STEPPER_ELEV_REVS_PER_DEGREE 100.0 * (50.0 / 25.0) * (70.0 / 10.0) / 360.0

// stepper steps per revolution = (motor's intrinsic steps per revolution) * (microstepping)
#define STEPPER_FOCUS_STEPS_PER_REV 200.0 * 1.0

// stepper focus revs per inch = revs per inch of the worm gear system
#define STEPPER_FOCUS_REVS_PER_INCH 12.0

#define STEPPER_ELEV_STEPS_PER_DEGREE (STEPPER_ELEV_STEPS_PER_REV * STEPPER_ELEV_REVS_PER_DEGREE)
#define STEPPER_FOCUS_STEPS_PER_INCH (STEPPER_FOCUS_STEPS_PER_REV * STEPPER_FOCUS_REVS_PER_INCH)

SolarController controller;
DataLink datalink;

void setup() {
  bool IRTempExists;

  Stepper stepper_elev(STEPPER_ELEV_STEPS_PER_DEGREE, PIN_STEPPER_ELEV_STEP, PIN_STEPPER_ELEV_DIR, PIN_STEPPER_ELEV_ENABLE);
  Stepper stepper_focus_left(STEPPER_FOCUS_STEPS_PER_INCH, PIN_STEPPER_FOCUS_LEFT_STEP, PIN_STEPPER_FOCUS_LEFT_DIR, PIN_STEPPER_FOCUS_LEFT_ENABLE);
  Stepper stepper_focus_right(STEPPER_FOCUS_STEPS_PER_INCH, PIN_STEPPER_FOCUS_RIGHT_STEP, PIN_STEPPER_FOCUS_RIGHT_DIR, PIN_STEPPER_FOCUS_RIGHT_ENABLE);
  Display displaypanel;
  GPS gps;

  Serial.begin(115200);
  Serial.println("\nStarting Setup");
  Serial.println("  Serial");

  Serial.println("  LCD");
  displaypanel.init();
  displaypanel.banner(FIRMWARE_VERSION);

  Serial.println("  IO Pins");
  pins_init();

  Serial.println("  GPS");
  Serial2.begin(9600);
  gps.init();

  Serial.println("  Serial Data Link");
  datalink.init(&Serial3);
  
  Serial.println("  Stepper Elev");
  stepper_elev.setID(0);
  stepper_elev.setMaxAcceleration(20000);
  stepper_elev.setMaxVelocity(5000);
  stepper_elev.enable();

  Serial.println("  Stepper Focus Left");
  stepper_elev.setID(1);
  stepper_focus_left.setMaxAcceleration(20000);
  stepper_focus_left.setMaxVelocity(5000);
  stepper_focus_left.enable();

  Serial.println("  Stepper Focus Right");
  stepper_elev.setID(2);
  stepper_focus_right.setMaxAcceleration(20000);
  stepper_focus_right.setMaxVelocity(5000);
  stepper_focus_right.enable();

  Serial.println("  1000 Hz Clock");
  configureTimerInterrupt();

  Serial.println("  Thermocouple A");
  MAX6675 thermocouple(PIN_THERMOCOUPLE_A_CLK, PIN_THERMOCOUPLE_A_CS, PIN_THERMOCOUPLE_A_DO);
  delay(500);
  Serial.print("    Current temperature is ");
  Serial.print(thermocouple.readFahrenheit());
  Serial.println(" F");

  Serial.println("  Checking for Melexis IR Thermometer");
  MelexisTempProbe irProbe;
  if (irProbe.sensorExists()) Serial.println("    Sensor found");
  else Serial.println("    Sensor not found");

  Serial.println("Finished Setup\n");

  Serial.println("Creating controller\n");

  controller.setup(&displaypanel, &stepper_elev, &stepper_focus_left, &stepper_focus_right, &gps, &thermocouple, &irProbe, &datalink);
  
  Serial.println("Checking for IO Test Request");
  if (getInput(PIN_TEMP_DOWN)) {
    Serial.println("Entering IO Test Mode");
    pins_test(displaypanel);
  }
  Serial.println("Checking for GPS Test Request");
  if (getInput(PIN_TEMP_UP)) {
    Serial.println("Entering GPS Test Mode");
    gps_test(gps, displaypanel);
  }

  Serial.println("Running controller\n");
  controller.run();
}

void configureTimerInterrupt() {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  //OCR1A = 1999;
  OCR1A = 1100;
  //OCR1A = 999;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

ISR(TIMER1_COMPA_vect) {
  controller.khz();
}

void loop() {
  // this never happens, because setup() calls controller.run(), which never returns
}


