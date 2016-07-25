#include "Pins.h"

void pins_init() {

	// Limit Switch Inputs

	pinMode(PIN_LIMIT_FOCUS_LEFT, INPUT_PULLUP);
	pinMode(PIN_LIMIT_FOCUS_RIGHT, INPUT_PULLUP);
	pinMode(PIN_LIMIT_ELEV, INPUT_PULLUP);

	// Button Inputs

	pinMode(PIN_E_STOP, INPUT_PULLUP);
	pinMode(PIN_NAV_SELECT, INPUT_PULLUP);
	pinMode(PIN_NAV_UP, INPUT_PULLUP);
	pinMode(PIN_NAV_DOWN, INPUT_PULLUP);
	pinMode(PIN_ELEV_LEFT, INPUT_PULLUP);
	pinMode(PIN_ELEV_RIGHT, INPUT_PULLUP);
	pinMode(PIN_FOCUS_LEFT, INPUT_PULLUP);
	pinMode(PIN_FOCUS_RIGHT, INPUT_PULLUP);
	pinMode(PIN_AUTO_ELEV, INPUT_PULLUP);
	pinMode(PIN_AUTO_FOCUS, INPUT_PULLUP);
	pinMode(PIN_TEMP_UP, INPUT_PULLUP);
	pinMode(PIN_TEMP_DOWN, INPUT_PULLUP);

	// Steppers

	pinMode(PIN_STEPPER_ELEV_ENABLE, OUTPUT);
	pinMode(PIN_STEPPER_ELEV_STEP, OUTPUT);
	pinMode(PIN_STEPPER_ELEV_DIR, OUTPUT);

	pinMode(PIN_STEPPER_FOCUS_LEFT_ENABLE, OUTPUT);
	pinMode(PIN_STEPPER_FOCUS_LEFT_STEP, OUTPUT);
	pinMode(PIN_STEPPER_FOCUS_LEFT_DIR, OUTPUT);

	pinMode(PIN_STEPPER_FOCUS_RIGHT_ENABLE, OUTPUT);
	pinMode(PIN_STEPPER_FOCUS_RIGHT_STEP, OUTPUT);
	pinMode(PIN_STEPPER_FOCUS_RIGHT_DIR, OUTPUT);

	// LEDs

	pinMode(PIN_LED_NAV_SELECT, OUTPUT);
	pinMode(PIN_LED_NAV_UP, OUTPUT);
	pinMode(PIN_LED_NAV_DOWN, OUTPUT);
	pinMode(PIN_LED_AUTO_ELEV, OUTPUT);
	pinMode(PIN_LED_AUTO_FOCUS, OUTPUT);
	pinMode(PIN_LED_TEMP_UP, OUTPUT);
	pinMode(PIN_LED_TEMP_DOWN, OUTPUT);

  // Thermocouples

  pinMode(PIN_THERMOCOUPLE_A_VCC, OUTPUT);
  pinMode(PIN_THERMOCOUPLE_A_GND, OUTPUT);
  digitalWrite(PIN_THERMOCOUPLE_A_VCC, HIGH);
  digitalWrite(PIN_THERMOCOUPLE_A_GND, LOW);  
}

bool pinOverrides[64] = { 0 };

void setPinOverride(byte pin, bool value) {
  pinOverrides[pin] = value;  
}

bool getInput(byte pin) {
	if (pin == PIN_E_STOP || pin == PIN_LIMIT_ELEV || pin == PIN_LIMIT_FOCUS_LEFT || pin == PIN_LIMIT_FOCUS_RIGHT) {
		return digitalRead(pin) || pinOverrides[pin];
	}
	else {
		return !digitalRead(pin) || pinOverrides[pin];
	}
}

bool getInputDebounced(byte pin) {
  static byte pinDebounceCounters[64] = { 0 };
  bool pinOn = getInput(pin);
  if (pinOn) {
    if (pinDebounceCounters[pin] < PIN_DEBOUNCE_COUNT) {
      pinDebounceCounters[pin]++;
    }
  }
  else {
    pinDebounceCounters[pin] = 0;
  }
  return pinDebounceCounters[pin] == PIN_DEBOUNCE_COUNT;
}

bool getLimitDebounced(byte pin) {
  static byte pinDebounceCounters[64] = { 0 };
  bool pinOn = getInput(pin);
  if (pinOn) {
    if (pinDebounceCounters[pin] < LIMIT_DEBOUNCE_COUNT) {
      pinDebounceCounters[pin]++;
    }
  }
  else {
    pinDebounceCounters[pin] = 0;
  }
  return pinDebounceCounters[pin] == LIMIT_DEBOUNCE_COUNT;
}

bool getInputMomentary(byte pin, byte pinOff) {
  static int pinDebounceCounters[64] = { 0 };
  bool pinOn = getInput(pin);
  if (pinOff) pinOn &= !getInput(pinOff);
  bool output = false;
  if (pinOn) {
    if (pinDebounceCounters[pin] < PIN_LONG_PRESS_COUNT + 1) {
      pinDebounceCounters[pin]++;
    }
  }
  else {
    if (pinDebounceCounters[pin] > 0) {
      //Serial.println(pinDebounceCounters[pin]);
      output = pinDebounceCounters[pin] > PIN_DEBOUNCE_COUNT && pinDebounceCounters[pin] < PIN_LONG_PRESS_COUNT;
    }
    pinDebounceCounters[pin] = 0;
  }
  return output;
}

bool getDoubleLong(byte pinA, byte pinB) {
  static int pinDebounceCounters[64] = { 0 };
  bool pinOnA = getInput(pinA);
  bool pinOnB = getInput(pinB);
  if (pinOnA) {
    if (pinDebounceCounters[pinA] < PIN_LONG_PRESS_COUNT * 0.5) {
      pinDebounceCounters[pinA]++;
    }
  }
  else {
    pinDebounceCounters[pinA] = 0;
  }
  if (pinOnB) {
    if (pinDebounceCounters[pinB] < PIN_LONG_PRESS_COUNT - 1) {
      pinDebounceCounters[pinB]++;
    }
  }
  else {
    pinDebounceCounters[pinB] = 0;
  }
  if (pinDebounceCounters[pinA] >= PIN_LONG_PRESS_COUNT * 0.5 && pinDebounceCounters[pinB] >= PIN_LONG_PRESS_COUNT * 0.5) {
    return true;
  }
  else {
    return false;
  }
  
}

bool getInputMomentaryLong(byte pin, byte pinOff) {
  static int pinDebounceCounters[64] = { 0 };
  bool pinOn = getInput(pin);
  if (pinOff) pinOn &= !getInput(pinOff);
  if (pinOn) {
    if (pinDebounceCounters[pin] < PIN_LONG_PRESS_COUNT + 1) {
      pinDebounceCounters[pin]++;
    }
  }
  else {
    pinDebounceCounters[pin] = 0;
  }
  return pinDebounceCounters[pin] == PIN_LONG_PRESS_COUNT;
}

bool getInputMomentaryRepeating(byte pin) {
  static int pinDebounceCounters[64] = { 0 };
  bool pinOn = getInput(pin);
  if (pinOn) {
    if (pinDebounceCounters[pin] == PIN_DEBOUNCE_COUNT + PIN_REPEAT_DELAY + PIN_REPEAT_INTERVAL) {
      pinDebounceCounters[pin] = PIN_DEBOUNCE_COUNT + PIN_REPEAT_DELAY;
    }
    pinDebounceCounters[pin]++;
  }
  else {
    pinDebounceCounters[pin] = 0;
  }
  return pinDebounceCounters[pin] == PIN_DEBOUNCE_COUNT || pinDebounceCounters[pin] == PIN_DEBOUNCE_COUNT + PIN_REPEAT_DELAY + PIN_REPEAT_INTERVAL;
}

void pins_test(Display disp) {

	Serial.println("IO Test Mode");

	disp.clear();
	disp.setCursor(0, 0);
	disp.print("IO Test Mode");

	while (true) {

		delay(100);

		disp.setCursor(0, 2);
		disp.print("                    ");
		disp.setCursor(0, 2);

		digitalWrite(PIN_LED_NAV_SELECT, LOW);
		digitalWrite(PIN_LED_NAV_UP, LOW);
		digitalWrite(PIN_LED_NAV_DOWN, LOW);
		digitalWrite(PIN_LED_AUTO_ELEV, LOW);
		digitalWrite(PIN_LED_AUTO_FOCUS, LOW);
		digitalWrite(PIN_LED_TEMP_UP, LOW);
		digitalWrite(PIN_LED_TEMP_DOWN, LOW);

		// Limit Switch Inputs

		if (getInput(PIN_LIMIT_FOCUS_LEFT)) {
			disp.print("PIN_LIMIT_FOCUS_LEFT");
		}
		else if (getInput(PIN_LIMIT_FOCUS_RIGHT)) {
			disp.print("PIN_LIMIT_FOCUS_RIGHT");
		}
		else if (getInput(PIN_LIMIT_ELEV)) {
			disp.print("PIN_LIMIT_ELEV");
		}

		// Button Inputs

		else if (getInput(PIN_E_STOP)) {
			disp.print("PIN_E_STOP");
		}
		else if (getInput(PIN_NAV_SELECT)) {
			digitalWrite(PIN_LED_NAV_SELECT, HIGH);
			disp.print("PIN_NAV_SELECT");
		}
		else if (getInput(PIN_NAV_UP)) {
			digitalWrite(PIN_LED_NAV_UP, HIGH);
			disp.print("PIN_NAV_UP");
		}
		else if (getInput(PIN_NAV_DOWN)) {
			digitalWrite(PIN_LED_NAV_DOWN, HIGH);
			disp.print("PIN_NAV_DOWN");
		}
		else if (getInput(PIN_ELEV_LEFT)) {
			disp.print("PIN_ELEV_LEFT");
		}
		else if (getInput(PIN_ELEV_RIGHT)) {
			disp.print("PIN_ELEV_RIGHT");
		}
		else if (getInput(PIN_FOCUS_LEFT)) {
			disp.print("PIN_FOCUS_LEFT");
		}
		else if (getInput(PIN_FOCUS_RIGHT)) {
			disp.print("PIN_FOCUS_RIGHT");
		}
		else if (getInput(PIN_AUTO_FOCUS)) {
			digitalWrite(PIN_LED_AUTO_FOCUS, HIGH);
			disp.print("PIN_AUTO_FOCUS");
		}
		else if (getInput(PIN_AUTO_ELEV)) {
			digitalWrite(PIN_LED_AUTO_ELEV, HIGH);
			disp.print("PIN_AUTO_ELEV");
		}
		else if (getInput(PIN_TEMP_UP)) {
			digitalWrite(PIN_LED_TEMP_UP, HIGH);
			disp.print("PIN_TEMP_UP");
		}
		else if (getInput(PIN_TEMP_DOWN)) {
			digitalWrite(PIN_LED_TEMP_DOWN, HIGH);
			disp.print("PIN_TEMP_DOWN");
		}
	}
}


