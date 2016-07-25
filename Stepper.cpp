/* Stepper angular motion control class for combined position / manual control.
*
* Usage:
*
* Connect a stepper driver to the arduino with four pins:
* - ground
* - enable
* - step
* - direction
* You should configure these pins as outputs in your setup() routine.  Pins
* are assumed to be normally high; that is a logic 1 corresponds to the resting
* state and a logic 0 corresponds to the active state.
*
* Next, figure out how many steps ultimately correspond to a degree of rotation,
* after considering all of the mechanical components of the system.
* Create an instance of the class using these data:
*
* Stepper myStepper(StepsPerDegree, stepPin, directionPin, enablePin);
*
* Any stepper has limits to how quickly it can turn, and how quickly it can
* accelerate, before glitching / missing steps.  Once the stepper glitches,
* its accuracy is destroyed.  Consequently, we need to configure your stepper's
* max velocity and acceleration.  These are measured in steps per second, and
* steps per second per second respectively.
*
* myStepper.setMaxAcceleration(1000);
* myStepper.setMaxVelocity(500);
*
* To make the stepper motion controller operate, we need to call it's run()
* method at regular intervals.  The best way to do this is by setting up a
* timer interrupt.  A less reliable, but also possible method is to do this
* inside your main loop.  For example:
*
* while (true) {
*   myStepper.run(0.001); // 0.001 seconds since the last time run() was called
*   delay(1);
* }
*
* Now we can enable the device.  This both energizes the motor, and starts
* the control algorithm running:
*
* myStepper.enable();
*
* There are two different ways to control the stepper: manual, and position.
* Either mode can be used at any time; issues a command automatically switches
* the system to the correct mode.
*
* For manual control, the class includes two methods: goForward() and goBackward().
* These are instantanious commands - they work for a few milliseconds and then the
* motors will return to idle.  To sustain motion, goForward() and goBackward()
* should be called repeatedly.  For example, these commands could be combined
* with buttons for forward and backwards motion:
*
* while (true) {
*   if (digitalRead(BUTTON_FORWARD)) myStepper.goForward();
*   if (digitalRead(BUTTON_BACKWARD)) myStepper.goBackward();
* }
*
* For position control, just tell the stepper what angle to go to, and it'll go
* there:
*
* myStepper.gotoAngle(45);
*
* Created by Andy Fabian <andy@afabian.com>
*/

#include "Stepper.h"
#include "Pins.h"

int motorsMoving[3] = { 0, 0, 0 };
int lastEnabledState = 0;

Stepper::Stepper(float newStepsPerDegree, byte stepPin, byte dirPin, byte enablePin) {
	setStepsPerDegree(newStepsPerDegree);
	pinStep = stepPin;
	pinDir = dirPin;
	pinEnable = enablePin;
	digitalWrite(pinEnable, 1);
}

void Stepper::setMaxVelocity(float maxStepsPerSecond) {
	maxVelocity = maxStepsPerSecond;
}

void Stepper::setMaxAcceleration(float maxStepsPerSecondSquared) {
	maxAcceleration = maxStepsPerSecondSquared;
}

void Stepper::setStepsPerDegree(float newStepsPerDegree) {
	stepsPerDegree = newStepsPerDegree;
}

float Stepper::getAngle() {
	return (float)(position) / (stepsPerDegree * 10.0);
}

void Stepper::setCurrentAngle(float currentDegrees) {
	position = currentDegrees * stepsPerDegree * 10.0;
	positionTarget = position;
	motorPosition = position;
	velocity = 0;
}

void Stepper::gotoAngle(float targetDegrees) {
	positionTarget = targetDegrees * stepsPerDegree * 10.0;
	positionMode = true;
}

void Stepper::run(float dt) {
  
	// if raised the step pin in the last cycle, we must lower it this cycle, which means
	// that there's nothing else than needs to be done and we can return.
	if (pulseActive) {
		digitalWrite(pinStep, 1);
		pulseActive = false;
	}

	// if last cycle was not a pulse cycle
	else {

		// only run any control code if we are enabled
		if (running) {

			// if we're in position mode, generate a velocity based on the position we're trying to get to
			if (positionMode) {
				digitalWrite(13, 1);
				if (positionTarget > position) {
					velocity = min(maxVelocityForDistance(), velocity + (float)maxAcceleration * dt);
				}
				if (positionTarget < position) {
					velocity = max(-maxVelocityForDistance(), velocity - (float)maxAcceleration * dt);
				}
			}

			// if we're in manual mode, generate a velocity based on the current velocity and which way 
			// we've been asked to go
			else {
				digitalWrite(13, 0);
        float velocityChange = maxAcceleration * dt;
				if (directionForward) {
					velocity = min(maxVelocity, velocity + velocityChange);
					if (millis() - directionStartTime > POSITION_INSTANT_RELEASE_TIME_MS) stop();
				}
				else if (directionBackward) {
					velocity = max(-maxVelocity, velocity - velocityChange);
					if (millis() - directionStartTime > POSITION_INSTANT_RELEASE_TIME_MS) stop();
				}
				else if (velocity > 0) {
					velocity = max(0, velocity - velocityChange);
				}
				else if (velocity < 0) {
					velocity = min(0, velocity + velocityChange);
				}
			}

			// apply movement limits
			if (velocity > 0) {
				velocity = min(velocity, softLimitVelocityForwardPercent * maxVelocity);
				if (!runningForward) velocity = 0;
			}

			else if(velocity < 0) {
				velocity = max(velocity, -softLimitVelocityBackPercent * maxVelocity);
				if (!runningBackward) velocity = 0;
			}

      motorsMoving[myID] = velocity != 0;

      if (motorsMoving[0] || motorsMoving[1] || motorsMoving[2]) {
        if (lastEnabledState == 0) {
          digitalWrite(PIN_STEPPER_ELEV_ENABLE, !running);
          digitalWrite(PIN_STEPPER_FOCUS_LEFT_ENABLE, !running);
          digitalWrite(PIN_STEPPER_FOCUS_RIGHT_ENABLE, !running);
          lastEnabledState = 1;
        }  
      }
      else {
        if (lastEnabledState == 1) {
          digitalWrite(PIN_STEPPER_ELEV_ENABLE, 1);
          digitalWrite(PIN_STEPPER_FOCUS_LEFT_ENABLE, 1);
          digitalWrite(PIN_STEPPER_FOCUS_RIGHT_ENABLE, 1);
          lastEnabledState = 0;
        }
      }
      
			// update our position by adding the current velocity and dt
			position += velocity * dt;

			// apply to motor
			if (position - motorPosition > 5 && runningForward) {
				digitalWrite(pinStep, 0);
				digitalWrite(pinDir, 1);
				pulseActive = true;
				motorPosition += 10;
				if (position - motorPosition > 10) {
				  position = motorPosition;
				}
			}
			else if (motorPosition - position > 5 && runningBackward) {
				digitalWrite(pinStep, 0);
				digitalWrite(pinDir, 0);
				pulseActive = true;
				motorPosition -= 10;
				if (motorPosition - position > 10) {
				  position = motorPosition;
				}
			}
		}
	}
}

float Stepper::maxVelocityForDistance() {
	float distance = abs(positionTarget - position);
	double timeToStop = sqrt(distance / (float)maxAcceleration);
	return min(maxVelocity, (float)maxAcceleration * timeToStop);
}

void Stepper::disable() {
	running = false;
	digitalWrite(pinEnable, 1);
}

void Stepper::enable() {
	running = true;
	digitalWrite(pinEnable, 0);
}

void Stepper::setAllowedMotion(bool backwardAllowed, bool forwardAllowed) {
	runningForward = forwardAllowed;
	runningBackward = backwardAllowed;
}

void Stepper::goForward() {
	positionMode = false;
	directionForward = true;
	directionBackward = false;
	directionStartTime = millis();
}

void Stepper::goBackward() {
	positionMode = false;
	directionBackward = true;
	directionForward = false;
	directionStartTime = millis();
}

void Stepper::setPositionMode(bool value) {
  positionMode = value;
}

void Stepper::stop() {
	// note that stop() is only effective in manual mode, so it's not suitable for an 
	// emergency stop.  For that, use disable()
	directionForward = false;
	directionBackward = false;
  positionTarget = motorPosition;
}

void Stepper::setSoftLimitVelocities(float velocityForward, float velocityBackward) {
	softLimitVelocityForwardPercent = max(0.0, min(1.0, velocityForward));
	softLimitVelocityBackPercent = max(0.0, min(1.0, velocityBackward));
}

void Stepper::setID(int ID) {
  myID = ID;
}

