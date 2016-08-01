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


#ifndef _STEPPER_h
#define _STEPPER_h

#include "arduino.h"

#define POSITION_INSTANT_RELEASE_TIME_MS 100

class Stepper {

public:
	Stepper(float newStepsPerDegree, byte stepPin, byte DirPin, byte EnablePin);
	void setMaxVelocity(float maxStepsPerSecond);
	void setMaxAcceleration(float maxStepsPerSecondSquared);
	void setStepsPerDegree(float newStepsPerDegree);
	void setAllowedMotion(bool backwardAllowed, bool forwardAllowed);
	float getAngle();
	void setCurrentAngle(float currentDegrees);
	void gotoAngle(float targetDegrees);
	void goForward();
	void goBackward();
	void stop();
	void run(float dt);
	void disable();
	void enable();
	void setSoftLimitVelocities(float velocityForward, float velocityBackward);
  void setPositionMode(bool value);
  void setID(int ID);

private:
  int myID = 0;
	float maxVelocityForDistance();
	float stepsPerDegree;
	long motorPosition = 0;
	long position = 0;
  int lastStepAge = 0;
	float velocity = 0;
	float softLimitVelocityBackPercent = 1;
	float softLimitVelocityForwardPercent = 1;
	long positionTarget = 0;
	float maxVelocity;
	float maxAcceleration;
	byte pinStep;
	byte pinDir;
	byte pinEnable;
	bool running = false;
	bool runningForward = false;
	bool runningBackward = false;
	bool pulseActive = false;
	bool positionMode = true;
	bool directionForward = false;
	bool directionBackward = false;
	unsigned long directionStartTime = 0;
	byte counter = 0;
};

#endif

