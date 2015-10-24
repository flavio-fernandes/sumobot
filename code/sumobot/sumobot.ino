// Sumobot.ino
// Super simple Sumobot starter code
// Written by Brian Bailey
// brian@bunedoggle.com
// This code released under a Beerware license
// free for all to use, modify and share

/*******************************************************
* Include statements - include extra code we need
*******************************************************/
// This line includes the code we need to control servos
#include <Servo.h>
// This line includes the code we need to use the ultrasonic range finder
#include <NewPing.h>
#include "common.h"

/********************************************************
* Class object instances - These will help us control the
* sensors and motors.
********************************************************/
// NewPing setup of pins and maximum distance.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Servo motor control objects
// Up to twelve servo objects can be created and controlled
Servo rightWheel;
Servo leftWheel;

State state;

void initGlobals() {
  memset(&state, 0, sizeof(state));

  state.now = millis();

  state.frontRightSensorTriggerTs = state.now;
  state.frontLeftSensorTriggerTs = state.now;
  state.rearRightSensorTriggerTs = state.now;
  state.rearLeftSensorTriggerTs = state.now;

  updateNextTime(&state.nextFasttime, 2);
  updateNextTime(&state.next250time, 250);
  updateNextTime(&state.next500time, 500);
  updateNextTime(&state.next1000time, 1000);
  updateNextTime(&state.next5000time, 5000);
  updateNextTime(&state.next10000time, 10000);
  updateNextTime(&state.next25000time, 25000);
  updateNextTime(&state.next1mintime, 60000);
  updateNextTime(&state.next5mintime, (60000 * 5));
  updateNextTime(&state.next10mintime, (60000 * 10));
}

/********************************************************
* Setup code runs one time at the very beginning after we
* power up (or reset)
********************************************************/
void setup()
{
  initGlobals();

  // Open serial monitor so we can print out debug information
  // When connected to a USB port
  Serial.begin(9600);

  pinMode(FRONT_EDGE_LEFT_SENSOR, INPUT);
  pinMode(FRONT_EDGE_RIGHT_SENSOR, INPUT);
  pinMode(REAR_EDGE_LEFT_SENSOR, INPUT);
  pinMode(REAR_EDGE_RIGHT_SENSOR, INPUT);
	
  // Tell the servo objects which pins the servos are connected to
  leftWheel.attach(LEFT_WHEEL_PIN);
  rightWheel.attach(RIGHT_WHEEL_PIN);
  leftWheel.write(LEFT_WHEEL_STOP_VALUE);
  rightWheel.write(RIGHT_WHEEL_STOP_VALUE);

  // start a delay
  delay(5000);
}

/********************************************************
* Loop code runs over and over, forever while powered on
********************************************************/
void loop()
{
  state.now = millis();

  while (true) {
    if (state.nextFasttime <= state.now) {
      checkForEdges();

      updateNextTime(&state.nextFasttime, 2); continue;
    } else if (state.next250time <= state.now) {
      reduceSpeed();

      updateNextTime(&state.next250time, 250); continue;
    } else if (state.next500time <= state.now) {
      // changeMoveState();

      updateNextTime(&state.next500time, 500); continue;
    } else if (state.next1000time <= state.now) {   // 1 sec
      debugPrintSensors();

      updateNextTime(&state.next1000time, 1000); continue;
    } else if (state.next5000time <= state.now) {   // 5 secs

      updateNextTime(&state.next5000time, 5000); continue;
    } else if (state.next10000time <= state.now) {  // 10 secs

      updateNextTime(&state.next10000time, 10000); continue;
    } else if (state.next25000time <= state.now) {  // 25 seconds

      updateNextTime(&state.next25000time, 25000); continue;
    } else if (state.next1mintime <= state.now) {  // 1 min

      updateNextTime(&state.next1mintime, 60000); continue;
    } else if (state.next5mintime <= state.now) {  // 5 mins

      updateNextTime(&state.next5mintime, (60000 * 5)); continue;
    } else if (state.next10mintime <= state.now) {  // 10 mins

      updateNextTime(&state.next10mintime, (60000 * 10)); continue;
    }

    break;
  } // while
}

// -----

void updateNextTime(unsigned long *nextTimePtr, unsigned long increment) {
  *nextTimePtr = state.now + increment;
  while (*nextTimePtr < state.now) {
    Serial.println("FIXME: updateNextTime cannot handle wraps");
    delay(1000);
  }
}

// ----

void changeMoveState() {
  static int lastValue = 0;

  ++lastValue;
  if (lastValue == 1) {
    // go backwards
    leftWheel.write(0);
    rightWheel.write(180);
  } else if (lastValue == 2) {
    leftWheel.write(LEFT_WHEEL_STOP_VALUE);
    rightWheel.write(RIGHT_WHEEL_STOP_VALUE);
  } else if (lastValue == 4) {
    // go forward
    leftWheel.write(180);
    rightWheel.write(0);
  } else if (lastValue > 4) {
    leftWheel.write(LEFT_WHEEL_STOP_VALUE);
    rightWheel.write(RIGHT_WHEEL_STOP_VALUE);
    lastValue = 0;
  }

}

// ----

void debugPrintSensors() {
  int tmpValue;

  tmpValue = ping_cm_BugFix();
  Serial.print("Distance (cm): "); Serial.println(tmpValue);

  tmpValue = analogRead(FRONT_EDGE_LEFT_SENSOR);
  Serial.print("Front Edge Left: "); Serial.println(tmpValue);
  tmpValue = analogRead(FRONT_EDGE_RIGHT_SENSOR);
  Serial.print("Front Edge Right: "); Serial.println(tmpValue);

  tmpValue = analogRead(REAR_EDGE_LEFT_SENSOR);
  Serial.print("Rear Edge Left: "); Serial.println(tmpValue);
  tmpValue = analogRead(REAR_EDGE_RIGHT_SENSOR);
  Serial.print("Rear Edge Right: "); Serial.println(tmpValue);
}

// ----

void checkForEdges()
{
  int tmpValue;
  bool needToReactLeft = false;
  bool needToReactRight = false;

  // Less than LIGHT_COLOR_VALUE means we see a dark color
  // This number can be tweaked if the IR sensor is closer to the ground
  tmpValue = analogRead(FRONT_EDGE_LEFT_SENSOR);
  if (tmpValue > LIGHT_COLOR_VALUE) {
    state.leftWheelDirection = DIRECTION_BACK;
    state.leftWheelSpeed = SPEED_MAX;
    state.frontLeftSensorTriggerTs = state.now;
    needToReactLeft = true;
  } else {
    tmpValue = analogRead(REAR_EDGE_LEFT_SENSOR);
    if (tmpValue > LIGHT_COLOR_VALUE) {
      state.leftWheelDirection = DIRECTION_FWD;
      state.leftWheelSpeed = SPEED_MAX;
      state.rearLeftSensorTriggerTs = state.now;
      needToReactLeft = true;
    }
  }
  if (needToReactLeft) {
    setWheelSpeed(LEFT_WHEEL_PIN, state.leftWheelDirection, state.leftWheelSpeed);
  }

  tmpValue = analogRead(FRONT_EDGE_RIGHT_SENSOR);
  if (tmpValue > LIGHT_COLOR_VALUE) {
    state.rightWheelDirection = DIRECTION_BACK;
    state.rightWheelSpeed = SPEED_MAX;
    state.frontRightSensorTriggerTs = state.now;
    needToReactRight = true;
  } else {
    tmpValue = analogRead(REAR_EDGE_RIGHT_SENSOR);
    if (tmpValue > LIGHT_COLOR_VALUE) {
      state.rightWheelDirection = DIRECTION_FWD;
      state.rightWheelSpeed = SPEED_MAX;
      state.rearRightSensorTriggerTs = state.now;
      needToReactRight = true;
    }
  }
  if (needToReactRight) {
    setWheelSpeed(RIGHT_WHEEL_PIN, state.rightWheelDirection, state.rightWheelSpeed);

    if (!needToReactLeft && state.leftWheelSpeed == 0) {
      state.leftWheelSpeed = state.rightWheelSpeed / 2;
      state.leftWheelDirection = opositeDirection(state.rightWheelDirection);
      setWheelSpeed(LEFT_WHEEL_PIN, state.leftWheelDirection, state.leftWheelSpeed);
    }
  } else {
    // left needed to react, and right did not. make right assist left
    if (needToReactLeft && state.rightWheelSpeed == 0) {
      state.rightWheelSpeed = state.leftWheelSpeed / 2;
      state.rightWheelDirection = opositeDirection(state.leftWheelDirection);
      setWheelSpeed(RIGHT_WHEEL_PIN, state.rightWheelDirection, state.rightWheelSpeed);
    }
  }
}

// ----

int opositeDirection(int direction) {
  return direction == DIRECTION_FWD ? DIRECTION_BACK : DIRECTION_FWD;
}

// ----

void reduceSpeed()
{
  if (state.rightWheelSpeed > SPEED_MIN) {
    if ((state.frontRightSensorTriggerTs + SENSOR_REACT_INTERVAL_MS) < state.now &&
	(state.rearRightSensorTriggerTs  + SENSOR_REACT_INTERVAL_MS) < state.now) {
	setWheelSpeed(RIGHT_WHEEL_PIN, state.rightWheelDirection, --state.rightWheelSpeed);
      }
  }
  if (state.leftWheelSpeed > SPEED_MIN) {
    if ((state.frontLeftSensorTriggerTs + SENSOR_REACT_INTERVAL_MS) < state.now &&
	(state.rearLeftSensorTriggerTs + SENSOR_REACT_INTERVAL_MS) < state.now) {
      setWheelSpeed(LEFT_WHEEL_PIN, state.leftWheelDirection, --state.leftWheelSpeed);
    }
  }
}

// ----

void setWheelSpeed(int wheel, int direction, int speed)
{
  int servoValue;


  if (wheel == LEFT_WHEEL_PIN) {
    if (direction == DIRECTION_FWD) {
      servoValue = map(speed, SPEED_MIN, SPEED_MAX, LEFT_WHEEL_STOP_VALUE, 180);
    } else {
      servoValue = map(speed, SPEED_MIN, SPEED_MAX, LEFT_WHEEL_STOP_VALUE, 0);
    }

    Serial.print("left wheel set to: "); Serial.print(servoValue);
    Serial.print(" direction: "); Serial.print(direction);
    Serial.print(" speed: "); Serial.print(speed);
    Serial.println("");
#ifndef DO_NOT_MOVE
    leftWheel.write(servoValue);
#endif // #ifndef DO_NOT_MOVE
  } else {
    if (direction == DIRECTION_FWD) {
      servoValue = map(speed, SPEED_MIN, SPEED_MAX, RIGHT_WHEEL_STOP_VALUE, 0);
    } else {
      servoValue = map(speed, SPEED_MIN, SPEED_MAX, RIGHT_WHEEL_STOP_VALUE, 180);
    }
    Serial.print("right wheel set to: "); Serial.print(servoValue);
    Serial.print(" direction: "); Serial.print(direction);
    Serial.print(" speed: "); Serial.print(speed);
    Serial.println("");
#ifndef DO_NOT_MOVE
    rightWheel.write(servoValue);
#endif // #ifndef DO_NOT_MOVE
  }
}


// ----

// This is a wrapper function that tries to avoid a bug with the
// HC-SR04 modules where they can get stuck return zero forever
int ping_cm_BugFix() {
  const int distCm = sonar.ping_cm();

  if (distCm == 0) {
    delay(100);
    pinMode(ECHO_PIN, OUTPUT);
    digitalWrite(ECHO_PIN, LOW);
    delay(100);
    pinMode(ECHO_PIN, INPUT);
  }

  return distCm;
}

// ----
// ----

// =====

#if 0
// junk zone // blabla
void foo()
{
	// Variables we will need in the loop code
	int distInCentimeters;  // We'll store the ultrasonic range distance here
	int frontEdgeValue;     // This will be a reading from the IR edge sensor in the front
	int rearEdgeValue;      // This will be a reading from the IR edge sensor in the back
	

	/*******************************
	* IR Edge finder code
	********************************/
	// Check front and back edge sensors for edge of the ring
	// Lower numbers mean less reflective colors
	// 1023 means nothing was reflected.
	frontEdgeValue = analogRead(FRONT_EDGE_SENSOR);
	// Print front edge value for debug
	Serial.print("Front Edge: ");
	Serial.println(frontEdgeValue);
	// Less than LIGHT_COLOR_VALUE means we see a dark color
	// This number can be tweaked if the IR sensor is closer to the ground
	if(frontEdgeValue > LIGHT_COLOR_VALUE)
	{
		// Better back up!!
		Serial.println("Front edge detected, backing up!");
		leftWheel.write(180);
		rightWheel.write(0);
		delay(1000);
	}
	// Now check the back edge
	int backEdgeValue = analogRead(REAR_EDGE_SENSOR);
	Serial.print("Back Edge: ");
	Serial.println(backEdgeValue);
	if(backEdgeValue > LIGHT_COLOR_VALUE)
	{
		// Better go forward!!
		Serial.println("Back edge detected, moving forward!");
		leftWheel.write(0);
		rightWheel.write(180);
		delay(1000);
	}

	/*******************************
	* Ultrasonic Range Finder Code
	********************************/
	// This code takes a distance reading from the range finder
	distInCentimeters = sonar.ping_cm();

	// Print distance for debug (0 = outside set distance range)
	Serial.print("Ping: ");
	Serial.print(distInCentimeters);
	Serial.println("cm");

        // Check for ultrasonic rangefinder bug.  If we get a zero we
        // could be stuck, run the work around just in case.
        if ( distInCentimeters == 0 ) {
            delay(100);
            pinMode(ECHO_PIN, OUTPUT);
            digitalWrite(ECHO_PIN, LOW);
            delay(100);
            pinMode(ECHO_PIN, INPUT);
        }

	// If don't see opponent, spin to look around
	if(distInCentimeters > 39 || distInCentimeters == 0){
		Serial.println("Spinning");
		leftWheel.write(180);
		rightWheel.write(180);
	}
	// Otherwise attack!
	else {
		Serial.println("Attacking");
		leftWheel.write(0);
		rightWheel.write(180);
	}
	
	delay(50);
}
#endif
