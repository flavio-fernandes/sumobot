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

  state.ringScanClosestDistanceTs = state.now;

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
  Serial.begin(115200);

  pinMode(FRONT_EDGE_LEFT_SENSOR, INPUT);
  pinMode(FRONT_EDGE_RIGHT_SENSOR, INPUT);
  pinMode(REAR_EDGE_LEFT_SENSOR, INPUT);
  pinMode(REAR_EDGE_RIGHT_SENSOR, INPUT);
	
  // Tell the servo objects which pins the servos are connected to
  leftWheel.attach(LEFT_WHEEL_PIN);
  rightWheel.attach(RIGHT_WHEEL_PIN);
  leftWheel.write(LEFT_WHEEL_STOP_VALUE);
  rightWheel.write(RIGHT_WHEEL_STOP_VALUE);

  Serial.println("init done");
  // start a delay
  delay(5000);
  Serial.println("sumo fighter unleashed!");
}

/********************************************************
* Loop code runs over and over, forever while powered on
********************************************************/
void loop()
{
  static RobotMode lastRobotMode = robotModeCount;

  state.now = millis();

  while (true) {
    if (lastRobotMode != state.robotMode) {
      Serial.print("Robot mode changing from "); Serial.print(lastRobotMode); Serial.print(" to ");
      Serial.print(state.robotMode); Serial.println("");
      lastRobotMode = state.robotMode;
    }

    checkForEdges();

    if (state.nextFasttime <= state.now) {
      checkStopRingScan();
      checkFaceObstacle();

      updateNextTime(&state.nextFasttime, 2); continue;
    } else if (state.next250time <= state.now) {
      reduceEdgeAvoidSpeed();

      updateNextTime(&state.next250time, 250); continue;
    } else if (state.next500time <= state.now) {
      // ringScanPing();

      updateNextTime(&state.next500time, 500); continue;
    } else if (state.next1000time <= state.now) {   // 1 sec
      checkAttackProgress();

      updateNextTime(&state.next1000time, 1000); continue;
    } else if (state.next5000time <= state.now) {   // 5 secs
      checkStartRingScan();

      updateNextTime(&state.next5000time, 5000); continue;
    } else if (state.next10000time <= state.now) {  // 10 secs
      debugPrintSensors();

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

  // LEFT SIDE (1 of 2)
  tmpValue = analogRead(FRONT_EDGE_LEFT_SENSOR);
  if (tmpValue > LIGHT_COLOR_VALUE) {
    state.leftWheelDirection = directionBackward;
    state.frontLeftSensorTriggerTs = state.now;
    state.leftWheelSpeed = SPEED_MAX;
    needToReactLeft = true;
  } else {
    tmpValue = analogRead(REAR_EDGE_LEFT_SENSOR);
    if (tmpValue > LIGHT_COLOR_VALUE) {
      state.leftWheelDirection = directionForward;
      state.rearLeftSensorTriggerTs = state.now;
      state.leftWheelSpeed = SPEED_MAX;
      needToReactLeft = true;
    }
  }
  if (needToReactLeft) {
    setWheelSpeed(wheelLeft, state.leftWheelDirection, state.leftWheelSpeed);
  }

  // RIGHT SIDE
  tmpValue = analogRead(FRONT_EDGE_RIGHT_SENSOR);
  if (tmpValue > LIGHT_COLOR_VALUE) {
    state.rightWheelDirection = directionBackward;
    state.frontRightSensorTriggerTs = state.now;
    state.rightWheelSpeed = SPEED_MAX;
    needToReactRight = true;
  } else {
    tmpValue = analogRead(REAR_EDGE_RIGHT_SENSOR);
    if (tmpValue > LIGHT_COLOR_VALUE) {
      state.rightWheelDirection = directionForward;
      state.rearRightSensorTriggerTs = state.now;
      state.rightWheelSpeed = SPEED_MAX;
      needToReactRight = true;
    }
  }
  if (needToReactRight) {
    setWheelSpeed(wheelRight, state.rightWheelDirection, state.rightWheelSpeed);

    // if we made it here, we know that right wheel was engaged to deal with an
    // edge situation. If the other wheel is currently not moving make it rotate
    // the other way to enhance the reacting wheel.
    if (state.leftWheelSpeed == 0) {
      state.leftWheelSpeed = state.rightWheelSpeed / 2;
      state.leftWheelDirection = opositeDirection(state.rightWheelDirection);
      setWheelSpeed(wheelLeft, state.leftWheelDirection, state.leftWheelSpeed);
    }
  } else {

    // LEFT SIDE (2 of 2)
    if (needToReactLeft && state.rightWheelSpeed == 0) {
      // if we made it here, we know that left wheel was engaged to deal with an
      // edge situation. If the other wheel is currently not moving, make it rotate
      // the other way to enhance the reacting wheel.
      state.rightWheelSpeed = state.leftWheelSpeed / 2;
      state.rightWheelDirection = opositeDirection(state.leftWheelDirection);
      setWheelSpeed(wheelRight, state.rightWheelDirection, state.rightWheelSpeed);
    }
  }

  if (needToReactLeft || needToReactRight) {
    state.robotMode = robotModeAvoidEdge;
  }
}

// ----

Direction opositeDirection(Direction direction) {
  return direction == directionForward ? directionBackward : directionForward;
}

// ----

void reduceEdgeAvoidSpeed()
{
  if (state.robotMode != robotModeAvoidEdge) return;

  // if wheel has non min speed, check if we are still within the minimum reaction
  // interval since sensor triggered. If so, reduce speed by 1.

  if (state.rightWheelSpeed > SPEED_MIN) {
    if ((state.frontRightSensorTriggerTs + SENSOR_REACT_INTERVAL_MS) < state.now &&
	(state.rearRightSensorTriggerTs  + SENSOR_REACT_INTERVAL_MS) < state.now) {
	setWheelSpeed(wheelRight, state.rightWheelDirection, --state.rightWheelSpeed);
      }
  }
  if (state.leftWheelSpeed > SPEED_MIN) {
    if ((state.frontLeftSensorTriggerTs + SENSOR_REACT_INTERVAL_MS) < state.now &&
	(state.rearLeftSensorTriggerTs + SENSOR_REACT_INTERVAL_MS) < state.now) {
      setWheelSpeed(wheelLeft, state.leftWheelDirection, --state.leftWheelSpeed);
    }
  }

  if (state.rightWheelSpeed == SPEED_MIN && state.leftWheelSpeed == SPEED_MIN) {
    state.robotMode = robotModeNoop;
  }
}

// ----

void checkStartRingScan() {
  if (state.robotMode != robotModeNoop) return;
  if ((state.ringScanClosestDistanceTs + RING_SCAN_TIME) >= state.now) return;  // scan/attacked happened recently
  
  state.ringScanClosestDistanceTs = state.now;
  state.ringScanClosestDistance = ping_cm_BugFix();

  state.leftWheelDirection = state.ringScanLeftDirection;
  state.rightWheelDirection = opositeDirection(state.leftWheelDirection);
  state.leftWheelSpeed = 
    state.rightWheelSpeed = SPEED_MAX;
  setWheelSpeed(wheelLeft, state.leftWheelDirection, state.leftWheelSpeed);
  setWheelSpeed(wheelRight, state.rightWheelDirection, state.rightWheelSpeed);
  
  // calculate how long to spin back till obstacle in in front of robot
  updateNextTime(&state.ringScanObstacleTime, RING_SCAN_TIME);

  state.robotMode = robotModeRingScan;
  state.ringScanLeftDirection = opositeDirection(state.ringScanLeftDirection);  // for next time
}

// ----

void ringScanPing() {
  if (state.robotMode != robotModeRingScan) return;

  const int pingSnapshot = ping_cm_BugFix();

  if (pingSnapshot <= state.ringScanClosestDistance) {
    state.ringScanClosestDistanceTs = state.now;
    state.ringScanClosestDistance = pingSnapshot;

    Serial.print("New best target distance (cm): "); Serial.print(state.ringScanClosestDistance);
    Serial.println("");
  }
}

// ----

void checkStopRingScan() {
  if (state.robotMode != robotModeRingScan) return;

  if (state.ringScanObstacleTime > state.now) return;  // scan is not done

  Serial.print("Best target distance (cm): "); Serial.print(state.ringScanClosestDistance);

  // if obstacle is too far, then there are none... simply do nothing
  if (state.ringScanClosestDistance >= MAX_TARGET_DISTANCE) {
    Serial.println(" ... will not attack because that is too far");
    setStateNoop();
    return;
  }

  // calculate how long to spin back till obstacle in in front of robot
  const unsigned long deltaTimeToObstacle = state.now - state.ringScanClosestDistanceTs;
  updateNextTime(&state.ringScanObstacleTime, deltaTimeToObstacle);

  Serial.print(" spinning back for (ms): "); Serial.print(deltaTimeToObstacle);
  Serial.println("");

  // invert spin from direction used by scan, so we can go back to where enemy is
  state.leftWheelDirection = opositeDirection(state.leftWheelDirection);
  state.rightWheelDirection = opositeDirection(state.leftWheelDirection);
  setWheelSpeed(wheelLeft, state.leftWheelDirection, state.leftWheelSpeed);
  setWheelSpeed(wheelRight, state.rightWheelDirection, state.rightWheelSpeed);
  
  state.robotMode = robotModeFaceObstacle;
}

// ----

void checkFaceObstacle() {
  if (state.robotMode != robotModeFaceObstacle) return;

  if (state.ringScanObstacleTime > state.now) return;  // still spinning to face obstacle

  // charge!
  state.leftWheelDirection = 
    state.rightWheelDirection = directionForward;
  setWheelSpeed(wheelLeft, state.leftWheelDirection, state.leftWheelSpeed);
  setWheelSpeed(wheelRight, state.rightWheelDirection, state.rightWheelSpeed);

  state.ringScanClosestDistance = ping_cm_BugFix();
  Serial.print("Facing obstacle at (cm): "); Serial.print(state.ringScanClosestDistance);
  Serial.println("");

  state.robotMode = robotModeAttacking;
}

// ----

void checkAttackProgress() {
  if (state.robotMode != robotModeAttacking) return;
  static const int attackDistanceMaxDev = 20; // cm

  const int pingSnapshot = ping_cm_BugFix();

  if (pingSnapshot != 0 && pingSnapshot >= (state.ringScanClosestDistance + attackDistanceMaxDev)) {
    Serial.print("Target distance increased to (cm): "); Serial.print(pingSnapshot);
    Serial.println(" attack is stopping");
    setStateNoop();
    return;
  }

  state.ringScanClosestDistanceTs = state.now;
  state.ringScanClosestDistance = pingSnapshot;

  Serial.print("Target being attacked tracked at (cm): "); Serial.print(pingSnapshot);
  Serial.println("");

  // continue attacking!
}

// ----

void setStateNoop() {
  state.leftWheelSpeed = 
    state.rightWheelSpeed = SPEED_MIN;
  setWheelSpeed(wheelLeft, state.leftWheelDirection, state.leftWheelSpeed);
  setWheelSpeed(wheelRight, state.rightWheelDirection, state.rightWheelSpeed);
  state.robotMode = robotModeNoop;
}

// ----

void setWheelSpeed(Wheel wheel, Direction direction, int speed)
{
  int servoValue;

  if (wheel == wheelLeft) {
    if (direction == directionForward) {
      servoValue = map(speed, SPEED_MIN, SPEED_MAX, LEFT_WHEEL_STOP_VALUE, 180);
    } else {
      servoValue = map(speed, SPEED_MIN, SPEED_MAX, LEFT_WHEEL_STOP_VALUE, 0);
    }

    Serial.print("left wheel set to: "); Serial.print(servoValue);
    Serial.print(" direction: "); Serial.print(direction == directionForward ? "fwd" : "back");
    Serial.print(" speed: "); Serial.print(speed);
    Serial.println("");
    leftWheel.write(servoValue);
  } else {
    if (direction == directionForward) {
      servoValue = map(speed, SPEED_MIN, SPEED_MAX, RIGHT_WHEEL_STOP_VALUE, 0);
    } else {
      servoValue = map(speed, SPEED_MIN, SPEED_MAX, RIGHT_WHEEL_STOP_VALUE, 180);
    }
    Serial.print("right wheel set to: "); Serial.print(servoValue);
    Serial.print(" direction: "); Serial.print(direction == directionForward ? "fwd" : "back");
    Serial.print(" speed: "); Serial.print(speed);
    Serial.println("");
    rightWheel.write(servoValue);
  }

}


// ----

// This is a wrapper function that tries to avoid a bug with the
// HC-SR04 modules where they can get stuck return zero forever
int ping_cm_BugFix() {
  // static unsigned long lastPingTs = state.now;
  // static int lastDistCm = 200;

  // if (state.now < (lastPingTs + 500)) {
  //   return lastDistCm;
  // }

  int distCm = sonar.ping_cm();
  if (distCm == 0) {
    delay(100);
    pinMode(ECHO_PIN, OUTPUT);
    digitalWrite(ECHO_PIN, LOW);
    delay(100);
    pinMode(ECHO_PIN, INPUT);
  }

  // lastPingTs = state.now;
  // lastDistCm = distCm;

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
#endif
