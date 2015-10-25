#ifndef _COMMON_H
#define _COMMON_H

#define DO_NOT_MOVE 1


/*******************************************************
* Define statements - Define constants we will use later
*******************************************************/
#define TRIGGER_PIN       A0    // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN          A1    // Arduino pin tied to echo pin on the ultrasonic sensor.

#ifdef DO_NOT_MOVE
#define LEFT_WHEEL_PIN     2
#define RIGHT_WHEEL_PIN    3
#else
#define LEFT_WHEEL_PIN     9    // Arduino pin tied to the left servo wheel motor
#define RIGHT_WHEEL_PIN   10    // Arduino pin tied to the right servo wheel motor
#endif // #ifdef DO_NOT_MOVE

#define MAX_DISTANCE      200   // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define MAX_TARGET_DISTANCE 40  // Anything beyond this target distance is to be ignored

#define FRONT_EDGE_LEFT_SENSOR  A2    // Pin connected to the front edge IR sensor
#define FRONT_EDGE_RIGHT_SENSOR A3    // Pin connected to the front edge IR sensor
#define REAR_EDGE_LEFT_SENSOR   A4    // Pin connected to the rear edge IR sensor
#define REAR_EDGE_RIGHT_SENSOR  A5    // Pin connected to the rear edge IR sensor
#define LIGHT_COLOR_VALUE      810   // This is a value returned from the edge sensors when they see a lighter color, may need tweaking

#define LEFT_WHEEL_STOP_VALUE   87    // 0 back;   87 stop;    180 fwd
#define RIGHT_WHEEL_STOP_VALUE  88    // 0 fwd;    88 stop;    180 back

#define RING_SCAN_TIME        3500    // milliseconds it takes for scanning for obstacles

typedef enum {
  wheelRight,
  wheelLeft,
} Wheel;

typedef enum {
  directionForward,
  directionBackward
} Direction;

typedef enum {
  robotModeNoop = 0,
  robotModeAvoidEdge,
  robotModeRingScan,
  robotModeFaceObstacle,
  robotModeAttacking,
  robotModeCount
} RobotMode;

#define SPEED_MIN 0
#define SPEED_MAX 10

#define SENSOR_REACT_INTERVAL_MS 250

typedef struct {
  unsigned long now; // aka  millis();

  Direction leftWheelDirection;
  int leftWheelSpeed;

  Direction rightWheelDirection;
  int rightWheelSpeed;

  unsigned long frontRightSensorTriggerTs;
  unsigned long frontLeftSensorTriggerTs;
  unsigned long rearRightSensorTriggerTs;
  unsigned long rearLeftSensorTriggerTs;

  Direction ringScanLeftDirection;          // spin direction used for enemy scan
  unsigned long ringScanClosestDistanceTs;  // ts when 'best' enemy was located
  int ringScanClosestDistance;         // distance to enemy, updated when scanning and when attacking
  unsigned long ringScanObstacleTime;  // timestamp till spin will get to enemy; or timestamp
                                       // scan for enemies is completed

  RobotMode robotMode;
  
  unsigned long nextFasttime;   // few milliseconds timer
  unsigned long next250time;    // 250 milliseconds timer
  unsigned long next500time;    // 500 milliseconds timer
  unsigned long next1000time;   // 1 second timer
  unsigned long next5000time;   // 5 second timer
  unsigned long next10000time;  // 10 second timer
  unsigned long next25000time;  // 25 second timer
  unsigned long next1mintime;   // 1 min timer
  unsigned long next5mintime;   // 5 min timer
  unsigned long next10mintime;  // 10 min timer
} State;

extern State state;

#endif // _COMMON_H

