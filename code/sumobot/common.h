#ifndef _COMMON_H
#define _COMMON_H

// #define DEBUG 1

/*******************************************************
* Define statements - Define constants we will use later
*******************************************************/
#define TRIGGER_PIN       A0    // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN          A1    // Arduino pin tied to echo pin on the ultrasonic sensor.
#if 0
#define LEFT_WHEEL_PIN     9    // Arduino pin tied to the left servo wheel motor
#define RIGHT_WHEEL_PIN   10    // Arduino pin tied to the right servo wheel motor
#else
#define LEFT_WHEEL_PIN     4    // Arduino pin tied to the left servo wheel motor
#define RIGHT_WHEEL_PIN    5    // Arduino pin tied to the right servo wheel motor
#endif // #if 0
#define MAX_DISTANCE      200   // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define FRONT_EDGE_LEFT_SENSOR  A2    // Pin connected to the front edge IR sensor
#define FRONT_EDGE_RIGHT_SENSOR A3    // Pin connected to the front edge IR sensor
#define REAR_EDGE_LEFT_SENSOR   A4    // Pin connected to the rear edge IR sensor
#define REAR_EDGE_RIGHT_SENSOR  A5    // Pin connected to the rear edge IR sensor
#define LIGHT_COLOR_VALUE      512   // This is a value returned from the edge sensors when they see a lighter color, may need tweaking

#define LEFT_WHEEL_STOP_VALUE   86    // 0 back;   86 stop;    180 fwd
#define RIGHT_WHEEL_STOP_VALUE  88    // 0 fwd;    88 stop;    180 back


typedef struct {
  
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

