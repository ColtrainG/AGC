// Colton Guillotte
// Autonomous Gardening Cart

#ifndef CONSTANTS_H
#define CONSTANTS_H

#define BLYNK_USE_DIRECT_CONNECT
// Blynk Auth
char auth[] = "pOwmezhNXqp6kE9NA_MjnVhu2Y6rxwRz";

// pins 2,3,18,19,20,21 on mega for external interrupts
// Pin variables
#define ST_TX_PIN 2 //Arduino transmit pin 2 to sabertooth S1
#define ST_RX_PIN 3 //Arduino receive pin 3 to sabertooth S2

#define R_ECHO_PIN 4
#define R_TRIG_PIN 5
#define L_ECHO_PIN 6
#define L_TRIG_PIN 7

// If one motor tends to spin faster than the other, add offset
//#define MOTOR_OFFSET 5

#define BLUETOOTH_TX_PIN 16 //Serial2
#define BLUETOOTH_RX_PIN 17

#define GPS_TX_PIN 18 //Serial1
#define GPS_RX_PIN 19

//#define SDA 20
//#define SCL 21

//lift motor driver
#define LIFT_LPWM_PIN 8
#define LIFT_RPWM_PIN 9
#define LIFT_R_EN_PIN 10
#define LIFT_L_EN_PIN 11

// You must then add your 'Declination Angle' to the compass, which is the 'Error' of the magnetic field in your location.
// Find yours here: http://www.magnetic-declination.com/
// Mine is: 13° 24' E (Positive), which is ~13 Degrees, or (which we need) 0.23 radians
#define DECLINATION_ANGLE -0.0389f

// The offset of the mounting position to true north
// It would be best to run the /examples/magsensor sketch and compare to the compass on your smartphone
#define COMPASS_OFFSET 0.0f

// How often the GPS should update in MS
// Keep this above 1000
#define GPS_UPDATE_INTERVAL 1000

// Number of changes in movement to timeout for GPS streaming
// Keeps the cooler from driving away if there is a problem
#define GPS_STREAM_TIMEOUT 18
// Number of changes in movement to timeout for GPS waypoints
// Keeps the cooler from driving away if there is a problem
#define GPS_WAYPOINT_TIMEOUT 45

//const long loopTimeMicros = 20000;
//const long loopFrequency = 50;
//const float convFact = 0.0002282; // m/Inc
#define toRad  0.0174532925199432957f
#define toDeg  57.295779513082320876f
//const float velMax = 0.5; // m/s

// structure for lat. and long.
struct waypoint 
{
  int32_t lat;
  int32_t lon;
};

#endif
