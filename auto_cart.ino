// Colton Guillotte
// Autonomous Gardening Cart

// pins 2,3,18,19,20,21 on mega for external interrupts

#include "constants.h"
#include <HardwareSerial.h>
#include <BlynkSimpleSerialBLE.h>
#include "ultrasonic.h"
#include "motors.h"
#include "gps.h"
#include "compass.h"
#include <SPI.h>

void driveTo();

// Loop timing    
//long timeOld = 0;
//long dt = 0;
const float maxDist = 0.5; //meters
//int angleSetPoint = 0;
//int overflowCounter = 0;
float heading = 0.0;
//float headingOld = 0.0; //jump detection
//float bearingOld = 0.0; //jump detection
//float bearingSP = 0;
//float deltaHeading = 0.0;
//float deltaHeadingInt = 0.0; 

void setup() 
{
  // serial connection back to computer debugging
  Serial.begin(9600);
  Serial.print("\nStarting...\n");

  //GPS
  Serial1.begin(9600);
  setupGPS();
  
  //bluetooth
  Serial2.begin(9600);
  Blynk.begin(Serial2,auth);

  //cart motors and lift
  motors::setupMotors();
  motors::setupLift();

  //compass
  compass::setupCompass();
  //calibrateCompass();

  //best place to do your time-consuming work, right after
  //the RMC sentence was received.  If you do anything in "loop()",
  //you could cause GPS characters to be lost, and you will not get a good lat/lon.
  //For this example, we just print the lat/lon.  If you print too much,
  //this routine will not get back to "loop()" in time to process the next set of GPS data.
}

void loop() 
{
  GPSloop();
  Blynk.run();
  ultrasonic::ultrasonicDist();
  driveTo();
}

void driveTo()
{
  if((valSens1 <= 76 && valSens1 != 0) || (valSens2 <= 76 && valSens2 != 0)) //76cm away form object stop
  {
    ST.stop();
    return;
  }
  
  if (cart.lat != 0 && cart.lon != 0 && enabled == true) 
  {
    int stopSpeed = 0; //fullspeed set by user_setSpeed

    heading = compass::getHeading();
    float turn = bearing - heading;
      
    Serial.print("Dist: ");
    Serial.println(getDistance(user, cart));
    Serial.print("Bear: ");
    Serial.println(getBearing(user, cart));
    Serial.print("head: ");
    Serial.println(compass::getHeading());

    int s = user_setSpeed;
    if (distance < 3)
    {
      int wouldBeSpeed = s - stopSpeed;
      wouldBeSpeed *= (distance/3.0f);
      s = stopSpeed + wouldBeSpeed;
    }

    int autoThrottle = constrain(s, stopSpeed, user_setSpeed);
    autoThrottle = user_setSpeed;

    float t = turn;
    while (t < -180) t += 360;
    while (t >  180) t -= 360;

    float t_modifier = (180.0 - abs(t)) / 180.0;
    float autoSteerA = 1;
    float autoSteerB = 1;

    if (t < 0) 
    {
      autoSteerB = t_modifier;
    } 
    else if (t > 0)
    {
      autoSteerA = t_modifier;
    }

    int speedA = (int) (((float) autoThrottle) * autoSteerA);
    int speedB = (int) (((float) autoThrottle) * autoSteerB);

    ST.motor(1, speedA);   //sets speed of specific motor
    ST.motor(2, speedB);

    //timeout--;  
    if (distance < 2.0)
    {
      Serial.print("\nArrived at user!!!\n");
      ST.stop();
    }
  }
} 

/*void setSpeed()
{
  ForwardReverse_power = user_setSpeed;
  LeftRight_power = (user_setSpeed/1.5); //turn at less than drive speed
  if(distance < maxDist)
  {
    //position reached
    ForwardReverse_power = 0;  
    Serial.println("pos reached\n");
  }
  if(distance < (1.5*maxDist))
  {
    ForwardReverse_power = (ForwardReverse_power/1.5); //slow nearby the goal 70%
    LeftRight_power = (LeftRight_power/1.2);
  }
}*/

/*void calcAngleSetPoint()
{ //this function uses heading and bearing to calculate 
  //the anglesetPoint with an PI-Controller. This is the outer cascade of the direction controller 

  headingOld = heading;
  heading = compass::getHeading();

  //This is to handle the case when bearing and heading jump between -180 and 180 degree. 
  //Its necessery othewise the controller won't work when driving in southern direction

  if(headingOld < -90 && heading > 90){overflowCounter += 1;}  //jump from -180 to +180 
  if(headingOld > 90 && heading < -90){overflowCounter -= 1;}  //jump from +180 to -180
  if(bearingOld < -90 && bearing > 90){overflowCounter -= 1;}  //jump from -180 to +180 
  if(bearingOld > 90 && bearing < -90){overflowCounter += 1;}  //jump from +180 to -180 
  bearingSP = overflowCounter*360 + bearing;
  
  deltaHeading = bearingSP - heading ;
  if(deltaHeading < -89){deltaHeading = -89;} 
  if(deltaHeading > 89){deltaHeading = 89;}
  
  float uHeading = 1.0*deltaHeading + 0.05*deltaHeadingInt;
  deltaHeadingInt += deltaHeading;
  
  if(deltaHeading <= 0.05 && deltaHeading >= -0.05 || //anti-windup
     deltaHeading < 0.0 && deltaHeadingInt > 0.0 || 
     deltaHeading > 0.0 && deltaHeadingInt < 0.0)
  {deltaHeadingInt = 0.0;} 

  if(deltaHeadingInt>1000){ deltaHeadingInt = 1000; powerSetPoint = 50;}
  if(deltaHeadingInt<-1000){ deltaHeadingInt = -1000; powerSetPoint = 50;}
  
  if(uHeading<-89){uHeading = -89;} //only allow forward driving
  if(uHeading>89){uHeading = 89;}
  angleSetPoint = static_cast<int>(90.0 - uHeading + 0.5);   //1...179 and 90 means straight, 0.5 for rounding since always positive
}*/
