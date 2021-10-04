// Colton Guillotte
// Autonomous Gardening Cart

//left ultrasonic sensor = sensor1
//right ultrasonic sensor = sensor2
// layout viewed from top
//     front
// L S1     R S2

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <NewPing.h>

void ultrasonicDist();

//void manual_avoid();
int valSens1 = 0;
int valSens2 = 0;

NewPing sensor1(5,4,120); //100cm max distance
NewPing sensor2(7,6,120);

namespace ultrasonic
{
   void ultrasonicDist()
   {
      valSens1 = sensor1.ping_cm(); //read ultrasonic sensor 1
      valSens2 = sensor2.ping_cm(); //read ultrasonic sensor 2
   }
}


//const int pingPin = ; // Trigger Pin of Ultrasonic Sensor
//const int echoPin = ; // Echo Pin of Ultrasonic Sensor
   
   /*long duration, inches;
   pinMode(pingPin, OUTPUT);
   digitalWrite(pingPin, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(pingPin, LOW);
   pinMode(echoPin, INPUT);
   duration = pulseIn(echoPin, HIGH);
   inches = microsecondsToInches(duration);
   Serial.print(inches);
   Serial.print("in, ");
   Serial.println();
   delay(100);*/

 
/* 
long microsecondsToInches(long microseconds) 
{
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) 
{
   return microseconds / 29 / 2;
}
*/

/*right_sensor()
{

}

left_sensor()
{

}*/

//avoid_main_fix_distance()

/*void manual_avoid()
{
   long duration = pulseIn(echoPin, HIGH);
   long inches = duration / 74 / 2;

   if(inches <= 24)
      ST.stop();


}*/

#endif
