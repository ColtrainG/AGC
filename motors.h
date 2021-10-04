// Colton Guillotte
// Autonomous Gardening Cart
// motor layout viewed from top
//     front
// L M1     R M2

// sabertooth dip switch:
// Packetized serial
// ON:         4 5 6
// OFF:  1 2 3

// -127 for full CounterClockwise, 0 for stop, +127 for full Clockwise
//ST.motor(1,0);   //sets power of specific motor
//ST.motor(2,0);

#ifndef MOTORS_H
#define MOTORS_H

#include <SoftwareSerial.h>
#include <Sabertooth.h>

void setupMotors();
void setupLift();

SoftwareSerial SWSerial(ST_RX_PIN, ST_TX_PIN); // RX on pin 3 (to S2), TX on pin 2 (to S1).
Sabertooth ST(128, SWSerial); // Address 128, and use SWSerial as the serial port.

int user_setSpeed = 0; // 0-127
int ForwardReverse_power = 0; // -127 for full reverse, 0 for stop, +127 for full forward.
int LeftRight_power = 0;
float manual_offset = 0.0;

// Master Enable
bool enabled = false;

namespace motors
{
  void setupMotors()
  {
    SWSerial.begin(9600); // serial connection to Sabertooth
    ST.drive(0); // Send command to stop transldational motion
    ST.turn(0); // Send command to stop rotational motion

    ST.setTimeout(20000); // A value of 0 disables the serial timeout. Useful for safety   
                           // If the S1 wire gets cut for some reason, or if your program crashes,
                           // the Sabertooth will stop receiving commands from the Arduino.
    //ST.setMinVoltage(10);  //Shutdown if minimum voltage detected for safety
    Serial.print("motors set\n");
  }

  void setupLift()
  {
    pinMode(LIFT_LPWM_PIN, OUTPUT);
    pinMode(LIFT_RPWM_PIN, OUTPUT);
    pinMode(LIFT_R_EN_PIN, OUTPUT);
    pinMode(LIFT_L_EN_PIN, OUTPUT);
  }
}

/////////////////////// FOLLOW MODE////////////////////////
BLYNK_WRITE(V1) 
{
  enabled = !enabled;
}

/////////////////////// USER SLIDE BUTTON////////////////////////
BLYNK_WRITE(V4)
{
  user_setSpeed = param.asInt(); //0-127 auto speed
}

/////////////////////// JOYSTICK CONTROLS////////////////////////
BLYNK_WRITE(V5)
{
  if(enabled == false) //autonomous mode disabled
  {
    int x = param[0].asInt();  // positive is turn left, neg is turn right
    int y = param[1].asInt();  // positive is forward, neg is reverse
    
    if((valSens1 <= 76 && valSens1 != 0) || (valSens2 <= 76 && valSens2 != 0)) //76cm away form object stop
    {
      if(y>0)
        ST.stop();
      else if(y<0)
        ST.drive(y);    
    } 
    x = (x/(2.6+manual_offset)); //scaled for slower turn
    y = (y/(2.0+manual_offset)); //scaled for slower drive
    ST.turn(x);
    ST.drive(y);
  }
}

BLYNK_WRITE(V6)
{
  manual_offset = param.asFloat(); //offset 0.0-2.0
}

/////////////////////// LIFT CONTROLS////////////////////////
BLYNK_WRITE(V8) //raise
{
  if(param.asInt())
  {
    analogWrite(LIFT_R_EN_PIN, 255);
    digitalWrite(LIFT_LPWM_PIN, LOW);
    digitalWrite(LIFT_RPWM_PIN, HIGH);
  }
  else
  {
    //analogWrite(LIFT_R_EN_PIN, 255);
    digitalWrite(LIFT_LPWM_PIN, LOW);
    digitalWrite(LIFT_RPWM_PIN, LOW);
  }
}
BLYNK_WRITE(V9) //lower
{
  if(param.asInt())
  {
    analogWrite(LIFT_L_EN_PIN, 255);
    digitalWrite(LIFT_LPWM_PIN, HIGH);
    digitalWrite(LIFT_RPWM_PIN, LOW);
  }
  else
  {
    //analogWrite(LIFT_R_EN_PIN, 255);
    digitalWrite(LIFT_LPWM_PIN, LOW);
    digitalWrite(LIFT_RPWM_PIN, LOW); 
  }
}


#endif
