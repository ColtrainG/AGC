// Colton Guillotte
// Autonomous Gardening Cart

#ifndef COMPASS_H
#define COMPASS_H

#include <Wire.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor.h>

float getHeading();
void displaySensorDetails(void);
void setupCompass();

namespace compass
{

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified compass = Adafruit_LSM303_Mag_Unified(12345);

void setupCompass()
{
  // Enable auto-gain
  compass.enableAutoRange(true);

  // Initialise the sensor
  if(!compass.begin())
  {
    // There was a problem detecting the LSM303
    Serial.println("Oops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  Serial.print("compass set\n");
  //Serial.flush();
  
}

void displaySensorDetails(void) 
{
  sensor_t sensor;
  compass.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

float getHeading()
	{
		//@brief: returns the current heading and substracts 180 Deg because the compass is mounted with zero pointing to the back
		//return Compass.GetHeadingDegrees()-180.0;
    //Serial.println(F("compass head"));
    //Get a new sensor event
    sensors_event_t event; 
    compass.getEvent(&event);

    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float cal_heading = atan2(event.magnetic.y, event.magnetic.x);

    // Offset
    cal_heading -= DECLINATION_ANGLE;
    cal_heading -= COMPASS_OFFSET;
  
    // Correct for when signs are reversed.
    if(cal_heading < 0)
      cal_heading += 2*PI;
    
    // Check for wrap due to addition of declination.
    if(cal_heading > 2*PI)
      cal_heading -= 2*PI;
   
    // Convert radians to degrees for readability.
    float headingDegrees = cal_heading * 180/M_PI; 

    // Map to -180 - 180
    while (headingDegrees < -180) headingDegrees += 360;
    while (headingDegrees >  180) headingDegrees -= 360;
    
    return headingDegrees;
	}
}
#endif

/*void calibrateCompass() 
{
  Serial.print("Calibrating compass... ");
  
  memset( &compCalib, 0, sizeof(compCalib) );
  
  unsigned long startTime = millis();

  while ( millis() - startTime < 30000 ) {
    if ( getCompassRaw() ) {
      for (int i = 0; i < 3; i++) {
        compCalib.lower[i] = min( compCalib.lower[i], compassNow[i] );
        compCalib.upper[i] = max( compCalib.upper[i], compassNow[i] );
      }
    }
    delay(1);
  }  
  
  for (int i = 0; i < 3; i++) {
    compCalib.center[i] = 0.5 * compCalib.lower[i] + 0.5 * compCalib.upper[i];
  }
  for (int i = 0; i < 3; i++) {
    compCalib.scale[i] = (compCalib.upper[i] - compCalib.lower[i]) * 0.5;
  }
  float averageSpan = 0.3333 * compCalib.scale[0] + 0.3333 * compCalib.scale[1] + 0.3333 * compCalib.scale[2];
  for (int i = 0; i < 3; i++) {
    compCalib.scale[i] = averageSpan / compCalib.scale[i];
  }

  printCompassCalibrationValues();
  
  for ( int i = 0; i < (int)sizeof(compCalib); i++)
    EEPROM.write( i, ((byte*)&compCalib)[i] );
    
  Serial.println("done.");
}*/
