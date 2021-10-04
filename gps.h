// Colton Guillotte
// Autonomous Gardening Cart

#ifndef GPS_H
#define GPS_H

#include <NMEAGPS.h>
#include <GPSport.h>

//prototype statements
float getDistance(struct waypoint, struct waypoint);
float getBearing(struct waypoint, struct waypoint);

float distance = 0.0;
float bearing = 0.0;

static NMEAGPS gps; // This parses the GPS characters
static gps_fix fix; // object that stores GPS data

waypoint user; //struct to store user position in int32
waypoint cart; //struct to store cart position in int32

//NeoGPS::Location_t user_loc(user.lat, user.lon);
  
//  Print the 32-bit integer degrees *as if* they were high-precision floats
/*static void printL( Print & outs, int32_t degE7 );
static void printL( Print & outs, int32_t degE7 )
{
  // Extract and print negative sign
  if (degE7 < 0) {
    degE7 = -degE7;
    outs.print( '-' );
  }

  // Whole degrees
  int32_t deg = degE7 / 10000000L;
  outs.print( deg );
  outs.print( '.' );

  // Get fractional degrees
  degE7 -= deg*10000000L;

  // Print leading zeroes, if needed
  int32_t factor = 1000000L;
  while ((degE7 < factor) && (factor > 1L)){
    outs.print( '0' );
    factor /= 10L;
  }
  
  // Print fractional degrees
  outs.print( degE7 );
}*/

void setupGPS()
{
  while (!Serial);

  Serial.print( F("GPS setup...\n") );
  Serial.print( F("NMEAGPS object size = ") );
  Serial.println( sizeof(gps) );
  Serial.println( F("CONNECT BLUETOOTH NOW, GPS port: " GPS_PORT_NAME) );

  //Serial.flush();
}

static void getFix();//protocol
static void getFix()
{
  if (fix.valid.location) 
  {
    cart.lat = fix.latitudeL();
    cart.lon = fix.longitudeL();
    //Serial.print(cart.lat);
    //Serial.println("\nC POS");
    //Serial.print(cart.lon);
    Serial.print("\nSatN: ");
    Serial.print(fix.satellites);
    //DEBUG_PORT.print( fix.latitude(), 6 ); // floating-point display
    //DEBUG_PORT.print( fix.latitudeL() ); // integer display
    //printL( DEBUG_PORT, fix.latitudeL() ); // prints int like a float

    distance = getDistance(user, cart);
    bearing = getBearing(user, cart);
    /*Serial.print(distance, 6);
    Serial.println( F(" m") );
    Serial.print(bearing, 6);
    Serial.println( F(" deg") );*/
  } 
  else 
  {
    // No valid location data yet!
    Serial.print( '?' ); // maybe add motor stop so cart doesn't move unless valid location
  }
  Serial.println();
}

static void GPSloop(); //protocol
static void GPSloop()
{
  cart.lat = 0;
  cart.lon = 0;
  while (gps.available(gpsPort)) // if while body skipped then what?
  {
    fix = gps.read();
    getFix();
  }
}
/*
// NeoGPS functions
float getDistance()
{
  float dist = fix.location.DistanceMiles(user_loc); //distance user/cart
  //Serial.print( dist );
  //Serial.println( F(" mi") );
  return dist;
}

float getBearing()
{
  float bearing = fix.location.BearingToDegrees(user_loc); //bearing user/cart
  //Serial.print( bearing );
  //Serial.println( F(" rads") );
  return bearing;
}*/

float getDistance(struct waypoint a, struct waypoint b) 
{ 
    /*printL(Serial, a.lat); // prints int like a float
    Serial.println();
    printL(Serial, b.lat); // prints int like a float
    Serial.println();
    printL(Serial, a.lon); // prints int like a float
    Serial.println();
    printL(Serial, a.lon); // prints int like a float
    Serial.println();*/
  
  //@brief: returns position between a and b in meters
  const int R = 6371000; //earth radius in meter
  int32_t p1 = a.lat * toRad;
  int32_t p2 = b.lat * toRad;
  int32_t dp = (b.lat-a.lat) * toRad;
  int32_t dl = (b.lon-a.lon) * toRad;

  float x = sin((dp/10000000L)/2) * sin((dp/10000000L)/2) + cos((p1/10000000L)) * cos((p2/10000000L)) * sin((dl/10000000L)/2) * sin((dl/10000000L)/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));
  return R * y;
}

float getBearing(struct waypoint a, struct waypoint b) 
{
    //Serial.print(user.lat);
    //Serial.println();
    //Serial.print(user.lon);
    //Serial.println();
  
  //@brief: returns the angle between (a,b) and (a,North) in Degree.
  int32_t aLat = a.lat * toRad; 
  int32_t aLon = a.lon * toRad;
  int32_t bLat = b.lat * toRad; 
  int32_t bLon = b.lon * toRad;
  
  float y = sin((bLon-aLon)/10000000L) * cos((bLat/10000000L));
  float x = cos((aLat/10000000L))*sin((bLat/10000000L)) - sin((aLat/10000000L))*cos((bLat/10000000L))*cos((bLon-aLon)/10000000L);
  return atan2(y,x) * toDeg;
}

//user GPS
BLYNK_WRITE(V2) 
{
    int32_t user_lat = 0;
    int32_t user_lon = 0;
    user_lat = (param[0].asDouble()*10000000L);
    user_lon = (param[1].asDouble()*10000000L);
    user.lat = user_lat;
    user.lon = user_lon;
    /*Serial.print(user.lat);
    Serial.println("\nU POS");
    Serial.print(user.lon);
    Serial.println();*/
}

#endif
