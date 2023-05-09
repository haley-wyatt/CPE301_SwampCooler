/*
 *Wyatt Haley
 *CPE301 Spring 2023
 *Final Project- Swamp Cooler
 *Instructors Bashira Anima and Shawn Ray
 */

/* Library Includes */
#include <dht_nonblocking.h> //Humidity and Temperature sensor
#include <LiquidCrystal.h> // LCD Display

/* Macro defines and global variables*/

/* Humidity and temperature sensor */
#define DHT_SENSOR_TYPE DHT_TYPE_11
static const int DHT_SENSOR_PIN = 2;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );
/* Humidity and temperature sensor */

/* Water level detection*/
const int sensorPin= 0; //sensor pin connected to analog pin A0
int liquid_level;
/* Water level detection*/

/*
 * Poll for a measurement, keeping the state machine alive.  Returns
 * true if a measurement is available.
 */

static bool measure_environment( float *temperature, float *humidity )
{
  static unsigned long measurement_timestamp = millis( );

  /* Measure once every four seconds. */
  if( millis( ) - measurement_timestamp > 3000ul )
  {
    if( dht_sensor.measure( temperature, humidity ) == true )
    {
      measurement_timestamp = millis( );
      return( true );
    }
  }

  return( false );
}
void setup() {
  Serial.begin(9600); //sets the baud rate for data transfer in bits/second

  /* Water level detection*/
  pinMode(sensorPin, INPUT);//the liquid level sensor will be an input to the arduino
  /* Water level detection*/
}

void loop() {
  /* Water level detection*/
  liquid_level= analogRead(sensorPin); //arduino reads the value from the liquid level sensor
  Serial.println(liquid_level);//prints out liquid level sensor reading
  delay(100);//delays 100ms
  /* Water level detection*/
}