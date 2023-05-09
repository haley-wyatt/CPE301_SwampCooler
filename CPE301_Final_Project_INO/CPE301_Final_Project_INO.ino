/*
 *Wyatt Haley
 *CPE301 Spring 2023
 *Final Project- Swamp Cooler
 *Instructors Bashira Anima and Shawn Ray
 */

/******************Library Includes**************************/
#include <dht_nonblocking.h> //Humidity and Temperature sensor
#include <LiquidCrystal.h> // LCD Display
/************************************************************/

/*************Macro defines and global variables*************/
/* Humidity and temperature sensor */
#define DHT_SENSOR_TYPE DHT_TYPE_11
static const int DHT_SENSOR_PIN = 2;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );
/* Humidity and temperature sensor */

/* LCD */
LiquidCrystal lcd(7,8,9,10,11,12);
/* LCD */

/* Water level detection*/
const int sensorPin= 0; //sensor pin connected to analog pin A0
int liquid_level;
/* Water level detection*/

/* Timers */
volatile unsigned int  *myTCNT1   = (unsigned int *) 0x84; // Timer/Counter 1
volatile unsigned char *myTCCR1A  = (unsigned char *) 0X80; // Timer/Counter 1 Control Register A
volatile unsigned char *myTCCR1B  = (unsigned char *) 0X81; // Timer/Counter 1 Control Register B
volatile unsigned char *myTCCR1C  = (unsigned char *) 0X82;// Timer/Counter 1 Control Register C
// TIMSK - timer interrupt mask register
// bit 0 - TOIEn - timer/counter overflow interrupt enable(1) disable(0)
volatile unsigned char *myTIMSK1  = (unsigned char *) 0x6F;
// TIFR - timer interrupt flag register
// bit 0 - TOV - interrupt enable(1) disable (0)
volatile unsigned char *myTIFR1   = (unsigned char *) 0x36;
/* Timers */

/************************************************************/

/*
 * Poll for a measurement, keeping the state machine alive.  Returns
 * true if a measurement is available.
 */
static bool measure_environment( float *temperature, float *humidity ){
  static unsigned long measurement_timestamp = millis( );

  /* Measure once every four seconds. */
  if( millis( ) - measurement_timestamp > 3000ul ){
    if( dht_sensor.measure( temperature, humidity ) == true ){
      measurement_timestamp = millis( );
      return( true );
    }
  }
  return( false );
}

void setup() {
  Serial.begin(9600); //sets the baud rate for data transfer in bits/second

  /* LCD */
  lcd.begin(16,2); // LCD columns / rows
  /* LCD */

  /* Water level detection*/
  pinMode(sensorPin, INPUT); //the liquid level sensor will be an input to the arduino
  /* Water level detection*/

}

void loop() {

  /* Water level detection*/
  //liquid_level= analogRead(sensorPin); //arduino reads the value from the liquid level sensor
  //Serial.println(liquid_level);//prints out liquid level sensor reading
  //delay(100);//delays 100ms
  /* Water level detection*/

  /* Digital Humidity & Temperature Sensor */
  float temperature;
  float humidity;
  if( measure_environment( &temperature, &humidity ) == true ){
    temperature = temperature * 9/5 + 32; //Celsius to fahrenheit
    lcd.setCursor(0,0);
    lcd.print("T = ");
    lcd.print(temperature);
    lcd.print(" deg. F");
    lcd.setCursor(0,1);
    lcd.print("H = ");
    lcd.print(humidity);
    lcd.print("%");
  }
  /* Digital Humidity & Temperature Sensor */

}