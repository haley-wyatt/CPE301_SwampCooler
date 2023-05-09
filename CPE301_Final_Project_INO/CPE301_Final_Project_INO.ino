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
int currentTicks = 0;
bool timer_running = 0;
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
volatile unsigned char *myEICRB = (unsigned char *) 0x6A; // external interrupt control register B pg 111
volatile unsigned char *myEIMSK = (unsigned char *) 0x3D; // external interrupt mask register pg 111

/* ADC */
volatile unsigned char *myADMUX = (unsigned char *) 0x7C; // ADC multiplexer selection register pg 281
volatile unsigned char *myADCSRA = (unsigned char *) 0x7A; // ADC control and status register A pg 285
volatile unsigned char *myADCSRB = (unsigned char *) 0x7B;
volatile unsigned int *myADCDR = (unsigned char *) 0x78; // ADC data register
/************************************************************/

void setup() {
  Serial.begin(9600); // sets the baud rate for data transfer in bits/second
  //U0Init(9600);
  setup_timer_regs(); // setup timer registers
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

// Timer setup function
void setup_timer_regs()
{
  // setup the timer control registers
  *myTCCR1A= 0x00;
  *myTCCR1B= 0X00;
  *myTCCR1C= 0x00;
  
  // reset the TOV flag
  *myTIFR1 |= 0x01; //0b 0000 0001
  
  // enable the TOV interrupt
  *myTIMSK1 |= 0b00000001;
}

// TIMER OVERFLOW ISR
ISR(TIMER1_OVF_vect)
{
  // Stop the Timer
  *myTCCR1B &= 0b11111000; //CSn2:0 set to 0 - no clock source
  // Load the Count
  *myTCNT1 =  (unsigned int) (65535 -  (unsigned long) (currentTicks));
  // Start the Timer
  *myTCCR1B |=   0b00000001; // no prescaler
  // if it's not the STOP amount
  if(currentTicks != 65535)
  {
    // XOR to toggle PB6
    *port_b ^= 0x40;
  }
}

// DHT measurement function
static bool measure_environment( float *temperature, float *humidity ){
  /* Poll for a measurement, keeping the state machine alive.  Returns
   true if a measurement is available.
  */
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

// ADC initialization function
void adc_init(){
  // setup A register
  *my_ADCSRA |= 0b10000000; // enable ADC
  *my_ADCSRA &= 0b11011111; // disable ADC auto trigger
  *my_ADCSRA &= 0b11110111; // disable ADC interrupt
  *my_ADCSRA &= 0b11111000; // prescaler set to division factor of 2
  // setup B register
  *my_ADCSRB &= 0b11110111; // mux5 0
  *my_ADCSRB &= 0b11111000; //ADC auto trigger free running mode
  // setup MUX register
  *my_ADMUX &= 0b01111111; // 1.1V voltage reference
  *my_ADMUX |= 0b01000000; // 1.1V voltage reference
  *my_ADMUX &= 0b11011111; // ADLAR to 0 - right justified
  //*my_ADMUX |= 0b00100000; // ADLAR to 1 - left justified
  *my_ADMUX &= 0b11100000; // MUX4:0 to 0 to reset channel
}

// serial set up function
void U0Init(int U0baud){
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1); // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20; //0b 0010 0000 - Transmitter for USART ready
 *myUCSR0B = 0x18; //0b 0001 1000 - Enable USART rx and tx
 *myUCSR0C = 0x06; //0b 0000 0110 - UCSZn2=0 UCSZn1=1 - 8bit char size for rx/tx
 *myUBRR0  = tbaud;
}