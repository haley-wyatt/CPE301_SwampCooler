/*
 *Wyatt Haley
 *CPE301 Spring 2023
 *Final Project- Swamp Cooler
 *Instructors Bashira Anima and Shawn Ray
 */

/******************Library Includes**************************/
#include <dht_nonblocking.h> //Humidity and Temperature sensor
#include <LiquidCrystal.h> // LCD Display
#include <Wire.h> //External clock
#include <DS3231.h> // External clock
/************************************************************/

/*************Macro defines and global variables*************/
/* External Clock */
DS3231 clock;
RTCDateTime dt;

/* Humidity and temperature sensor */
#define DHT_SENSOR_TYPE DHT_TYPE_11
static const int DHT_SENSOR_PIN = 2;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

/* LCD */
LiquidCrystal lcd(7,8,9,10,11,12);

/* Water level detection*/
const int sensorPin= 0; //sensor pin connected to analog pin A0
int liquid_level;

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

/* Interrupts */
volatile unsigned char *myEICRB   = (unsigned char *) 0x6A; // external interrupt control register B pg 111
volatile unsigned char *myEIMSK   = (unsigned char *) 0x3D; // external interrupt mask register pg 111

/* ADC */
volatile unsigned char *myADMUX   = (unsigned char *) 0x7C; // ADC multiplexer selection register pg 281
volatile unsigned char *myADCSRA = (unsigned char *) 0x7A; // ADC control and status register A pg 285
volatile unsigned char *myADCSRB = (unsigned char *) 0x7B; // ADC control and status register A pg 285
volatile unsigned int *myADCDR   = (unsigned int *) 0x78; // ADC data register

/* UART */
volatile unsigned char *myUCSR0A  = (unsigned char *) 0x00C0;
volatile unsigned char *myUCSR0B  = (unsigned char *) 0x00C1;
volatile unsigned char *myUCSR0C  = (unsigned char *) 0x00C2;
volatile unsigned int  *myUBRR0   = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0    = (unsigned char *) 0x00C6;

/* GPIO */
volatile unsigned char *port_b    = (unsigned char *) 0x25;// Port B Data Register
volatile unsigned char *ddr_b     = (unsigned char *) 0x24;// Port B Data Direction Register

volatile unsigned char *pin_e     = (unsigned char *) 0x2C;// Port E Input Pins Register
volatile unsigned char *port_e    = (unsigned char *) 0x2E; //Port E Data Register
volatile unsigned char *ddr_e     = (unsigned char *) 0x2D;// Port E Data Direction Register

/* Other variables */ 
bool disabled = true;
bool idle = false;
/************************************************************/

void setup() {
  Serial.begin(9600); // sets the baud rate for data transfer in bits/second
  //U0Init(9600);

  *ddr_e &= 0b11011111; // PIN E5 - Digital 3 - set as input
  *port_e |= 0b00100000; // enable pull-up resistor
  *myEIMSK = 0b00100000; // enable interrupt on INT 5 - PIN E5 - Digital 3
  *myEICRB = 0b00001000; // ICS51 set to 1, ISC50 set to 0 - Falling edge samples
  
  setup_timer_regs(); // setup timer registers

  lcd.begin(16,2); // set-up LCD with 16 columns, 2 rows

   // set-up external clock
  clock.begin();
  clock.setDateTime(__DATE__, __TIME__);

  /* Water level detection*/
  pinMode(sensorPin, INPUT); //the liquid level sensor will be an input to the arduino

}

void loop() {

  if(!disabled){
    /* Clock 
    dt = clock.getDateTime();
    Serial.print("Time: ");
    Serial.print(dt.year);   Serial.print("-");
    Serial.print(dt.month);  Serial.print("-");
    Serial.print(dt.day);    Serial.print(" ");
    Serial.print(dt.hour);   Serial.print(":");
    Serial.print(dt.minute); Serial.print(":");
    Serial.print(dt.second); Serial.println("");
    delay(1000);
    */

    /* Water level detection*/
    //liquid_level= analogRead(sensorPin); //arduino reads the value from the liquid level sensor
    //Serial.println(liquid_level);//prints out liquid level sensor reading
    //delay(100);//delays 100ms

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
  }

  if(disabled){
    lcd.clear();
  }
}

//INT 5 - PIN E5 - Digital 3
/* If button is pressed, start a timer */
ISR(INT5_vect){
    // Set count
    *myTCNT1 = 0;
    // Start the timer
    *myTCCR1B |=   0b00000001; // no prescaler
    timer_running = true;
}

// TIMER OVERFLOW ISR
ISR(TIMER1_OVF_vect){
  // Stop the Timer
  *myTCCR1B &= 0b11111000; //CSn2:0 set to 0 - no clock source
  timer_running = false;
  disabled = !disabled;
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

// DHT measurement function
static bool measure_environment(float *temperature, float *humidity){
  /* Poll for a measurement, keeping the state machine alive.  Returns
   true if a measurement is available.
  */
  static unsigned long measurement_timestamp = millis( );
  /* Measure once every four seconds. */
  if( millis( ) - measurement_timestamp > 3000ul ){
    if( dht_sensor.measure( temperature, humidity ) == true ){
      measurement_timestamp = millis( );
      return(true);
    }
  }
  return(false);
}

// ADC initialization function
void adc_init(){
  // setup A register
  *myADCSRA |= 0b10000000; // enable ADC
  *myADCSRA &= 0b11011111; // disable ADC auto trigger
  *myADCSRA &= 0b11110111; // disable ADC interrupt
  *myADCSRA &= 0b11111000; // prescaler set to division factor of 2
  // setup B register
  *myADCSRB &= 0b11110111; // mux5 0
  *myADCSRB &= 0b11111000; //ADC auto trigger free running mode
  // setup MUX register
  *myADMUX &= 0b01111111; // 1.1V voltage reference
  *myADMUX |= 0b01000000; // 1.1V voltage reference
  *myADMUX &= 0b11011111; // ADLAR to 0 - right justified
  //*myADMUX |= 0b00100000; // ADLAR to 1 - left justified
  *myADMUX &= 0b11100000; // MUX4:0 to 0 to reset channel
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