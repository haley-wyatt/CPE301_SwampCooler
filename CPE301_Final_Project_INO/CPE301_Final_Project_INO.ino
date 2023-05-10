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
int liquid_level = 0;

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

volatile unsigned char *pin_h     = (unsigned char *) 0x100;// Port H Input Pins Register
volatile unsigned char *port_h    = (unsigned char *) 0x102; //Port H Data Register
volatile unsigned char *ddr_h     = (unsigned char *) 0x101;// Port H Data Direction Register

volatile unsigned char *pin_g     = (unsigned char *) 0x32;// Port G Input Pins Register
volatile unsigned char *port_g    = (unsigned char *) 0x34; //Port G Data Register
volatile unsigned char *ddr_g     = (unsigned char *) 0x33;// Port G Data Direction Register

/* Other variables */
#define RDA 0x80
#define TBE 0x20
#define liquid_min = 50;
#define liquid_max = 400;
char state = 'd';
/************************************************************/

void setup() {
  //set-up serial comms
  U0Init(9600);
  //set-up adc
  adc_init();
  //set-up timers
  setup_timer_regs();
  //set-up LCD display
  lcd.begin(16,2); // set-up LCD with 16 columns, 2 rows
  // set-up external clock
  clock.begin();
  clock.setDateTime(__DATE__, __TIME__);
  //set up GPIO
  *ddr_h |= 0b00001000; // PIN H3 - Digital 6 - set as OUTPUT
  *ddr_e |= 0b00001000; // PIN E3 - Digital 5 - set as OUTPUT
  *ddr_g |= 0b00100000; // PIN G5 - Digital 4 - set as OUTPUT
  *ddr_e &= 0b11011111; // PIN E5 - Digital 3 - set as INPUT
  *port_e |= 0b00100000;//PIN E5 - Digital 3 enable pull-up resistor
  *myEIMSK = 0b00100000; // enable interrupt on INT 5 - PIN E5 - Digital 3
  *myEICRB = 0b00001000; // ICS51 set to 1, ISC50 set to 0 - Falling edge samples
}

void loop() {
  switch(state){
    case 'd':
      //all led off
      display_disabled();
      turn_off_fan();
      break;
    case 'i':
      //green led
      display_dht();
      turn_off_fan();
      break;
    case 'r':
      //blue led
      display_dht();
      turn_on_fan();
      break;
    case 'e':
      //red led
      display_error();
      turn_off_fan();
      break;
  }
  liquid_level = adc_read(0);
  delay(100);
  if( (liquid_level < liquid_min) || (liquid_level > liquid_max) ){
    state = 'e';
  }
}
void print_time(){
  dt = clock.getDateTime();
  Serial.print("Time: ");
  Serial.print(dt.year);   Serial.print("-");
  Serial.print(dt.month);  Serial.print("-");
  Serial.print(dt.day);    Serial.print(" ");
  Serial.print(dt.hour);   Serial.print(":");
  Serial.print(dt.minute); Serial.print(":");
  Serial.print(dt.second); Serial.println("");
  //delay(1000);
}
void display_error(){
  lcd.setCursor(0,0);
  lcd.print("ERROR           ");
  lcd.setCursor(0, 2);
  lcd.print("                ");
}
void display_disabled(){
  lcd.setCursor(0,0);
  lcd.print("DISABLED        ");
  lcd.setCursor(0, 2);
  lcd.print("                ");
}
void display_dht(){
  float temperature;
  float humidity;
  if( measure_environment( &temperature, &humidity ) == true ){
      temperature = temperature * 9/5 + 32;
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

void turn_on_fan(){
  *port_h |= 0b00001000; // set PORT H3 - digital 6 - to high
  *port_e |= 0b00001000; // set PORT E3 - Digital 5 - to high
  *ddr_g &= 0b11011111; // set PORT G5 - Digital 4 -  to high
}
//INT 5 - PIN E5 - Digital 3
/* If button is pressed, start a timer */
ISR(INT5_vect){
    // Set count to 0
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
  //switch between disabled and idle states
  if(state == 'd'){
    state = 'i';
  }
  else if(state == 'i'){
    state = 'd';
  }
}

// Timer setup function
void setup_timer_regs(){
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

unsigned int adc_read(unsigned char adc_channel_num){
  // clear the channel selection bits (MUX 4:0)
  *myADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *myADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7){
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *myADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *myADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *myADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*myADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *myADCDR;
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

unsigned char U0getchar(){
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata){
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}
