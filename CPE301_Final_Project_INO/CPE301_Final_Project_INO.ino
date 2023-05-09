/*
  Wyatt Haley
  CPE301 Spring 2023
  Final Project- Swamp Cooler
  Instructors Bashira Anima and Shawn Ray
*/

/* Water level detection*/
const int sensorPin= 0; //sensor pin connected to analog pin A0
int liquid_level;
/* Water level detection*/

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