/* 
*  Mitchell Chunn
*  ECE 1013 - Rep_Tracker project
*  User interface subsystem
*
*  Project Notes: code up to date as of april 1st
*
*/

#include <Wire.h>
#include <LiquidCrystal.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

//heartrate definitions below
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

/*
byte heart[8] = {              //the bit assignments for the heart icon
  0b00000,
  0b01010,
  0b11111,
  0b11111,
  0b11111,
  0b01110,
  0b00100,
  0b00000
};
*/

int repititions = 12; // temporary variables used for testing purposes
float beatsPerMinute;
int beatAvg;

const int rs = 0, en = 1, d4 = 2, d5 = 3, d6 = 4, d7 = 5;    // initialize the library by associating any needed LCD interface pin
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                   // with the arduino pin number it is connected to

//function definition of LCD_Output
/*
void LCD_Output(int reps, int heartrate){
  lcd.setCursor(0,1);// set the cursor to column 0, line 1 (note: line 1 is the second row, since counting begins with 0):
  lcd.print(reps); // reps is a placeholder for the rep variable
  lcd.setCursor(13, 1);
  lcd.print(heartrate); // heartrate is a placeholder for the heartrate variable
}*/

void setup() {
   //display stuff below
  //lcd.createChar(8, heart);    // Load the custom character into CGRAM location 8
  lcd.begin(16, 2);            // define/set the LCD's number of columns and rows:
  lcd.print("# of reps:");    // Print a message to the LCD.
  lcd.setCursor(13, 0);       // change LCD print location
  //lcd.write(byte(8));         // output custom char: heart
  lcd.print("HR:");

//Heartrate 
  Serial.begin(115200);

  //Initialize sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void loop() {

long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);
    
    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
      beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000){
    Serial.print(" No finger?");
    beatAvg = 0;
    lcd.setCursor(14, 1);
    lcd.print("  ");
  }
  Serial.println();

  lcd.setCursor(0,1);// set the cursor to column 0, line 1 (note: line 1 is the second row, since counting begins with 0):
  lcd.print(repititions); // reps is a placeholder for the rep variable
  lcd.setCursor(13, 1);
  lcd.print(beatAvg); // heartrate is a placeholder for the heartrate variable
}