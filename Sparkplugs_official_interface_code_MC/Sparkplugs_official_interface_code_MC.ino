/* 
*  Mitchell Chunn
*  ECE 1013 - Rep_Tracker project
*  User interface subsystem
* 
*  Project Notes: 
*  HeartRate Subsystem fully integrated. 
*  Working on integrating Accelorometer now
*  Code up to date as of april 8th
*  Pin Assignments:
*  User Interface: LCD: Digital Pin: 0-5
*  Accelerometer: Digital Pins 8 + 9. Analog Pins: A4 + A5 
*  Heartbeat Sensor: SDA + SSL
*  RTC Clock Module: Analog Pins: A4 + A5
*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include "MAX30105.h"
#include "heartRate.h"

Adafruit_MPU6050 mpu;
MAX30105 particleSensor;

//heartrate definitions below
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

//Accellorometer definitions below
//used for set and rep incrementing logic
int rep = 0, motionDet = 0, set = 0;
// stores the time of the last interrupt
unsigned long lastInterruptTime = 0;
// define a time threshold (in ms)
const unsigned long TIME_THRESHOLD_REP = 2000; // 500 = 0.5
const unsigned long TIME_THRESHOLD_SET = 5000;
unsigned long currentTime;
unsigned long timeSinceLastInterrupt;


float beatsPerMinute; //Heartrate definitions
int beatAvg;          //

const int rs = 0, en = 1, d4 = 2, d5 = 3, d6 = 4, d7 = 5;    // initialize the library by associating any needed LCD interface pin
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                   // with the arduino pin number it is connected to

void setup() {
   //LCD Display setup
  lcd.begin(16, 2);            // define/set the LCD's number of columns and rows:
  lcd.print("# of reps:");    // Print a message to the LCD.
  lcd.setCursor(12, 0);       // change LCD print location
  lcd.print("Set:");

  //Accelorometer initialization code below
  Serial.begin(9600);

   while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(4);
  mpu.setMotionDetectionDuration(500);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(200);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);

  //Initialize Heartrate sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  //Heartrate code end
}

void loop() {
//Accelorometer Code and output below
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
// get current time
   currentTime = millis();
    // calculate time since last interrupt
  if ((currentTime - lastInterruptTime) >= TIME_THRESHOLD_SET && rep != 0)
    {
      set++;
      rep = 0;
    }

  if(mpu.getMotionInterruptStatus()) {
    //Get new sensor events with the readings
    // Calculate time since last interrupt
    timeSinceLastInterrupt = currentTime - lastInterruptTime;
    
    // Condition: only proceed if enough time has passed since last interrupt
   if ( ((a.acceleration.z<=-5.0) || (a.acceleration.z>=11.0))) // NOTE: In final code, we need to use the x and y rather than z axis (or just the y axis)
     {
       // Update the last interrupt time
       motionDet++;
       if (motionDet == 1)
       { rep++;
         motionDet = 0;
       }
 
       lastInterruptTime = currentTime;
     rep++;
     /* Get new sensor events with the readings */
     sensors_event_t a, g, temp;
     mpu.getEvent(&a, &g, &temp);
 
     /*if (a.acceleration.x>0 && a.acceleration.y>0 && a.acceleration.z>0){
     repCount ++;
     }

    /* Print out the values */
    Serial.print("AccelX:");
    Serial.print(a.acceleration.x);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(a.acceleration.y);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(a.acceleration.z);
    Serial.print(", ");
    Serial.print("GyroX:");
    Serial.print(g.gyro.x);
    Serial.print(",");
    Serial.print("GyroY:");
    Serial.print(g.gyro.y);
    Serial.print(",");
    Serial.print("GyroZ:");
    Serial.print(g.gyro.z);
    Serial.println("");

    /* motionDet ++;
    if (motionDet == 2){
      rep ++;
    motionDet = 0;
    }
    */
   

     // only proceed if enough time has passed since last interrupt (FOR SETS COUNTED LOGIC)
    /*if (timeSinceLastInterrupt >= TIME_THRESHOLD_SET) {
      // update the last interrupt time
      lastInterruptTime = currentTime;
    }
    */
    
    Serial.print("Reps Performed: ");
    Serial.print(rep);
    Serial.println("");

    Serial.print("Current Set: ");
    Serial.print(set+1);
    Serial.println("");

    lcd.setCursor(0, 1);// set the cursor to column 0, line 1 (note: line 1 is the second row, since counting begins with 0):
    lcd.print(rep); // reps is a placeholder for the rep variable
  }
  delay(10);
  }

//Heartrate code and output below
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
  
  Serial.print("IR=");        //all serial print code will be uneccesary for final code
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000){         //if there is no finger on the sensor reset beatAvg to zero
   if(beatAvg > 0){
    lcd.setCursor(13, 1);
    lcd.print("   ");
   }
    Serial.print(" No finger?");
    beatAvg = 0;
    lcd.setCursor(12, 0);
    lcd.print("Set:");
    lcd.setCursor(13, 1);
    lcd.print(set);
  }

  if (irValue > 50000){
    lcd.setCursor(12, 0);
    lcd.print(" HR:");
    lcd.setCursor(13, 1);
    lcd.print(beatAvg);
  }
    
  if (beatAvg < 100){
    lcd.setCursor(15, 1);
    lcd.print(" ");
  }
Serial.println();
}