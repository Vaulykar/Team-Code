/* 
*  Mitchell Chunn
*  ECE 1013 - Rep_Tracker project
*  User interface subsystem
* 
*  Project Notes: 
*  Code up to date as of april 9th 1:03PM
*  HeartRate Subsystem fully integrated. 
*  Working on integrating Accelorometer power system and rtc simultaniously
*  sir RITZ please check lines 49, 50 & 51 once you are done updating the accelorometer code using the rtc
*  
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
#include <RTClib.h>           // DS1307 RTC library

Adafruit_MPU6050 mpu;
MAX30105 particleSensor;
RTC_DS1307 rtc;

//Heartrate definitions below
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute; 
int beatAvg;   

/*Accellorometer definitions below*/
//used for set and rep incrementing logic
int rep = 0, motionDet = 0, set = 0;
// stores the time of the last interrupt
unsigned long lastInterruptTime = 0;
// define a time threshold (in ms)
const unsigned long TIME_THRESHOLD_REP_MIN = 1000; // 500 = 0.5
const unsigned long TIME_THRESHOLD_SET = 15000; // The time before a set has ended (increase after testing is complete)
const float ACCEL_THRESHOLD_LOW = -9.0;
const float ACCEL_THRESHOLD_HIGH = 11.0;
unsigned long currentTime;
unsigned long timeSinceLastInterrupt;
unsigned timeDel; // These definitions were added by MC so it would compile
unsigned lastTime;//
unsigned slowRep; //

//Power System definitions
const int batteryPin = A0;  // Analog pin to read battery voltage
const int redPin = 9;       // Red LED pin (PWM)
const int greenPin = 10;    // Green LED pin (PWM)
const int bluePin = 11;     // Blue LED pin (not used in this case)
const float referenceVoltage = 5.0;  // Arduino operating voltage

//RTC definitions
unsigned long restStartTime = 0;
bool isResting = false;
const int REST_DURATION = 120; // 2 minutes in seconds
//bool movementDetected = false; // Placeholder for accelerometer input
//int setCounter = 0;            // Tracks completed sets

//Display Definitions
const int rs = 0, en = 1, d4 = 2, d5 = 3, d6 = 4, d7 = 5;    // initialize the library by associating any needed LCD interface pin
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                   // with the arduino pin number it is connected to

// Optional: Scan I2C devices
void scanI2C() {
  byte error, address;
  int count = 0;

  Serial.println("Scanning I2C devices...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      count++;
    }
  }
  if (count == 0)
    Serial.println("No I2C devices found.");
  else
    Serial.println("I2C scan complete.");
}

void setup() {
 Serial.begin(9600);
 Wire.begin();
  delay(1000); // Ensure peripherals are ready, comment this out if takes too long
  
  scanI2C(); // Debugging: Confirm I2C devices

  //Initialize Heartrate sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
   {
    //Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
   }
   //Serial.println("Place your index finger on the sensor with steady pressure.");

   particleSensor.setup(); //Configure sensor with default settings
   particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
   particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

   //Serial.println("Adafruit MPU6050 test!");

   // Try to initialize Accelerometer
   if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
   }
   //Serial.println("MPU6050 Found!");

   //setup motion detection
   mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
   mpu.setMotionDetectionThreshold(1);               // Change these valies for motion detection sensitivity
   mpu.setMotionDetectionDuration(500);              //
   mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
   mpu.setInterruptPinPolarity(true);
   mpu.setMotionInterrupt(true);

   //Serial.println("");
   delay(100);

  //Power system setup
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  // Initialize RTC 
  
   if (!rtc.begin()) {
    //Serial.println("RTC not found!");
    while (1);
   }
   if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set RTC to compile time
   }

//Setup LCD Display 
  lcd.begin(16, 2);            
  lcd.print("# of reps:");    
  lcd.setCursor(12, 0);       
  lcd.print("Set:");
}

void loop() {
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

 /*
  //Serial.print("IR=");        //all serial print code will be uneccesary for final code
  //Serial.print(irValue);
  //Serial.print(", BPM=");
  //Serial.print(beatsPerMinute);
  //Serial.print(", Avg BPM=");
  //Serial.print(beatAvg);
 */
  if (irValue < 50000){         //if there is no finger on the sensor reset beatAvg to zero
   if(beatAvg > 0){
    lcd.setCursor(13, 1);
    lcd.print("   ");
   }
    //Serial.print(" No finger?");
    beatAvg = 0;
    lcd.setCursor(12, 0);
    lcd.print("Set:");
    lcd.setCursor(13, 1);
    lcd.print(set+1);
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
  //Serial.println();

// RTC Code below
currentTime = millis();
 DateTime now = rtc.now();
 if (isResting) {
    unsigned long elapsedTime = now.unixtime() - restStartTime;
    if (elapsedTime < REST_DURATION) {
      int remainingSeconds = REST_DURATION - elapsedTime;
     // Serial.print("Resting | Set: ");
     // Serial.print(set + 1);
     // Serial.print(" | Time Left: ");
     // Serial.print(remainingSeconds / 60);
     // Serial.print(":");
      if ((remainingSeconds % 60) < 10) Serial.print("0");
      //Serial.println(remainingSeconds % 60);
    } else {
     // Serial.println("Rest done! Get back to work!");
      isResting = false;
      restStartTime = 0;
      delay(2000);
    }
    delay(100);
    return;
  }

 if (rep > 0 && (timeSinceLastInterrupt) >= TIME_THRESHOLD_SET) {
    set++;
    rep = 0;
    isResting = true;
    restStartTime = now.unixtime();
    //Serial.print("New set started: ");
    //Serial.println(set + 1);
    return;
  }

//Accelorometer Code and output below
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
 // get current time
   
    // calculate time since last interrupt
 if (rep > 0 && (currentTime - lastInterruptTime) >= TIME_THRESHOLD_SET) {
    set++;
    rep = 0;
    isResting = true;
    restStartTime = now.unixtime();
    //Serial.print("New set started: ");
    //Serial.println(set + 1);
    return;
  }

  if (mpu.getMotionInterruptStatus()) {
    unsigned long timeSinceLastInterrupt = currentTime - lastInterruptTime;

    // Extra check to verify actual motion (debounce false positives)
    mpu.getEvent(&a, &g, &temp);
    float z = a.acceleration.z;

    if (((z <= ACCEL_THRESHOLD_LOW || z >= ACCEL_THRESHOLD_HIGH) &&
         timeSinceLastInterrupt >= TIME_THRESHOLD_REP_MIN) && slowRep >= 20) {
      rep++;
      lastInterruptTime = currentTime;
      if (slowRep<20 && z > ACCEL_THRESHOLD_LOW && z < ACCEL_THRESHOLD_HIGH ) {
        slowRep++;
        lastInterruptTime = currentTime;

        //Serial.print("Rep: ");
        //Serial.print(rep);
        //Serial.print(" | Set: ");
        //Serial.println(set + 1);
      }
    }
    else if ( slowRep >= 20)
    rep++;
    slowRep = 0;
    } 

     
     /* Get new sensor events with the readings */
     sensors_event_t a, g, temp;
     mpu.getEvent(&a, &g, &temp);
 
     /*if (a.acceleration.x>0 && a.acceleration.y>0 && a.acceleration.z>0){
     repCount ++;
     */
     }

    /* Print out the values */
    //Serial.print("AccelX:");
    //Serial.print(a.acceleration.x);
    //Serial.print(",");
    //Serial.print("AccelY:");
    //Serial.print(a.acceleration.y);
    //Serial.print(",");
    //Serial.print("AccelZ:");
    //Serial.print(a.acceleration.z);
    //Serial.print(", ");
    //Serial.print("GyroX:");
    //Serial.print(g.gyro.x);
    //Serial.print(",");
    //Serial.print("GyroY:");
    //Serial.print(g.gyro.y);
    //Serial.print(",");
    //Serial.print("GyroZ:");
    //Serial.print(g.gyro.z);
    //Serial.println("");

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
    
    //Serial.print("Reps Performed: ");
    //Serial.print(rep);
    //Serial.println("");

    //Serial.print("Current Set: ");
    //Serial.print(set+1);
    //Serial.println("");
    }
    lcd.setCursor(0, 1);// set the cursor to column 0, line 1 (note: line 1 is the second row, since counting begins with 0):
    lcd.print(rep); // reps is a placeholder for the rep variable
  if (rep < 10){
  lcd.setCursor(1, 1);
   lcd.print("  ");
  }

//Power System Code below
  int rawValue = analogRead(batteryPin); // Read the analog input
  float batteryVoltage = (rawValue / 1023.0) * referenceVoltage; // Convert to voltage

  //Serial.print("Battery Voltage: ");
  //Serial.println(batteryVoltage);

  // Change LED color based on voltage level
  if (batteryVoltage >= 4.0) {
    analogWrite(redPin, 0);   // Red off
    analogWrite(greenPin, 255); // Green on
  }
  else if (batteryVoltage > 3.0 && batteryVoltage < 4.0) {
    analogWrite(redPin, 170);
    analogWrite(greenPin, 85);
  }
  else {
    analogWrite(redPin, 255);
    analogWrite(greenPin, 0);
  }
  analogWrite(bluePin, 0); // Blue remains off
}