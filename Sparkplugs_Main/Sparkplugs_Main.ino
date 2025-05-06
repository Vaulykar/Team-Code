/*  Team: The Sparkplugs
 *  Mitchell Chunn, Tyler Dunn, Joshua Lane, Jeremiah "Christian" Pope, Christopher Ritz
 *  
 *  ECE 1013 - Rep_Tracker project
 *  User interface subsystem
 * 
 *  Project Notes: 
 *  Code up to date as of April 25, 2025
 *  HeartRate Subsystem fully integrated. 
 *  Accelerometer and RTC integrated with 5-second inactivity trigger for 2-minute countdown
 *  
 *  Pin Assignments:
 *  User Interface: LCD: Digital Pin: 0-5
 *  Accelerometer: Digital Pins 8 + 9. Analog Pins: A4 + A5 
 *  Heartbeat Sensor: SDA + SCL
 *  RTC Clock Module: Analog Pins: A4 + A5
 */

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <RTClib.h>

Adafruit_MPU6050 mpu;
MAX30105 particleSensor;
RTC_DS1307 rtc;

// Heart rate definitions
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute; 
int beatAvg;   

// Accelerometer definitions
int rep = 0, motionDet = 0, set = 0;
unsigned long lastInterruptTime = 0;
const unsigned long TIME_THRESHOLD_REP_MIN = 600;
const unsigned long TIME_THRESHOLD_SET = 20000;
const float Y_ACCEL_THRESHOLD_LOW = -7.0;
const float Y_ACCEL_THRESHOLD_HIGH = 8.5;
unsigned long currentTime;
unsigned long timeSinceLastInterrupt;
unsigned long timeDel = 0;
unsigned long lastTime = 0;
bool isInitialized = false;
const int MIN_REPS_FOR_SET = 3;

// Power system definitions
const int batteryPin = A0;
const int redPin = 9;
const int greenPin = 10;
const int bluePin = 11;
const float referenceVoltage = 5.0;

// RTC definitions
unsigned long restStartTime = 0;
bool isResting = false;
const int REST_DURATION = 120000;
bool setCompleted = false;

// Display definitions
const int rs = 0, en = 1, d4 = 2, d5 = 3, d6 = 4, d7 = 5;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
/* SPARE CODE FOR REFERENCE
-------------------------------------------------------------------

  Serial.begin(9600);
  Wire.begin();
  delay(1000);
  
  // Initialize LCD early
  lcd.begin(16, 2);
  lcd.print("Initializing...");
  
  //Scan I2C devices 
  byte error, address;
  int count = 0;

  lcd.clear();
  lcd.print("Scanning I2C...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      lcd.clear();
      lcd.print("Device at 0x");
      if (address < 16) lcd.print("0");
      lcd.print(address, HEX);
      delay(1000);
      count++;
    }
  }
  lcd.clear();
  if (count == 0)
    lcd.print("No I2C devices");
  else
    lcd.print("I2C scan done");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("# of reps:");
  lcd.setCursor(12, 0);
  lcd.print("Set:");

  //Initialize Heartrate sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
   {
    while (!particleSensor.begin(Wire, I2C_SPEED_FAST)){
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    if (particleSensor.begin(Wire, I2C_SPEED_FAST)){
    break;
    }
    }
    
   }
   Serial.println("Place your index finger on the sensor with steady pressure.");

   particleSensor.setup(); //Configure sensor with default settings
   particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
   particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  // Initialize accelerometer with reset
  Wire.beginTransmission(0x68); // MPU6050 default address
  Wire.write(0x6B); // Power management register
  Wire.write(0x80); // Reset device
  Wire.endTransmission(true);
  delay(100); // Wait for reset

  if (!mpu.begin()) {
    lcd.clear();
    lcd.print("MPU6050 fail");
    while (1) {
      delay(10);
    }
  }
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(2.5);
  mpu.setMotionDetectionDuration(600);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(false); // Disable interrupts during stabilization
  
  // Extended stabilization
  for (int i = 0; i < 10; i++) {
    mpu.getMotionInterruptStatus();
    delay(100);
  }
  delay(6000); // 6 seconds stabilization
  mpu.setMotionInterrupt(true); // Enable interrupts after stabilization
  isInitialized = true;

  // Power system setup
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Initialize RTC 
  if (!rtc.begin()) {
    lcd.clear();
    lcd.print("RTC fail");
    while (1);
  }
  /*
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }


  // Setup LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("# of reps:");
  lcd.setCursor(12, 0);
  lcd.print("Set:");
  lcd.setCursor(0, 1);
  //lcd.print(max(rep - 1, 0)); // Apply -1 offset, ensure non-negative
  lcd.print(rep);
  lcd.setCursor(13, 1);
  lcd.print(set + 1);

  // Debug: Confirm rep is 0
  Serial.print("Initial rep count: ");
  Serial.println(rep);
  ---------------------------------------------------------------------------------
  */

 Serial.begin(9600);
 delay(1000); // Ensure peripherals are ready
 //scanI2C(); // Debugging: Confirm I2C devices  

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

  
  // Initialize accelerometer with reset
  Wire.beginTransmission(0x68); // MPU6050 default address
  Wire.write(0x6B); // Power management register
  Wire.write(0x80); // Reset device
  Wire.endTransmission(true);
  delay(100); // Wait for reset

  if (!mpu.begin()) {
    Serial.print("MPU6050 fail");
    while (1) {
      delay(10);
    }
  }
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(2.5);
  mpu.setMotionDetectionDuration(600);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(false); // Disable interrupts during stabilization
  
  // Extended stabilization
  for (int i = 0; i < 10; i++) {
    mpu.getMotionInterruptStatus();
    delay(100);
  }
  delay(6000); // 6 seconds stabilization
  isInitialized = true;

// Initialize RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC. Check wiring.");
    while (1);
  }
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set compile time
    Serial.println("RTC was not running. Time set.");
  } else {
    Serial.println("RTC initialized.");
  }

   //setup motion detection
   mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
   mpu.setMotionDetectionThreshold(1);               // Change these valies for motion detection sensitivity
   mpu.setMotionDetectionDuration(400);              //
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
// Heart rate code
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    
    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
if(!isResting){
  if (irValue < 50000) {
    if (beatAvg > 0) {
      lcd.setCursor(13, 1);
      lcd.print("   ");
    }
    beatAvg = 0;
    lcd.setCursor(12, 0);
    lcd.print("Set:");
    lcd.setCursor(13, 1);
    lcd.print(set + 1);
  }

  if (irValue > 50000) {
    lcd.setCursor(12, 0);
    lcd.print(" HR:");
    lcd.setCursor(13, 1);
    lcd.print(beatAvg);
  }
  
  if (beatAvg < 100) {
    lcd.setCursor(15, 1);
    lcd.print(" ");
  }
}
// Accelerometer and timing logic
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  currentTime = millis();

  // Check for 15-second inactivity to trigger rest only after a valid set
  if (rep >= MIN_REPS_FOR_SET && (currentTime - lastInterruptTime) >= TIME_THRESHOLD_SET && !isResting) {
    setCompleted = true;
    isResting = true;
    restStartTime = currentTime;
  }

// Rest countdown logic
  if (isResting && setCompleted) {
    unsigned long elapsedTime = currentTime - restStartTime;
    if (elapsedTime < REST_DURATION && !mpu.getMotionInterruptStatus()) {
      unsigned long remainingTime = (REST_DURATION - elapsedTime) / 1000;
      int remainingSeconds = remainingTime;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Resting: ");
      lcd.print(remainingSeconds / 60);
      lcd.print(":");
      if ((remainingSeconds % 60) < 10) lcd.print("0");
      lcd.print(remainingSeconds % 60);
      lcd.setCursor(0, 1);
      lcd.print("Set: ");
      lcd.print(set + 1);

      ////////////HR DURING REST
      long irValue = particleSensor.getIR();
      if (checkForBeat(irValue) == true) {
        long delta = millis() - lastBeat;
        lastBeat = millis();
        beatsPerMinute = 60 / (delta / 1000.0);
    
        if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
        }
      }

      if (irValue < 50000) {
    if (beatAvg > 0) {
      lcd.setCursor(12, 0);
      lcd.print("   ");
      lcd.setCursor(13, 1);
      lcd.print("   ");
    }
    beatAvg = 0;
    /*
    lcd.setCursor(12, 0);
    lcd.print("Set:");
    lcd.setCursor(13, 1);
    lcd.print(set + 1);
    */
  }

  if (irValue > 50000) {
    lcd.setCursor(10, 1);
    lcd.print("HR:");
    lcd.setCursor(13, 1);
    lcd.print(beatAvg);
  }
  
  if (beatAvg < 100) {
    lcd.setCursor(15, 1);
    lcd.print(" ");
  }
  
     
 ////////////HR DURING REST

    if(elapsedTime >= REST_DURATION) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Back to Work!");
      delay(5000);
      isResting = false;
      setCompleted = false;
      rep = 0;
      set++;
      restStartTime = 0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("# of reps:");
      lcd.setCursor(12, 0);
      lcd.print("Set:");
      lcd.setCursor(0, 1);
      lcd.print("0"); // Reset to 0 after rest
      lcd.setCursor(13, 1);
      lcd.print(set + 1);
    }

            //Accelerometer incrementation during "Back to Work" logic

    /* CODE TO HANDLE MOVEMENT AND POSITIONING EARLIER THAN END OF REST TIME
------------------------------------------------------------------------
    if(mpu.getMotionInterruptStatus()){
      if(y<Y_ACCEL_THRESHOLD_HIGH && y>= Y_ACCEL_THRESHOLD_HIGH && isResting)
      {
      isResting = false;
      setCompleted = false;
      rep = 0;
      set++;
      restStartTime = 0;
       lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Back to Work!");
      delay(5000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("# of reps:");
      lcd.setCursor(12, 0);
      lcd.print("Set:");
      lcd.setCursor(0, 1);
      lcd.print("0"); // Reset to 0 after rest
      lcd.setCursor(13, 1);
      lcd.print(set + 1);
      }
------------------------------------------------------------------------
      */

      float y = a.acceleration.y;
      if (y>=Y_ACCEL_THRESHOLD_HIGH && mpu.getMotionInterruptStatus()) {
      isResting = false;
      setCompleted = false;
      rep++;
      }

    }
    delay(10);
    return;
  }

  // Accelerometer motion detection and rep incrementation logic
  /*z <= ACCEL_THRESHOLD_LOW || z >= ACCEL_THRESHOLD_HIGH)*/

  if (isInitialized && mpu.getMotionInterruptStatus()) {
    timeSinceLastInterrupt = currentTime - lastInterruptTime;
    float y = a.acceleration.y;
    Serial.print("Y-Value: ");
    Serial.println(y);
    if ((y >= Y_ACCEL_THRESHOLD_HIGH) &&
        timeSinceLastInterrupt >= TIME_THRESHOLD_REP_MIN) {
      rep++;
      lastInterruptTime = currentTime;
      lcd.setCursor(10, 1);
      lcd.print("M");
      delay(100);
      lcd.setCursor(10, 1);
      lcd.print(" ");
      // Debug: Log rep increment
      Serial.print("Rep incremented to: ");
      Serial.println(rep);
    }
  }

  // Update LCD with reps, apply -1 offset
  lcd.setCursor(0, 1);
  lcd.print(rep); 
  /*if ((rep - 1) < 10) {
    lcd.setCursor(1, 1);
    lcd.print(" ");
  }
*/
// Power system code
  int rawValue = analogRead(batteryPin);
  float batteryVoltage = (rawValue / 1023.0) * referenceVoltage;
  if (batteryVoltage >= 4.0) {
    analogWrite(redPin, 0);
    analogWrite(greenPin, 255);
  } else if (batteryVoltage > 3.3 && batteryVoltage < 4.0) {
    analogWrite(redPin, 170);
    analogWrite(greenPin, 85);
  } else {
    analogWrite(redPin, 255);
    analogWrite(greenPin, 0);
  }
  analogWrite(bluePin, 0);
}