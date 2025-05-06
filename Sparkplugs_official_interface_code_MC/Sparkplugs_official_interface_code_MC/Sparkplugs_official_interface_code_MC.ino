/* 
 *  Team: The Sparkplugs
 *  Mitchell Chunn, Tyler Dunn, Joshua Lane, Jeremiah "Christian" Pope, Christopher Ritz
 *  ECE 1013 - Rep_Tracker project
 *  User interface subsystem
 *  
 *  Pin Assignments:
 *  User Interface: LCD: Digital Pins 0–5
 *  Accelerometer: Digital Pin 8 (INT), Analog Pins A4 (SDA), A5 (SCL)
 *  Heartbeat Sensor: A4 (SDA), A5 (SCL)
 *  RTC Clock Module: A4 (SDA), A5 (SCL)
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

// Sensor status
bool mpuOK = false, hrOK = false, rtcOK = false;

// Heart rate
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

// Accelerometer
int rep = 0, set = 0;
unsigned long lastInterruptTime = 0;
const unsigned long TIME_THRESHOLD_REP_MIN = 600;
const unsigned long TIME_THRESHOLD_SET = 20000;
const float Y_ACCEL_THRESHOLD_LOW = -7.0;
const float Y_ACCEL_THRESHOLD_HIGH = 8.5;
unsigned long currentTime;
bool isInitialized = false;
const int MIN_REPS_FOR_SET = 3;

// Power system
const int batteryPin = A0;
const int redPin = 9;
const int greenPin = 10;
const int bluePin = 11;
const float referenceVoltage = 5.0;
float batteryVoltage = 0.0; // Global variable for battery voltage

// RTC and rest period
unsigned long restStartTime = 0, backToWorkStart = 0;
bool isResting = false, setCompleted = false;
const unsigned long REST_DURATION = 120000;
const unsigned long BACK_TO_WORK_DURATION = 5000;

// Display
const int rs = 0, en = 1, d4 = 2, d5 = 3, d6 = 4, d7 = 5;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.print(F("Booting..."));
  delay(1000);
  Wire.begin();

  // Heart rate
  for (int i = 0; i < 3; i++) {
    if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
      hrOK = true;
      break;
    }
    Serial.println(F("MAX30105 retry..."));
    delay(500);
  }
  if (hrOK) {
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
    Serial.println(F("Place finger on sensor"));
  }

  // Accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x80);
  Wire.endTransmission(true);
  delay(100);
  if (mpu.begin(0x69)) {
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(2.5);
    mpu.setMotionDetectionDuration(200);
    mpu.setInterruptPinLatch(true);
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(false);
    for (int i = 0; i < 5; i++) {
      mpu.getMotionInterruptStatus();
      delay(100);
    }
    delay(1500);
    mpu.setMotionInterrupt(true);
    mpuOK = true;
    isInitialized = true;
  } else {
    Serial.println(F("MPU6050 fail"));
    lcd.clear();
    lcd.print(F("MPU6050 fail"));
  }

  // RTC
  if (rtc.begin()) {
    if (!rtc.isrunning()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      Serial.println(F("RTC set"));
    }
    rtcOK = true;
  } else {
    Serial.println(F("RTC fail"));
    lcd.clear();
    lcd.print(F("RTC fail"));
  }

  // Power system
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("# of reps:"));
  lcd.setCursor(12, 0);
  lcd.print(F("Set:"));
  lcd.setCursor(0, 1);
  lcd.print(rep);
  lcd.setCursor(13, 1);
  lcd.print(set + 1);
}

void updateHeartRate() {
  if (!hrOK) return;
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  static int lastBeatAvg = -1;
  if (irValue < 50000) {
    if (beatAvg > 0) {
      lcd.setCursor(12, 0);
      lcd.print(F("Set:"));
      lcd.setCursor(13, 1);
      lcd.print(set + 1);
      beatAvg = 0;
    }
  } else if (beatAvg != lastBeatAvg) {
    lcd.setCursor(10, isResting ? 1 : 0);
    lcd.print(F("HR:"));
    lcd.setCursor(13, isResting ? 1 : 1);
    lcd.print(beatAvg);
    if (beatAvg < 100) lcd.print(F(" "));
    lastBeatAvg = beatAvg;
  }
}

void updateAccelerometer(sensors_event_t &a) {
  if (!mpuOK || !isInitialized) return;
  if (mpu.getMotionInterruptStatus()) {
    float y = a.acceleration.y;
    if ((y >= Y_ACCEL_THRESHOLD_HIGH || y <= Y_ACCEL_THRESHOLD_LOW) &&
        currentTime - lastInterruptTime >= TIME_THRESHOLD_REP_MIN) {
      rep++;
      lastInterruptTime = currentTime;
      lcd.setCursor(10, 1);
      lcd.print(F("M"));
      delay(100); // Consider removing for full non-blocking
      lcd.setCursor(10, 1);
      lcd.print(F(" "));
      Serial.print(F("Rep: "));
      Serial.println(rep);
    }
  }
}

void updateRestPeriod(sensors_event_t &a) {
  if (!isResting && rep >= MIN_REPS_FOR_SET && (currentTime - lastInterruptTime) >= TIME_THRESHOLD_SET) {
    setCompleted = true;
    isResting = true;
    restStartTime = currentTime;
    Serial.println(F("Rest started"));
  }
  if (isResting && setCompleted) {
    unsigned long elapsedTime = currentTime - restStartTime;
    if (elapsedTime < REST_DURATION && !mpu.getMotionInterruptStatus()) {
      unsigned long remainingTime = (REST_DURATION - elapsedTime) / 1000;
      int remainingSeconds = remainingTime;
      lcd.setCursor(0, 0);
      lcd.print(F("Rest "));
      lcd.print(remainingSeconds / 60);
      lcd.print(F(":"));
      if ((remainingSeconds % 60) < 10) lcd.print(F("0"));
      lcd.print(remainingSeconds % 60);
      lcd.print(F("  "));
      lcd.setCursor(0, 1);
      lcd.print(F("S"));
      if (set + 1 < 10) lcd.print(F("0"));
      lcd.print(set + 1);
      lcd.print(F(" V:"));
      lcd.print(batteryVoltage, 1);
    } else if (mpu.getMotionInterruptStatus()) {
      float y = a.acceleration.y;
      if ((y >= Y_ACCEL_THRESHOLD_HIGH || y <= Y_ACCEL_THRESHOLD_LOW) &&
          currentTime - lastInterruptTime >= TIME_THRESHOLD_REP_MIN) {
        rep++;
        lastInterruptTime = currentTime;
        static unsigned long motionStart = 0;
        if (motionStart == 0) motionStart = currentTime;
        if (currentTime - motionStart >= 2000) {
          isResting = false;
          setCompleted = false;
          set++;
          restStartTime = 0;
          motionStart = 0;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Back to Work!"));
          backToWorkStart = currentTime;
        }
      }
    } else if (elapsedTime >= REST_DURATION) {
      if (backToWorkStart == 0) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Back to Work!"));
        backToWorkStart = currentTime;
      }
      if (currentTime - backToWorkStart >= BACK_TO_WORK_DURATION) {
        isResting = false;
        setCompleted = false;
        rep = 0;
        set++;
        restStartTime = 0;
        backToWorkStart = 0;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("# of reps:"));
        lcd.setCursor(12, 0);
        lcd.print(F("Set:"));
        lcd.setCursor(0, 1);
        lcd.print(F("0"));
        lcd.setCursor(13, 1);
        lcd.print(set + 1);
      }
    }
  }
}

void loop() {
  currentTime = millis();

  // Power system
  int rawValue = analogRead(batteryPin);
  batteryVoltage = (rawValue / 1023.0) * referenceVoltage;
  if (batteryVoltage >= 4.0) {
    analogWrite(redPin, 0);
    analogWrite(greenPin, 128);
  } else if (batteryVoltage > 3.3) {
    analogWrite(redPin, 85);
    analogWrite(greenPin, 42);
  } else {
    analogWrite(redPin, 128);
    analogWrite(greenPin, 0);
  }
  analogWrite(bluePin, 0);

  // Accelerometer data
  sensors_event_t a, g, temp;
  if (mpuOK) mpu.getEvent(&a, &g, &temp);

  updateHeartRate();
  updateAccelerometer(a);
  updateRestPeriod(a);

  // LCD updates
  static int lastRep = -1, lastSet = -1;
  if (rep != lastRep) {
    lcd.setCursor(0, 1);
    lcd.print(rep);
    if (rep < 10) lcd.print(F(" "));
    lastRep = rep;
  }
  if (set + 1 != lastSet) {
    lcd.setCursor(13, 1);
    lcd.print(set + 1);
    lastSet = set + 1;
  }

  // Debug rest state
  static bool wasResting = false;
  if (isResting != wasResting) {
    Serial.print(F("Rest state: "));
    Serial.println(isResting ? F("ON") : F("OFF"));
    wasResting = isResting;
  }
}