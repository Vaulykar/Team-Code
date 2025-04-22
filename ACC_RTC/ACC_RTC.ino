#include <Wire.h>                // For I2C communication
#include <RTClib.h>              // Adafruit RTClib for DS1307
#include <Adafruit_MPU6050.h>    // MPU6050 library
#include <Adafruit_Sensor.h>     // Sensor library for MPU6050

RTC_DS1307 rtc;
Adafruit_MPU6050 mpu;

int rep = 0, set = 0, motionDet = 0;
unsigned long slowRep = 0; // Variable that is used to evaluate a "slow rep"
unsigned long lastInterruptTime = 0;

const unsigned long TIME_THRESHOLD_SET = 30000; // 30 seconds
const unsigned long TIME_THRESHOLD_REP_MIN = 2000; // 2 seconds
const float ACCEL_THRESHOLD_LOW = -5.0;
const float ACCEL_THRESHOLD_HIGH = 11.0;

unsigned long restStartTime = 0;
bool isResting = false;
const int REST_DURATION = 120; // in seconds

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
  delay(1000); // Ensure peripherals are ready

  scanI2C(); // Debugging: Confirm I2C devices

  // Initialize RTC
  if (!rtc.begin()) {
  //  Serial.println("Couldn't find RTC. Check wiring.");
    while (1);
  }
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set compile time
   // Serial.println("RTC was not running. Time set.");
  } else {
   // Serial.println("RTC initialized.");
  }

  // Initialize MPU6050
  if (!mpu.begin()) {
  // Serial.println("MPU6050 not detected. Check wiring.");
    while (1);
  }
  //Serial.println("MPU6050 initialized.");

  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(400);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  //Serial.println("Rep Counter Ready");
}

void loop() {
  unsigned long currentTime = millis();
  DateTime now = rtc.now();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

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

    if (((a.acceleration.z <= ACCEL_THRESHOLD_LOW || a.acceleration.z >= ACCEL_THRESHOLD_HIGH) &&
         timeSinceLastInterrupt >= TIME_THRESHOLD_REP_MIN) && slowRep >= 20) {
      rep++;
      lastInterruptTime = currentTime;
      if (slowRep<20 && a.acceleration.z > ACCEL_THRESHOLD_LOW && a.acceleration.z < ACCEL_THRESHOLD_HIGH ) {
        slowRep++;
        lastInterruptTime = currentTime;

        //Serial.print("Rep: ");
        //Serial.print(rep);
        //Serial.print(" | Set: ");
        //Serial.println(set + 1);
      }
    }
    else if { slowRep >= 20}
    rep++;
    slowRep = 0;
    } 
  

 /*
   Serial.print("AccelX: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.println(a.acceleration.z);
  */
  }

  delay(10);
}
