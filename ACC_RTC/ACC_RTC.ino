#include <Wire.h>                // For I2C communication
#include <RTClib.h>              // Adafruit RTClib for DS1307
#include <Adafruit_MPU6050.h>    // MPU6050 library
#include <Adafruit_Sensor.h>     // Sensor library for MPU6050
//#include <LiquidCrystal_I2C.h> // I2C LCD library (uncomment for LCD)

// Initialize objects
RTC_DS1307 rtc;
Adafruit_MPU6050 mpu;
//LiquidCrystal_I2C lcd(0x27, 16, 2); // Adjust I2C address if needed

// Rep and set tracking
int rep = 0, set = 0, motionDet = 0;
unsigned long slowRep = 0; // Counts slow movements
unsigned long lastInterruptTime = 0; // Last motion time
const unsigned long TIME_THRESHOLD_SET = 30000; // 30s for new set
const unsigned long TIME_THRESHOLD_REP_MIN = 2000; // Min time between reps
const float ACCEL_THRESHOLD_LOW = -5.0; // Lower z-axis threshold
const float ACCEL_THRESHOLD_HIGH = 11.0; // Upper z-axis threshold

// Rest timer variables
unsigned long restStartTime = 0;
bool isResting = false;
const int REST_DURATION = 120; // 2 minutes in seconds

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize RTC
  if (!rtc.begin() || !rtc.isrunning()) {
    Serial.println("RTC failed!");
    while (1);
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set to compile time
  Serial.println("RTC initialized");

  // Initialize MPU6050
  while (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    delay(10);
  }
  Serial.println("MPU6050 Found!");
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(200);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  // Initialize LCD (uncomment if used)
  /*
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Rep Counter");
  lcd.setCursor(0, 1);
  lcd.print("Ready...");
  delay(2000);
  lcd.clear();
  */

  Serial.println("Rep Counter Ready");
}

void loop() {
  // Get current time and sensor data
  unsigned long currentTime = millis();
  DateTime now = rtc.now();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Skip rep/set logic if resting
  if (isResting) {
    unsigned long elapsedTime = now.unixtime() - restStartTime;
    if (elapsedTime < REST_DURATION) {
      // Display rest countdown
      int remainingSeconds = REST_DURATION - elapsedTime;
      int minutes = remainingSeconds / 60;
      int seconds = remainingSeconds % 60;
      Serial.print("Resting, Set: ");
      Serial.print(set + 1);
      Serial.print(", Time Left: ");
      Serial.print(minutes);
      Serial.print(":");
      if (seconds < 10) Serial.print("0");
      Serial.println(seconds);

      /*
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Rest Time:");
      lcd.setCursor(0, 1);
      lcd.print("Set: ");
      lcd.print(set + 1);
      lcd.setCursor(8, 1);
      lcd.print(minutes);
      lcd.print(":");
      if (seconds < 10) lcd.print("0");
      lcd.print(seconds);
      */
    } else {
      // Rest period over
      Serial.println("Rest done! Get back to work!");
      /*
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Get back to");
      lcd.setCursor(0, 1);
      lcd.print("work! Set: ");
      lcd.print(set + 1);
      */
      isResting = false;
      restStartTime = 0;
      delay(2000); // Show message briefly
    }
    delay(100);
    return; // Skip rep counting during rest
  }

  // Check for new set (30s without motion)
  if (rep > 0 && (currentTime - lastInterruptTime) >= TIME_THRESHOLD_SET) {
    set++;
    rep = 0;
    isResting = true;
    restStartTime = now.unixtime();
    Serial.print("New set started: ");
    Serial.println(set + 1);
    return; // Wait for rest period
  }

  // Check for motion interrupt
  if (mpu.getMotionInterruptStatus()) {
    unsigned long timeSinceLastInterrupt = currentTime - lastInterruptTime;

    // Detect rep based on z-axis thresholds or slow movement
    if (((a.acceleration.z <= ACCEL_THRESHOLD_LOW || a.acceleration.z >= ACCEL_THRESHOLD_HIGH) &&
         timeSinceLastInterrupt >= TIME_THRESHOLD_REP_MIN) || slowRep >= 20) {
      motionDet++;
      if (motionDet >= 1) { // Adjust threshold if needed
        rep++;
        motionDet = 0;
        slowRep = 0;
        lastInterruptTime = currentTime;
        Serial.print("Rep: ");
        Serial.print(rep);
        Serial.print(", Set: ");
        Serial.println(set + 1);
      }
    } else if (timeSinceLastInterrupt < TIME_THRESHOLD_REP_MIN) {
      slowRep++; // Increment slow movement counter
    }

    // Debug accelerometer data
    Serial.print("AccelX:");
    Serial.print(a.acceleration.x);
    Serial.print(", Y:");
    Serial.print(a.acceleration.y);
    Serial.print(", Z:");
    Serial.print(a.acceleration.z);
    Serial.println("");
  }

  delay(10); // Small delay to avoid overwhelming the loop
}