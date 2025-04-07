//Edited by Christopher Ritz for Design Project (Rep Tracker)
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

//used for set and rep incrementing logic
int rep = 0, motionDet = 0;
// stores the time of the last interrupt
unsigned long lastInterruptTime = 0;
// define a time threshold (in ms)
const unsigned long TIME_THRESHOLD = 10000; // 10000 = 10s

void setup(void) {
  // Initialize baud rate
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
  mpu.setMotionDetectionThreshold(8);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);
}

void loop() {

  if(mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /*if (a.acceleration.x>0 && a.acceleration.y>0 && a.acceleration.z>0){
    repCount ++;
    }
    */

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

    motionDet ++;
    if (motionDet == 2){
      rep ++;
    motionDet = 0;
    }
     // get current time
    unsigned long currentTime = millis();
    // calculate time since last interrupt
    unsigned long timeSinceLastInterrupt = currentTime - lastInterruptTime;
    
    // only proceed if enough time has passed since last interrupt
    if (timeSinceLastInterrupt >= TIME_THRESHOLD) {
      // update the last interrupt time
      lastInterruptTime = currentTime;

    }
    Serial.print("Reps Performed: ");
    Serial.print(rep);
    Serial.println("");
  }
  delay(10);
}