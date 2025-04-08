//Edited by Christopher Ritz for Design Project (Rep Tracker)
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

//used for set and rep incrementing logic
int rep = 0, motionDet = 0, set = 0;
// stores the time of the last interrupt
unsigned long lastInterruptTime = 0;
// define a time threshold (in ms)
const unsigned long TIME_THRESHOLD_REP = 2000; // 500 = 0.5
const unsigned long TIME_THRESHOLD_SET = 5000;

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
  mpu.setMotionDetectionThreshold(4);
  mpu.setMotionDetectionDuration(500);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);
}


    unsigned long currentTime;
    unsigned long timeSinceLastInterrupt;

void loop() {

   // get current time
   currentTime = millis();
    // calculate time since last interrupt
  if ((currentTime - lastInterruptTime) >= TIME_THRESHOLD_SET && rep != 0)
    {
      set++;
      rep = 0;
    }

  if(mpu.getMotionInterruptStatus()) {
         // Calculate time since last interrupt
    timeSinceLastInterrupt = currentTime - lastInterruptTime;
    
    // Condition: only proceed if enough time has passed since last interrupt
    if (timeSinceLastInterrupt >= TIME_THRESHOLD_REP) 
    {
      // Update the last interrupt time
      lastInterruptTime = currentTime;
    rep++;
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

  }
  delay(10);
  }
}