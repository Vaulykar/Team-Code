//Edited by Christopher Ritz for Design Project (Rep Tracker)
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

//used for set and rep incrementing logic
int rep = 0, set = 0, motionDet = 0;
unsigned long slowRep = 0;

// stores the time of the last interrupt
unsigned long lastInterruptTime = 0;
// define a time threshold (in ms)
const unsigned long TIME_THRESHOLD_SET = 30000;
const unsigned long TIME_THRESHOLD_REP_MIN = 2000
//TIME_THRESHOLD_REP_MAX = 10000

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
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(200);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);
}

    unsigned long currentTime;
    unsigned long timeSinceLastInterrupt;

void loop() {

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

   // get current time
   currentTime = millis();
    // calculate time since last interrupt
  if ((currentTime - lastInterruptTime) >= TIME_THRESHOLD_SET && rep != 0)
    {
      set++;
      rep = 0;

      //RTC Logic for message display INSERT HERE:

    }

  if(mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    // Calculate time since last interrupt

    timeDel = currentTime - lastTime;
    lastTime = currentTime;

    timeSinceLastInterrupt = currentTime - lastInterruptTime;
    
    if (timeDel<TIME_THRESHOLD_REP_MIN)
    {
      slowRep++
    }
    // Condition: only proceed if enough time has passed since last interrupt

    //if ( ((a.acceleration.z<=-5.0) || (a.acceleration.z>=11.0)) || ((timeSinceLastInterrupt<TIME_THRESHOLD_REP_MAX) && (timeSinceLastInterrupt> TIME_THRESHOLD_REP_MIN)) ) // NOTE: In final code, we need to use the x and y rather than z axis (or just the y axis)

    if ( ((a.acceleration.z<=-5.0) || (a.acceleration.z>=11.0)) )// NOTE: In final code, we need to use the x and y rather than z axis (or just the y axis)
    //|| (slowRep >= 20))
    {
      motionDet++;
      if (motionDet == 1)
      { 
        rep++;
        motionDet = 0;
        slowRep = 0;
      }
    
      lastInterruptTime = currentTime;

      //Insert line 50-59 from RTC

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
  //insert else logic from RTC lines 61-101
  delay(10);
  }
}