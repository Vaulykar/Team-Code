/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Motion detection.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/
/*
#include <Wire.h>        // Library for I2C communication, used to talk to the MPU6050
#include <MPU6050.h>     // Library for the MPU6050 sensor, simplifies interaction with it

MPU6050 mpu;             // Create an MPU6050 object named 'mpu'

void setup() 
{
  Serial.begin(115200);  // Start serial communication at 115200 baud for debugging/output

  Serial.println("Initialize MPU6050");

  // Initialize the MPU6050 with a gyro scale of ±2000 degrees/sec and accelerometer range of ±16g
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);          // Wait 500ms before retrying if initialization fails
  }

  mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS); // Set accelerometer startup delay to 3ms

  // Disable various interrupt features (we're not using them in this code)
  mpu.setIntFreeFallEnabled(false);  
  mpu.setIntZeroMotionEnabled(false);
  mpu.setIntMotionEnabled(false);
  
  mpu.setDHPFMode(MPU6050_DHPF_5HZ); // Set digital high-pass filter to 5Hz for motion detection

  // Configure motion detection: threshold (sensitivity) and duration (how long motion must persist)
  mpu.setMotionDetectionThreshold(2);
  mpu.setMotionDetectionDuration(5);

  // Configure zero-motion detection (detecting stillness)
  mpu.setZeroMotionDetectionThreshold(4);
  mpu.setZeroMotionDetectionDuration(2);	
  
  checkSettings();       // Call function to print current MPU6050 settings for verification

  // Set 8 and 9 as outputs and initialize them to LOW (though they aren't used in loop())
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);  
}

void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:                ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Motion Interrupt:     ");
  Serial.println(mpu.getIntMotionEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Zero Motion Interrupt:     ");
  Serial.println(mpu.getIntZeroMotionEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Free Fall Interrupt:       ");
  Serial.println(mpu.getIntFreeFallEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Motion Threshold:          ");
  Serial.println(mpu.getMotionDetectionThreshold());

  Serial.print(" * Motion Duration:           ");
  Serial.println(mpu.getMotionDetectionDuration());

  Serial.print(" * Zero Motion Threshold:     ");
  Serial.println(mpu.getZeroMotionDetectionThreshold());

  Serial.print(" * Zero Motion Duration:      ");
  Serial.println(mpu.getZeroMotionDetectionDuration());
  
  Serial.print(" * Clock Source:              ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:             ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets:   ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());

  Serial.print(" * Accelerometer power delay: ");
  switch(mpu.getAccelPowerOnDelay())
  {
    case MPU6050_DELAY_3MS:            Serial.println("3ms"); break;
    case MPU6050_DELAY_2MS:            Serial.println("2ms"); break;
    case MPU6050_DELAY_1MS:            Serial.println("1ms"); break;
    case MPU6050_NO_DELAY:             Serial.println("0ms"); break;
  }  
  
  Serial.println();
}

void loop()
{
  // Read raw accelerometer data and activity status from the MPU6050
  Vector rawAccel = mpu.readRawAccel();
  Activites act = mpu.readActivites();

  // Control digital pin 8 based on detected motion (HIGH if motion, LOW if none)
  if (act.isActivity)
  {
    digitalWrite(8, HIGH);
  } else
  {
    digitalWrite(8, LOW);
  }

  // Control digital pin 9 based on detected inactivity (HIGH if still, LOW if moving)
  if (act.isInactivity)
  {
    digitalWrite(9, HIGH);
  } else
  {
    digitalWrite(9, LOW);
  }

  // Print activity status and directional motion data to Serial Monitor
  Serial.print(act.isActivity);
  Serial.print(act.isInactivity);

  Serial.print(" ");
  Serial.print(act.isPosActivityOnX);
  Serial.print(act.isNegActivityOnX);
  Serial.print(" ");

  Serial.print(act.isPosActivityOnY);
  Serial.print(act.isNegActivityOnY);
  Serial.print(" ");

  Serial.print(act.isPosActivityOnZ);
  Serial.print(act.isNegActivityOnZ);
  Serial.print("\n");
  delay(50); // Small delay to slow down the loop for readability
}
*/