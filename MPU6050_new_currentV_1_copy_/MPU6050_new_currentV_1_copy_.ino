/*
ACCELEROMETER
EDITOR: Christopher Ritz
  MPU6050 Raw

  A code for obtaining raw data from the MPU6050 module with the option to
  modify the data output format.

  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki
*/
#include<Wire.h>
#include "I2Cdev.h"    // Include I2C communication library
#include "MPU6050.h"   // Include MPU6050 sensor library

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;           // Create MPU6050 object with default address 0x68
//MPU6050 mpu(0x69);         // Alternative address for AD0 high
//MPU6050 mpu(0x68, &Wire1); // Alternative for AD0 low with second I2C bus

/* OUTPUT FORMAT DEFINITION----------------------------------------------------------------------------------
- Use "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated list of the accel 
X/Y/Z and gyro X/Y/Z values in decimal. Easy to read, but not so easy to parse, and slower over UART.

- Use "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit binary, one right after the other. 
As fast as possible without compression or data loss, easy to parse, but impossible to read for a human. 
This output format is used as an output.
--------------------------------------------------------------------------------------------------------------*/ 
#define OUTPUT_READABLE_ACCELGYRO    // Enable human-readable output format
//#define OUTPUT_BINARY_ACCELGYRO     // Alternative binary output format (commented out)

int16_t ax, ay, az;    // Variables to store accelerometer readings (X, Y, Z axes)
int16_t gx, gy, gz;    // Variables to store gyroscope readings (X, Y, Z axes)
bool blinkState;       // Variable to control LED blinking state

bool motionDet; // Variable for evaluating motion detection
int16_t valDelta; // Variable for calculating difference in acc. Values

void setup() {
  /*--Start I2C interface--*/
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE    // Check which I2C implementation is used
    Wire.begin();          // Initialize I2C communication using Arduino Wire library
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);    // Initialize fast I2C communication at 400kHz
  #endif

  Serial.begin(9600);    // Start serial communication at 9600 baud rate

  /*Initialize device and check connection*/ 
  Serial.println("Initializing MPU...");    // Print initialization message
  mpu.initialize();        // Initialize MPU6050 sensor
  Serial.println("Testing MPU6050 connection...");    // Print connection test message
  if(mpu.testConnection() == false){    // Test if MPU6050 is responding
    Serial.println("MPU6050 connection failed");    // Print failure message
    while(true);           // Infinite loop if connection fails (halts program)
  }
  else{
    Serial.println("MPU6050 connection successful");    // Print success message
  }

  /* Use the code below to change accel/gyro offset values. Use MPU6050_Zero to obtain the recommended offsets */ 
  Serial.println("Updating internal sensor offsets...\n");    // Print offset update message
  mpu.setXAccelOffset(0);    // Set X-axis accelerometer offset to 0
  mpu.setYAccelOffset(0);    // Set Y-axis accelerometer offset to 0
  mpu.setZAccelOffset(0);    // Set Z-axis accelerometer offset to 0
  mpu.setXGyroOffset(0);     // Set X-axis gyroscope offset to 0
  mpu.setYGyroOffset(0);     // Set Y-axis gyroscope offset to 0
  mpu.setZGyroOffset(0);     // Set Z-axis gyroscope offset to 0
  /*Print the defined offsets*/
  Serial.print("\t");        // Print tab for formatting
  Serial.print(mpu.getXAccelOffset());    // Print X-axis accelerometer offset
  Serial.print("\t");        // Print tab for formatting
  Serial.print(mpu.getYAccelOffset());    // Print Y-axis accelerometer offset
  Serial.print("\t");        // Print tab for formatting
  Serial.print(mpu.getZAccelOffset());    // Print Z-axis accelerometer offset
  Serial.print("\t");        // Print tab for formatting
  Serial.print(mpu.getXGyroOffset());     // Print X-axis gyroscope offset
  Serial.print("\t");        // Print tab for formatting
  Serial.print(mpu.getYGyroOffset());     // Print Y-axis gyroscope offset
  Serial.print("\t");        // Print tab for formatting
  Serial.print(mpu.getZGyroOffset());     // Print Z-axis gyroscope offset
  Serial.print("\n");        // Print newline

  /*Configure board LED pin for output*/ 
  pinMode(LED_BUILTIN, OUTPUT);    // Set built-in LED pin as output (COMMENT OUT FOR FINAL IMPLEMENTATION)
}

void loop() {
  /* Read raw accel/gyro data from the module. Other methods commented*/
  //mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);    // Get all 6 axes of motion data (commented)
  mpu.getAcceleration(&ax, &ay, &az);    // Alternative: Get only acceleration data
  //mpu.getRotation(&gx, &gy, &gz);        // Alternative: Get only rotation data (commented)

  /*Print the obtained data on the defined format*/
  #ifdef OUTPUT_READABLE_ACCELGYRO    // If readable format is defined
    Serial.print("a/g:\t");           // Print label and tab
    Serial.print(ax); Serial.print("\t");    // Print X-axis acceleration
    Serial.print(ay); Serial.print("\t");    // Print Y-axis acceleration
    Serial.println(az); Serial.print("\t");    // Print Z-axis acceleration
   /* Serial.print(gx); Serial.print("\t");    // Print X-axis gyroscope
    Serial.print(gy); Serial.print("\t");    // Print Y-axis gyroscope
    Serial.println(gz);                      // Print Z-axis gyroscope with newline
    */
  #endif

  #ifdef OUTPUT_BINARY_ACCELGYRO    // If binary format is defined
    Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));    // Write X accel high and low bytes
    Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));    // Write Y accel high and low bytes
    Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));    // Write Z accel high and low bytes
    Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));    // Write X gyro high and low bytes
    Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));    // Write Y gyro high and low bytes
    Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));    // Write Z gyro high and low bytes
  #endif

  /*Blink LED to indicate activity*/
  blinkState = !blinkState;           // Toggle blink state //comment out for final
  digitalWrite(LED_BUILTIN, blinkState);    // Update LED state //comment out for final
}