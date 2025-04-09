#include <LiquidCrystal.h>
#include <RTClib.h>    // Adafruit RTClib library
#include <SoftWire.h>  // Software I2C library

// LCD on pins 0-5
LiquidCrystal lcd(0, 1, 2, 3, 4, 5); // RS=0, EN=1, D4=2, D5=3, D6=4, D7=5

// Software I2C for RTC on A2 (SDA) and A3 (SCL)
SoftWire softI2c(A2, A3); // SDA=A2, SCL=A3
RTC_DS1307 clock;         // RTC object from RTClib

// Timer variables
unsigned long previousMillis = 0;
const long interval = 120000; // 2 minutes in milliseconds
bool timerRunning = false;
int remainingSeconds = 0;

// Accelerometer variables
const int motionPin = 8; // 8
const int stillPin = 9; // 9
bool movementDetected = false;
//const int accelThreshold = 100; // Adjust based on your accelerometer (sensistivity is determined in acc code)

void setup() {
    // Configure Software I2C pins
    softI2c.begin(); // Initialize Software I2C

    // Initialize RTC with Software I2C
    if (!clock.begin(&softI2c)) { // Pass SoftWire object to RTClib
        // If RTC fails to initialize, loop forever (for debugging)
        while (1);
    }

    lcd.begin(16, 2); // Initialize 16x2 LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Rep Counter");
    lcd.setCursor(0, 1);
    lcd.print("Active");
}

void startTimer() {
    timerRunning = true;
    previousMillis = millis();
    remainingSeconds = 120;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Rest Timer:");
}

void updateTimer() {
    if (timerRunning) {
        unsigned long currentMillis = millis();

        if (currentMillis - previousMillis >= 1000) {
            remainingSeconds--;
            previousMillis = currentMillis - (currentMillis - previousMillis - 1000); 
lcd.setCursor(0, 1);
            lcd.print("    "); // Clear previous time
            lcd.setCursor(0, 1);
            lcd.print(remainingSeconds / 60);
            lcd.print(":");
            if (remainingSeconds % 60 < 10) lcd.print("0");
            lcd.print(remainingSeconds % 60);

            if (remainingSeconds <= 0) {
                timerRunning = false;
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Get back to");
                lcd.setCursor(0, 1);
                lcd.print("work!");
            }
        }
    }
}

void loop() {
    // Read accelerometer values
    int accelX = analogRead(accelXPin); // A0
    int accelY = analogRead(accelYPin); // A1

    // Detect movement
    movementDetected = (abs(accelX - 512) > accelThreshold || abs(accelY - 512) > accelThreshold);

    if (movementDetected) {
        if (timerRunning) {
            timerRunning = false; // Reset timer if movement restarts
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Rep Counter");
            lcd.setCursor(0, 1);
            lcd.print("Active");
        }
    } else {
        if (!timerRunning) {
            startTimer();
        }
        updateTimer();
    }
}