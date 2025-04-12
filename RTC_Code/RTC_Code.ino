#include <Wire.h>                // For I2C communication
#include <RTClib.h>              // Adafruit RTClib for DS1307
//#include <LiquidCrystal_I2C.h> // I2C LCD library

// SDA: A4 (Analog pin 4 on Uno)
// SCL: A5 (Analog pin 5 on Uno)
// VCC: 5V
// GND: GND

// Initialize objects
RTC_DS1307 rtc;
//LiquidCrystal_I2C lcd(0x27, 16, 2); // Adjust I2C address if needed

// Variables
unsigned long restStartTime = 0;
bool isResting = false;
const int REST_DURATION = 120; // 2 minutes in seconds
bool movementDetected = false; // Placeholder for accelerometer input
int setCounter = 0;           // Tracks completed sets

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("RTC not found!");
    while (1);
  }
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set RTC to compile time
  }
/*
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Rep Counter");
  lcd.setCursor(0, 1);
  lcd.print("Ready...");
  delay(2000); // Show welcome message
  lcd.clear();*/
}

void loop() {
  // --- Placeholder for Accelerometer Data ---
  movementDetected = false; // Replace later if needed

  // Check if there's movement
  if (movementDetected) {
    isResting = false;
    restStartTime = 0; // Reset rest timer
    //lcd.clear();
    //lcd.setCursor(0, 0);
    //lcd.print("Working...");
    //lcd.setCursor(0, 1);
    //lcd.print("Set: ");
    //lcd.print(setCounter);
  } else {
    // No movement detected
    if (!isResting && restStartTime == 0) {
      // Start rest timer after a set
      setCounter++; // Increment set counter
      isResting = true;
      DateTime now = rtc.now();
      restStartTime = now.unixtime(); // Get current time in seconds
    }

    // Calculate elapsed rest time
    DateTime now = rtc.now();
    unsigned long elapsedTime = now.unixtime() - restStartTime;

    if (isResting) {
      if (elapsedTime < REST_DURATION) {
        // Display countdown
        int remainingSeconds = REST_DURATION - elapsedTime;
        int minutes = remainingSeconds / 60;
        int seconds = remainingSeconds % 60;

        //lcd.clear();
        //lcd.setCursor(0, 0);
        //lcd.print("Rest Time:");
        //lcd.setCursor(0, 1);
        //lcd.print("Set: ");
        //lcd.print(setCounter);
        //lcd.setCursor(8, 1);
        //lcd.print(minutes);
        //lcd.print(":");
        //if (seconds < 10) lcd.print("0"); // Add leading zero
        //lcd.print(seconds);
      } else {
        // Rest time is up
        //lcd.clear();
        //lcd.setCursor(0, 0);
        //lcd.print("Get back to");
        //lcd.setCursor(0, 1);
        //lcd.print("work! Set: ");
        //lcd.print(setCounter);
        delay(2000); // Show message for 2 seconds
        isResting = false;
        restStartTime = 0; // Reset for next rest period
      }
    }
  }

  delay(100); // Small delay to avoid overwhelming the loop
}