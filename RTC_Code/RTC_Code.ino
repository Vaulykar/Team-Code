#include <Wire.h>              // For I2C communication
#include <RTClib.h>           // DS1307 RTC library
#include <LiquidCrystal_I2C.h> // I2C LCD library

// Initialize objects
RTC_DS1307 rtc;
LiquidCrystal_I2C lcd(0x27, 16, 2); // Adjust I2C address if needed

// Variables
unsigned long restStartTime = 0;
bool isResting = false;
const int REST_DURATION = 120; // 2 minutes in seconds
bool movementDetected = false; // Placeholder for accelerometer input
int setCounter = 0;            // Tracks completed sets

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

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Rep Counter");
  lcd.setCursor(0, 1);
  lcd.print("Sets: 0"); // Initial set count display
}

void loop() {
  // --- Placeholder for Accelerometer Data ---
  // Your teammate should insert code here to set 'movementDetected' based on accel data
  // Example: Read accel values, calculate magnitude, and set movementDetected = true/false
  // For now, it’s a dummy value (false) to show the structure
  movementDetected = false; // Replace this with actual accel logic

  // --- End of Placeholder ---

  // Check if there's movement
  if (movementDetected) {
    // Movement detected, reset rest state
    isResting = false;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Working...");
    lcd.setCursor(0, 1);
    lcd.print("Sets: ");
    lcd.print(setCounter);
  } else {
    // No movement detected
    if (!isResting) {
      // Start rest timer
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

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Rest Time:");
        lcd.setCursor(0, 1);
        lcd.print(minutes);
        lcd.print(":");
        if (seconds < 10) lcd.print("0"); // Add leading zero
        lcd.print(seconds);
      } else {
        // Rest time is up
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Get back to");
        lcd.setCursor(0, 1);
        lcd.print("work!");
        delay(2000); // Show message for 2 seconds
        
        // Increment set counter after rest period ends
        setCounter++;
        
        // Update display with new set count
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Rest Over");
        lcd.setCursor(0, 1);
        lcd.print("Sets: ");
        lcd.print(setCounter);
        
        isResting = false; // Reset for next rest period
      }
    }
  }

  delay(100); // Small delay to avoid overwhelming the loop
}