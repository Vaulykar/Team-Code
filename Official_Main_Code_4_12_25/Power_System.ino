/*
/ Define pin connections
const int batteryPin = A0;  // Analog pin to read battery voltage
const int redPin = 9;       // Red LED pin (PWM)
const int greenPin = 10;    // Green LED pin (PWM)
const int bluePin = 11;     // Blue LED pin (not used in this case)

// Voltage reference and divider
const float referenceVoltage = 5.0;      // Arduino operating voltage
const float voltageDividerRatio = 2.0;   // Assuming equal resistors
const float voltageThreshold = 2.5;      // Threshold to switch to full green

void setup() {
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
    Serial.begin(9600);  // Corrected capitalization
}

void loop() {
    int rawValue = analogRead(batteryPin); // Read the analog input
    float batteryVoltage = (rawValue / 1023.0) * referenceVoltage * voltageDividerRatio; // Convert to voltage

    Serial.print("Battery Voltage: ");
    Serial.println(batteryVoltage);

    // Change LED color based on voltage level
    if (batteryVoltage >= voltageThreshold) {
        analogWrite(redPin, 0);      // Full green, red off
        analogWrite(greenPin, 255);
    } else {
        float ratio = batteryVoltage / voltageThreshold;
        ratio = constrain(ratio, 0.0, 1.0);  // Clamp to [0, 1]

        int redIntensity = (1.0 - ratio) * 255;
        int greenIntensity = ratio * 255;

        analogWrite(redPin, redIntensity);
        analogWrite(greenPin, greenIntensity);
    }

    analogWrite(bluePin, 0); // Blue remains off
    delay(500); // Small delay to avoid rapid fluctuations
}
*/