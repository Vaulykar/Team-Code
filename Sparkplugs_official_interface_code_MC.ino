/* 
*  Mitchell Chunn
*  ECE 1013 - Rep_Tracker project
*  User interface subsystem
*
*  Project Notes: 
*
*/

#include <Wire.h>
#include <LiquidCrystal.h>

byte heart[8] = {              //the bit assignments for the heart icon
  0b00000,
  0b01010,
  0b11111,
  0b11111,
  0b11111,
  0b01110,
  0b00100,
  0b00000
};

int repititions = 0; // temporary variables used for testing purposes
int heart_rate = 0;  //
int i = 0;           //

const int rs = 0, en = 1, d4 = 2, d5 = 3, d6 = 4, d7 = 5;    // initialize the library by associating any needed LCD interface pin
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                   // with the arduino pin number it is connected to

//function definition of LCD_Output
void LCD_Output(int reps, int heartrate){
  lcd.setCursor(0,1);// set the cursor to column 0, line 1 (note: line 1 is the second row, since counting begins with 0):
  lcd.print(reps); // reps is a placeholder for the rep variable
  lcd.setCursor(13, 1);
  lcd.print(heartrate); // heartrate is a placeholder for the heartrate variable
}

void setup() {
  lcd.createChar(8, heart);    // Load the custom character into CGRAM location 8
  lcd.begin(16, 2);            // define/set the LCD's number of columns and rows:
  lcd.print("# of reps:");    // Print a message to the LCD.
  lcd.setCursor(14, 0);       // change LCD print location
  lcd.write(byte(8));         // output custom char: heart
}

void loop() {

  for(i = 0; i < 1000; i++){                // This for loop is entirely for testing purposes
      repititions = i;                      // 
      heart_rate = i;                       // 
      delay(1000);                           //
      LCD_Output(repititions, heart_rate);  //
  }
}