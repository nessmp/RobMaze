#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>


/*-----( Declare Constants )-----*/
/*-----( Declare objects )-----*/
// set the LCD address to 0x27 for a 16 chars 2 line display
// A FEW use address 0x3F
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

void setup() 
{
  Serial.begin(9600);  // Used to type in characters 

  lcd.begin(20,4);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  for(int i = 0; i< 3; i++)
  {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight();

  /*lcd.print("Hello, world!");
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print("HI!YourDuino.com");
  delay(8000); */ 
  lcd.clear();
  lcd.print("Holis soy rasputia");
}


void loop()
{
  /*
  lcd.noBacklight();
  delay(500);
  lcd.backlight();
  delay(500);
  */
      /*
    // when characters arrive over the serial port...
    if (Serial.available()) {
      // wait a bit for the entire message to arrive
      delay(100);
      // clear the screen
      lcd.clear();
      // read all the available characters
      while (Serial.available() > 0) {
        // display each character to the LCD
        lcd.write(Serial.read());
      }
    }
  }
  */

}
