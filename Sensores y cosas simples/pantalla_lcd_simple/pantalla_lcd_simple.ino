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

  /*lcd.print("Hello, world!");
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print("HI!YourDuino.com");
  delay(8000); */ 
  lcd.clear();
}


void loop()
{
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.write("Hello");
}
