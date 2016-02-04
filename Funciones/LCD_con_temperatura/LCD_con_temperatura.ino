#include <Wire.h> // I2C library, required for MLX90614
#include <SparkFunMLX90614.h> // SparkFunMLX90614 Arduino library
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

IRTherm calor1; // Create an IRTherm object to interact with throughout

void setup() {
  // put your setup code here, to run once:
   // put your setup code here, to run once:
  Serial.begin(9600); // Initialize Serial to log output
  calor1.begin(0x1B); // Initialize thermal IR sensor
  calor1.setUnit(TEMP_C); 
  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  for(int i = 0; i< 3; i++)
  {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (calor1.read()) // On success, read() will return 1, on fail 0.
  {
    // Use the object() and ambient() functions to grab the object and ambient
  // temperatures.
  // They'll be floats, calculated out to the unit you set with setUnit().
    Serial.print("Object: " + String(calor1.object(), 2));
    Serial.write('°'); // Degree Symbol
    Serial.println("C");
  }
  lcd.setCursor(1,1); //Start at character 4 on line 0
  lcd.print("Object: " + String(calor1.object(), 2) + " C");
  delay(2000);
}
