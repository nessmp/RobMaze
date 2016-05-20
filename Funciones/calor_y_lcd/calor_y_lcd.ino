#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <SparkFunMLX90614.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
IRTherm therm;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  therm.begin(0x2C);
  therm.setUnit(TEMP_C);
  lcd.begin(16,2);
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Listo?");
  
}

void loop() {
  // put your main code here, to run repeatedly:
    lcd.clear();
    therm.read();
    Serial.print("Object: " + String(therm.object(), 2));
    Serial.write('°'); // Degree Symbol
    Serial.println("C");
    Serial.print("Ambient: " + String(therm.ambient(), 2));
    Serial.write('°'); // Degree Symbol
    Serial.println("C");
    Serial.println();
    lcd.print("Object: " + String(therm.object(), 2));
    lcd.write('°');
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Ambient: " + String(therm.ambient(), 2));
    lcd.write('°');
    lcd.print("C");
    delay(2000);
}
