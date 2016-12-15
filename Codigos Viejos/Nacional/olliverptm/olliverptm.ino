#include <LiquidCrystal_I2C.h>
#include <SharpIR.h> //sharps
#include <NewPing.h>
#include <Wire.h>
#define model 1080 //modelo del sharp GP2Y0A21Y
#define MAX_DISTANCE 100

//Ultrasonico especial para distancia larga hacia en frente
byte Trigger1 = 28;
byte Echo1 = 26;
byte Trigger2 = 38;
byte Echo2 = 36;

NewPing sonar1(Trigger1, Echo1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar1.ping_cm();
NewPing sonar2(Trigger2, Echo2, MAX_DISTANCE);
byte Enf = A0;
byte Izq = A1;
byte Der =  A2;

SharpIR SharpEn(Enf, 25, 93, model);
SharpIR SharpIz(Izq, 25, 93, model);
SharpIR SharpDe(Der, 25, 93, model);

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  Wire.begin();
  lcd.begin(16, 2);  // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.backlight(); // finish with backlight on
  lcd.setCursor(0, 0); //Start at character 4 on line 0
  lcd.print(";)");
}

void loop() {
  // put your main code here, to run repeatedly:
  lcd.clear();
  lcd.setCursor(0, 0);
  /*lcd.print("SharpEn: ");
  lcd.print(SharpEn.distance());
  Serial.println(SharpEn.distance());
  lcd.setCursor(0, 1);*/
  lcd.print("Sharp:   ");
  lcd.print(SharpEn.distance());
   lcd.setCursor(0, 1);
   lcd.print("Ult2:     ");
  lcd.print(sonar2.ping_cm());
  delay(1000);
}
