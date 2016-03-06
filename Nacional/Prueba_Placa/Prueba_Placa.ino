#include <NewPing.h>
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>
#include <SharpIR.h> //sharps
#include <SparkFunMLX90614.h>
#define MAX_DISTANCE 200
#define model 1080 //modelo del sharp GP2Y0A21Y

//////////////////
//////CALOR///////
//////////////////

IRTherm therm1;
IRTherm therm4;

//////////////////
//////SHARPS//////
//////////////////

byte Enf = A0;
byte Der =  A2;

SharpIR SharpEn(Enf, 25, 93, model);
SharpIR SharpDe(Der, 25, 93, model);

//////////////////
/////MOTORES//////
//////////////////

byte motDerE1 = 8;
byte motDerE2 = 9;

byte motDerA1 = 10;
byte motDerA2 = 11;

byte motIzqE1 = 4;
byte motIzqE2 = 5;

byte motIzqA1 = 6;
byte motIzqA2 = 7;

const byte MotD = 200;
const byte MotI = 200;

//////////////////
///ULTRASONICOS///
//////////////////

byte Trigger1 = 28;
byte Echo1 = 26;

NewPing sonar1(Trigger1, Echo1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar1.ping_cm();

//NO FUNCIONO
byte Trigger2 = 38;
byte Echo2 = 36;

//NO FUNCIONO
NewPing sonar2(Trigger2, Echo2, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar1.ping_cm();


byte Trigger3 = 42;
byte Echo3 = 40;


NewPing sonar3(Trigger3, Echo3, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte Trigger4 = 51;
byte Echo4 = 53;

NewPing sonar4(Trigger4, Echo4, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte Trigger5 = 47;
byte Echo5 = 49;

NewPing sonar5(Trigger5, Echo5, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte Trigger6 = 27;
byte Echo6 = 29;

NewPing sonar6(Trigger6, Echo6, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte Trigger7 = 23;
byte Echo7 = 25;

NewPing sonar7(Trigger7, Echo7, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte Trigger8 = 24;
byte Echo8 = 22;

NewPing sonar8(Trigger8, Echo8, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();


LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(motDerE1, OUTPUT);
  pinMode(motDerE2, OUTPUT);

  pinMode(motDerA1, OUTPUT);
  pinMode(motDerA2, OUTPUT);

  pinMode(motIzqE1, OUTPUT);
  pinMode(motIzqE2, OUTPUT);

  pinMode(motIzqA1, OUTPUT);
  pinMode(motIzqA2, OUTPUT);

  therm1.begin(0x1C);
  therm1.setUnit(TEMP_C);

  therm4.begin(0x4C);
  therm4.setUnit(TEMP_C);
  
  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.backlight(); // finish with backlight on  
  lcd.setCursor(0,0); //Start at character 4 on line 0
  lcd.print(";)");
  delay(1000);
  lcd.clear();
}

void Adelante()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, MotD);

  analogWrite(motDerA1, MotD);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, MotI);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, MotI);
}

void loop() {
  // put your main code here, to run repeatedly:
  lcd.clear();
  
  lcd.setCursor(0,0);
  lcd.print(sonar1.ping_cm());
  delay(29);
  
  lcd.setCursor(3,0);
  lcd.print(sonar3.ping_cm());
  delay(29);
  
  lcd.setCursor(6,0);
  lcd.print(sonar5.ping_cm());
  delay(29);
  
  lcd.setCursor(9,0);
  lcd.print(sonar7.ping_cm());
  delay(29);
  
  lcd.setCursor(0, 1);
  therm1.read();
  lcd.print(therm1.object());

  lcd.setCursor(6, 1);
  therm4.read();
  lcd.print(therm4.object());
 
  delay(1000);
}
