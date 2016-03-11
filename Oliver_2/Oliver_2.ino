///////EL ATTACH PARA LOS LACK OF PROGRESS ESTAN EN EL PIN #3//////////

#include <Encoder.h> //para que crees...
#include <NewPing.h>
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>
#include <SharpIR.h> //sharps
#include <Servo.h>
#include "I2Cdev.h" //MPU
#include "MPU6050_6Axis_MotionApps20.h" //MPU
#include <SparkFunMLX90614.h>
#define MAX_DISTANCE 12
#define model 1080 //modelo del sharp GP2Y0A21Y
#define OUTPUT_READABLE_YAWPITCHROLL //Yaw Pitch Roll del MPU

//////////////////
/////MOTORES//////
//////////////////

byte motDerE1 = 8;
byte motDerE2 = 9;

byte motDerA1 = 11;
byte motDerA2 = 10;

byte motIzqE1 = 7;
byte motIzqE2 = 6;

byte motIzqA1 = 5;
byte motIzqA2 = 4;

const byte MotD = 255;
const byte MotI = 255;
const byte MotDM = 150;
const byte MotIM = 90;

//////////////////
/////ENCODERS/////
//////////////////

Encoder EncDerE(18, 19);
Encoder Enc2(17, 27);

long oldPosition  = -999;

int const90 = 3600;

const int const30 = 6000;

//////////////////
//////CALOR///////
//////////////////

IRTherm therm1; //IZQUIERDA ADELANTE
IRTherm therm2; //DERECHA ATRAS
IRTherm therm3; //IZQUIERDA ATRAS
IRTherm therm4; //DERECHA ENFRENTE

//////////////////
//////SHARPS//////
//////////////////

byte Enf = A0;
byte Izq = A1;
byte Der =  A2;

/////////////////
//////SERVO//////
/////////////////

Servo Dispensador;
const int pinservo = 47;
const int PulsoMinimo = 650;
const int PulsoMaximo = 2550;

SharpIR SharpEn(Enf, 25, 93, model);
SharpIR SharpIz(Izq, 25, 93, model);
SharpIR SharpDe(Der, 25, 93, model);

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

void setup() {
  Serial.begin(115200);

  //CALOR
  therm1.begin(0x1C);
  therm1.setUnit(TEMP_C);

  therm2.begin(0x2C); //derecha atras
  therm2.setUnit(TEMP_C);

  therm3.begin(0x3C);
  therm3.setUnit(TEMP_C);

  therm4.begin(0x4C);
  therm4.setUnit(TEMP_C);
  
  //MOTOR
  pinMode(motDerE1, OUTPUT);
  pinMode(motDerE2, OUTPUT);

  pinMode(motDerA1, OUTPUT);
  pinMode(motDerA2, OUTPUT);

  pinMode(motIzqE1, OUTPUT);
  pinMode(motIzqE2, OUTPUT);

  pinMode(motIzqA1, OUTPUT);
  pinMode(motIzqA2, OUTPUT);

  analogWrite(motDerE1, LOW);
  analogWrite(motDerE2, LOW);

  analogWrite(motDerA1, LOW);
  analogWrite(motDerA2, LOW);

  analogWrite(motIzqE1, LOW);
  analogWrite(motIzqE2, LOW);

  analogWrite(motIzqA1, LOW);
  analogWrite(motIzqA2, LOW);

  //LCD
  lcd.begin(16, 2);  // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.noBacklight(); // finish with backlight on
  lcd.setCursor(0, 0); //Start at character 4 on line 0
  lcd.print(";)");

  //SERVO
  Dispensador.attach(pinservo, PulsoMinimo, PulsoMaximo);
}

//MOTORES
void Detenerse()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 0);
}

void Adelante()
{
  analogWrite(motDerE1, 160); //190
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 220); //255  //205
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 148); //178  //132
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 148); //178  //132
  analogWrite(motIzqA2, 0);
}

void Izquierda()
{
  analogWrite(motDerE1, 160);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 220);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 148);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 148);
}

void Derecha()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 160);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 220);

  analogWrite(motIzqE1, 148);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 148);
  analogWrite(motIzqA2, 0);
}

void Atras()
{
  analogWrite(motDerE1, 0); //190
  analogWrite(motDerE2, 160);

  analogWrite(motDerA1, 0); //255  //205
  analogWrite(motDerA2, 220);

  analogWrite(motIzqE1, 0); //178  //132
  analogWrite(motIzqE2, 148);

  analogWrite(motIzqA1, 0); //178  //132
  analogWrite(motIzqA2, 148);
}

void DerechaM()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 180);

  analogWrite(motDerA1, 200);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 150);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 160);
}

void IzquierdaM()
{
  analogWrite(motDerE1, 180);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 200);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 150);

  analogWrite(motIzqA1, 160);
  analogWrite(motIzqA2, 0);
}

void GiroDer90()
{

  delay(1000);
  EncDerE.write(0);
  int Enc = EncDerE.read();
  //Serial.println(EncDerE.read());
  while (Enc < const90 )
  {
    Derecha();
    Enc = EncDerE.read();
  }
  Detenerse();
}

void Adelante30()
{
  delay(500);
  EncDerE.write(0);
  int Enc = EncDerE.read();

  while (Enc < const30)
  {
    Adelante();
    //Victima();
    Enc = EncDerE.read();
    //Victima();
  }
  Detenerse();
}

void Atras30()
{
  EncDerE.write(0);
  int Enc = EncDerE.read();
  while (Enc > const30 * -1)
  {
    Atras();
    Enc = EncDerE.read();
    Serial.println(Enc);
  }
  Detenerse();
}

void GiroIzq90()
{
  delay(1000);
  EncDerE.write(0);
  int Enc = EncDerE.read();
  //Serial.println(EncDerE.read());
  while (Enc > const90 * -1 )
  {
    Izquierda();
    Enc = EncDerE.read();
  }
  Detenerse();
}

bool ParedDer()
{
  bool Pared = true;
  int Sharp = SharpDe.distance();
  Serial.println(Sharp);
  if (Sharp >= 14)
  {
    Pared = false;
  }
  return Pared;
}

bool ParedIzq()
{
  bool Pared = true;
  int Sharp = SharpIz.distance();
  Serial.println(Sharp);
  if (Sharp >= 14)
  {
    Pared = false;
  }
  return Pared;
}

bool ParedEnf()
{
  bool Pared = true;
  int Sharp = SharpEn.distance();
  Serial.println(Sharp);
  if (Sharp >= 10)
  {
    Pared = false;
  }
  return Pared;
}

void loop() {
    Dispensador.write(113);
    delay(1000);
    Dispensador.write(75);
    delay(1000);
}
