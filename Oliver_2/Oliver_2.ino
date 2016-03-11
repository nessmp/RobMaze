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

//////////////////
//////COLOR//////
//////////////////
const byte out = 13;

int red = 0;
int green = 0;
int blue = 0;
String colon = "";

const int sensibilidad = 30;


SharpIR SharpEn(Enf, 25, 93, model);
SharpIR SharpIz(Izq, 25, 93, model);
SharpIR SharpDe(Der, 25, 93, model);

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

//////////////////
///ULTRASONICOS///
//////////////////

byte Trigger1 = 28;
byte Echo1 = 26;

NewPing sonar1(Trigger1, Echo1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar1.ping_cm();

byte Trigger2 = 38;
byte Echo2 = 36;

NewPing sonar2(Trigger2, Echo2, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar1.ping_cm();


byte Trigger7 = 23;
byte Echo7 = 25;

NewPing sonar7(Trigger7, Echo7, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte Trigger8 = 24;
byte Echo8 = 22;

NewPing sonar8(Trigger8, Echo8, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

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

//COLOR
String Color()
{
  String color = "Blanco";
 
  int cont;

  // digitalWrite(s2, LOW);
  //digitalWrite(s3, LOW);
  //count OUT, pRed, RED
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
 // digitalWrite(s3, HIGH);
  //count OUT, pBLUE, BLUE
  blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
//  digitalWrite(s2, HIGH);
  //count OUT, pGreen, GREEN
  green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);

  /*Serial.println("red: ");
  Serial.println(red);
  Serial.println("green: ");
  Serial.println(green);
  Serial.println("blue: ");
  Serial.println(blue);*/
cont=0;
for (int i=0; i<5; i++)
{
  
  if (sensibilidad < red || sensibilidad < green || sensibilidad < blue)
  {
    
    cont++;
  }

    red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);

  blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);

  green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
}

if (cont>0)
{
  color = "Negro";
}
else if (cont=0)
{
  color = "Blanco";
}

  return color;
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

void IzquierdaLento()
{
  analogWrite(motDerE1, 110);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 170);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 100);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 100);
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

void DerechaLento()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 110);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 170);

  analogWrite(motIzqE1, 100);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 100);
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

void Adelante10()
{
  EncDerE.write(0);
  int Enc = EncDerE.read();
  int const10 = const30 / 3;

  while (Enc < const10)
  {
    Adelante();
    Enc = EncDerE.read();
  }
  Detenerse();
}

void Adelante30()
{
  Adelante10();
  Detectar();
  Adelante10();
  Detectar();
  Adelante10();
  Detectar();
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
  if (Sharp > 14)
  {
    Pared = false;
  }
  return Pared;
}

void Detectar()
{
  therm1.read();
  therm2.read();
  therm3.read();
  therm4.read();
  int Therm1 = therm1.object();
  int Therm2 = therm2.object(); //Derecha
  int Therm3 = therm3.object();
  int Therm4 = therm4.object(); //Derecha
  int Temp = 23;
  //Serial.println(therm2.object());
  if (Therm1 > Temp || Therm2 > Temp || Therm3 > Temp || Therm4 > Temp)
  {
    Detenerse();
    for (int iI = 113; iI > 75; iI--)
    {
      Dispensador.write(iI);
      delay(1);
    }
    delay(1000);
    for (int iI = 75; iI < 113; iI++)
    {
      Dispensador.write(iI);
      delay(1);
    }
  }
}

void SeguirDerecha()
{
  bool ParedD = ParedDer();
  bool ParedE = ParedEnf();
  delay(100);
  ParedD = ParedDer();
  ParedE = ParedEnf();
  delay(100);
  ParedD = ParedDer();
  ParedE = ParedEnf();

  if (ParedD == false)
  {
    GiroDer90();
    delay(200);
    Adelante30();
    delay(1000);
    Detenerse();
    delay(500);
  }
  else if (ParedE == false)
  {
    Adelante30();
    delay(100);
    Detenerse();
    delay(1000);

  }
  else if (ParedD == true && ParedE == true)
  {
    GiroIzq90();
    delay(100);
  }
  Acejarse();
  //Detectar(0);
}

void Acejarse()
{
  lcd.print(SharpDe.distance());
  //Serial.println(SharpDe.distance());
  int Dist = SharpDe.distance();
  if (Dist < 24)
  {
    //Serial.println("entro 1 if");
    Dist = SharpDe.distance();

    do {
      if (Dist < 7)
      {
        while (Dist < 7)
        {
          IzquierdaM();
          Dist = SharpDe.distance();
          Serial.println(SharpDe.distance());
        }
      }
      else if (Dist > 9)
      {
        while (Dist > 9)
        {
          DerechaM();
          Dist = SharpDe.distance();
          Serial.println(SharpDe.distance());
        }
      }
      else if (Dist == 8 || Dist == 7 || Dist == 9)
        break;
      Detenerse();
    } while (Dist != 8 || Dist != 7 || Dist != 9);
  }
  /*
  int DistU1 = sonar1.ping_cm();
  Serial.println(DistU1);
  int DistU2 = sonar2.ping_cm();
  Serial.println(DistU2);
  int DistS = SharpEn.distance();
  Serial.println(DistS);
  Serial.println();
  if (DistU1 < 5 || DistU2 < 5)
  {
    while (DistS > 9)
    {
      Atras();
      DistS = SharpEn.distance();
    }
    Detenerse();
  }
  lcd.clear();
  lcd.print("Salio loop");
  */

  int Dist2 = SharpIz.distance();
  if (Dist2 < 24)
  {
    //Serial.println("entro 1 if");
    Dist2 = SharpIz.distance();

    do {
      if (Dist2 < 7)
      {
        while (Dist2 < 7)
        {
          DerechaM();
          Dist2 = SharpIz.distance();
          Serial.println(SharpIz.distance());
        }
      }
      else if (Dist2 > 9)
      {
        while (Dist2 > 9)
        {
          IzquierdaM();
          Dist2 = SharpIz.distance();
          Serial.println(SharpIz.distance());
        }
      }
      else if (Dist2 == 8 || Dist2 == 7 || Dist2 == 9)
        break;
      Detenerse();
    } while (Dist2 != 8 || Dist2 != 7 || Dist2 != 9);
  }
}

void loop()
{
  SeguirDerecha();
}
