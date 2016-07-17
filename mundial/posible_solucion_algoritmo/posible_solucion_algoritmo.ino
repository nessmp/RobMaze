#include <Encoder.h> //para que crees...
#include <NewPing.h>
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>
#include <SharpIR.h> //sharps
#include <Servo.h>
#include "I2Cdev.h" //MPU
#include "MPU6050_6Axis_MotionApps20.h" //MPU
#include "Wire.h"
#include <SparkFunMLX90614.h>R
#define MAX_DISTANCE 30
#define model 1080 //modelo del sharp GP2Y0A21Y

//Max de arreglo de x
byte const PROGMEM XX = 40;
//Maximo de arreglo de y
byte const PROGMEM YY = 40;
//Maximo de arreglos de z
byte const PROGMEM ZZ = 3;
//Maximo de pasos posibles
byte const PROGMEM maxPasos = 100;
//Maximo de cuadros negros
byte const PROGMEM maxNegros = 20;
//Almacenara el numero de paso en la cordenada dictada por los corchetes
byte Pos[ZZ][XX][YY];
//Almacenara las posibilidades de movimiento en el paso del corchete
byte Possibility[maxPasos];
/*Almacenara las posibles coordenadas a las que se puede mover
  estara cifrado, y = zz+xx+(yy + 1); x = zz+(xx + 1)+yy;
  o = zz+(xx - 1)+yy; p = zz+xx+(yy - 1);
  r = (zz - 1) + (xx - 1) + yy; b = (zz + 1) + (xx + 1) + yy;
  s = (zz - 1) + (xx + 1) + yy; c = (zz + 1) + (xx - 1) + yy;
  t = (zz - 1) + xx + (yy + 1); d = (zz + 1) + xx + (yy + 1);
  u = (zz - 1) + xx + (yy - 1); e = (zz + 1) + xx + (yy - 1);
*/
char Run[maxPasos][4];
//Ubicación de cuadros negros
int listaX[maxNegros];
//Subindice para el areglos de listaX
byte subNegro = 0;
/*Dirección a la cual esta viendo el robot, donde 1 siempre es
  la dirección en donde empezo observando.
                1
                ^
                |
             3<- ->2
                |
                v
                4
*/
byte Direcc = 1;
//Paso en el que se quedo
byte Paso = 0;
//Paso en el que se esta
byte pasoActual = 0;
//Almacenara la coordenada actual de X
byte bX = XX / 2;
//Almacenara la coordenada actual de Y
byte bY = YY / 2;
//Almacenara la coordenada actual de Z
byte bZ = 1;
//true si en el mov anterios se cruzo una rampa
bool bARampa = false;
//Paso antes de mov de rampa
byte bPassRampa = 255;
//Paso de Rampa
byte brRampa = 255;
//Posibilidad de Run de Rampa
byte biI = 255;
//Char que va a cambiar el run antes de rampa
char cRampa = 'a';
//Char que va a cambiar el run despues de la rampa
char cRa = 'a';
//Para La rampa solo cambie datos cuando se esta explorando nuevo terreno
bool Explore = false;

bool bVictimaDetectada = false;
bool bInicio = true;
bool bRampaArriba = false;

//////////////////
///Calibracion////
//////////////////

byte CalibCalor = 25;
int const PROGMEM CalibNegro = 1000;
int const PROGMEM CalibBlanco = 600;

//////////////////
/////MPU6050//////
//////////////////

MPU6050 accelgyro;
int ax, ay, az;
int gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO

//////////////////
/////MOTORES//////
//////////////////

byte const PROGMEM motDerE1 = 8;
byte const PROGMEM motDerE2 = 9;

byte const PROGMEM motDerA1 = 11;
byte const PROGMEM motDerA2 = 10;

byte const PROGMEM motIzqE1 = 7;
byte const PROGMEM motIzqE2 = 6;

byte const PROGMEM motIzqA1 = 5;
byte const PROGMEM motIzqA2 = 4;

byte const PROGMEM MotD = 255;
byte const PROGMEM MotI = 255;
byte const PROGMEM MotDM = 150;
byte const PROGMEM MotIM = 90;

//////////////////
/////ENCODERS/////
//////////////////

Encoder EncDerE(18, 19);
Encoder Enc2(17, 27);

long oldPosition  = -999;

int const PROGMEM const90 = 3715;

int const PROGMEM const30 = 5500;

//////////////////
//////CALOR///////
//////////////////

IRTherm therm1; //IZQUIERDA ADELANTE

IRTherm therm3; //IZQUIERDA ATRAS

//////////////////
//////SHARPS//////
//////////////////

byte const PROGMEM Enf = A0;
byte const PROGMEM Izq = A1;
byte const PROGMEM Der =  A2;

SharpIR SharpEn(Enf, 25, 93, model);
SharpIR SharpIz(Izq, 25, 93, model);
SharpIR SharpDe(Der, 25, 93, model);

/////////////////
//////SERVO//////
/////////////////

Servo Dispensador;
byte const PROGMEM pinservo = 47;
int const PROGMEM PulsoMinimo = 650;
int const PROGMEM PulsoMaximo = 2550;

//////////////////
//////COLOR//////
//////////////////
byte const PROGMEM s0 = 13;
byte const PROGMEM s1 = 12;
byte const PROGMEM s2 = 29;
byte const PROGMEM s3 = 1;
byte const PROGMEM out = 14;

//////////////////
////////LCD///////
//////////////////

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

//////////////////
///ULTRASONICOS///
//////////////////

byte const PROGMEM Trigger1 = 28;
byte const PROGMEM Echo1 = 26;

NewPing sonar1(Trigger1, Echo1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar1.ping_cm();

byte const PROGMEM Trigger2 = 38;
byte const PROGMEM Echo2 = 36;

NewPing sonar2(Trigger2, Echo2, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar1.ping_cm();

byte const PROGMEM Trigger3 = 42;
byte const PROGMEM Echo3 = 40;

NewPing sonar3(Trigger3, Echo3, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte const PROGMEM Trigger4 = 51;
byte const PROGMEM Echo4 = 53;

NewPing sonar4(Trigger4, Echo4, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte const PROGMEM Trigger7 = 23;
byte const PROGMEM Echo7 = 25;

NewPing sonar7(Trigger7, Echo7, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte const PROGMEM Trigger8 = 24;
byte const PROGMEM Echo8 = 22;

NewPing sonar8(Trigger8, Echo8, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte const PROGMEM velMDE = 138;
byte const PROGMEM velMDA = 138;
byte const PROGMEM velMIE = 125;
byte const PROGMEM velMIA = 119;
byte Dif = 0;
byte cambioDer = 0;
byte cambioIzq = 0;

void setup() {
  Serial.begin(38400);
  Serial.println("Nesquik");

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  accelgyro.initialize();

  //CALOR
  therm1.begin(0x1C);
  therm1.setUnit(TEMP_C);



  therm3.begin(0x3C);
  therm3.setUnit(TEMP_C);

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
  lcd.backlight(); // finish with backlight on
  lcd.setCursor(0, 0); //Start at character 4 on line 0
  lcd.print("OLIVER");

  //SERVO
  Dispensador.attach(pinservo, PulsoMinimo, PulsoMaximo);
  Dispensador.write(113);

  //COLOR
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out , INPUT);

  digitalWrite(s0, LOW); //HIGH, HIGH dif de 20
  digitalWrite(s1, HIGH);

  digitalWrite(s2, HIGH);
  digitalWrite(s3, LOW);
  ////Serial.println("hola");

  for (byte iI = 0; iI < ZZ; iI++)
  {
    for (byte iJ = 0; iJ < XX; iJ++)
    {
      for (byte iK = 0; iK < YY; iK++)
      {
        Pos[iI][iJ][iK] = 255;
      }
    }
  }

  for (byte iI = 0; iI < maxPasos; iI++)
  {
    for (byte iJ = 0; iJ < 4; iJ++)
    {
      Run[iI][iJ] = 'a';
    }
    Possibility[iI] = 255;
  }
  Calor();
  Serial.println("Zucaritas");
}

void BubbleSort(int num[])
{
  int i, j, flag = 1;    // set flag to 1 to start first pass
  int temp;             // holding variable
  int numLength = 100;
  for (i = 1; (i <= numLength) && flag; i++)
  {
    flag = 0;
    for (j = 0; j < (numLength - 1); j++)
    {
      if (num[j + 1] > num[j])    // ascending order simply changes to <
      {
        temp = num[j];             // swap elements
        num[j] = num[j + 1];
        num[j + 1] = temp;
        flag = 1;               // indicates that a swap occurred.
      }
    }
  }
}

int MPUY()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  return az;
}

int MPUP()
{
  int totax[100];
  for (byte iI = 0; iI < 100; iI++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    totax[iI] = ax;
  }
  BubbleSort(totax);
  //Comentario inutil
  return totax[50];
}

int MPUR()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  return ay;
}

bool HoyoNegro()
{
  bool iReturn = false;
  int MPU = MPUP();
  if (MPU > 2000)
  {
    Atras30();
    if (ParedEnf())
    {
      Acomodo();
    }
    else
    {
      Acomodo();
      GiroDer90();
      Acomodo();
      GiroDer90();
      Acomodo();
      Atras30();
      Atras30();
      bRampaArriba = true;
    }
  }
  else {
    int color = pulseIn(out, LOW);
    if (color > CalibNegro)
    {
      iReturn = true;
    }
    color = pulseIn(out, LOW);
    if (color > CalibNegro)
    {
      iReturn = true;
    }
  }
  return iReturn;
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
  analogWrite(motDerE1, velMDE); //210
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, velMDA); //210
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, velMIE); //190
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, velMIA); //190
  analogWrite(motIzqA2, 0);
}

void AdelanteRampa()
{
  analogWrite(motDerE1, 210); //160  //190
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 210); //220  //240
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 170); //182  //192
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 170); //172  //192
  analogWrite(motIzqA2, 0);
}

void Atras()
{
  analogWrite(motDerE1, 0); //160  //190
  analogWrite(motDerE2, velMDE);

  analogWrite(motDerA1, 0); //220  //240
  analogWrite(motDerA2, velMDA);

  analogWrite(motIzqE1, 0); //182  //192
  analogWrite(motIzqE2, velMIE);

  analogWrite(motIzqA1, 0); //172  //192
  analogWrite(motIzqA2, velMIA);
}

void Atras2(byte cambioDe, byte cambioIz)
{
  analogWrite(motDerE1, 0); //160  //190
  analogWrite(motDerE2, velMDE + (cambioDe * velMDE / 100));

  analogWrite(motDerA1, 0); //220  //240
  analogWrite(motDerA2, velMDA + (cambioDe * velMDA / 100));

  analogWrite(motIzqE1, 0); //182  //192
  analogWrite(motIzqE2, velMIE + (cambioIz * velMIE / 100));

  analogWrite(motIzqA1, 0); //172  //192
  analogWrite(motIzqA2, velMIA + (cambioIz * velMIA / 100));
}

void AtrasRampa()
{
  analogWrite(motDerE1, 0); //160  //190
  analogWrite(motDerE2, velMDE + (20 * velMDE / 100));

  analogWrite(motDerA1, 0); //220  //240
  analogWrite(motDerA2, velMDA + (20 * velMDE / 100));

  analogWrite(motIzqE1, 0); //182  //192
  analogWrite(motIzqE2, velMIE + (20 * velMDE / 100));

  analogWrite(motIzqA1, 0); //172  //192
  analogWrite(motIzqA2, velMIA + (20 * velMDE / 100));
}

void Izquierda()
{
  analogWrite(motDerE1, velMDE); //210
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, velMDA); //210
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 0); //190
  analogWrite(motIzqE2, velMIE);

  analogWrite(motIzqA1, 0); //190
  analogWrite(motIzqA2, velMIA);
}

void Derecha()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, velMDE - (15 * velMDE / 100));

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, velMDA - (15 * velMDA / 100));

  analogWrite(motIzqE1, velMIE - (15 * velMIE / 100));
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, velMIA - (15 * velMIA / 100));
  analogWrite(motIzqA2, 0);
}

void DerechaM() //160 220 182 172
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 220 + Dif);

  analogWrite(motDerA1, 220 + Dif);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 182 + Dif);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 220 + Dif);
}

void IzquierdaM()//160 220 182 172
{
  analogWrite(motDerE1, 230 + Dif);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 220 + Dif);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 182 + Dif);

  analogWrite(motIzqA1, 182 + Dif);
  analogWrite(motIzqA2, 0);
}

void IzquierdaDiagonalAtras()//160 220 182 172
{
  analogWrite(motDerE1, 0); //210
  analogWrite(motDerE2, velMDE + (50 * velMDE / 100));

  analogWrite(motDerA1, 0); //210
  analogWrite(motDerA2, 255);

  analogWrite(motIzqE1, 0); //190
  analogWrite(motIzqE2, 255);

  analogWrite(motIzqA1, 0); //190
  analogWrite(motIzqA2, velMIA);
}

void DerechaDiagonalAtras()//160 220 182 172
{
  analogWrite(motDerE1, 0); //210
  analogWrite(motDerE2, 255);

  analogWrite(motDerA1, 0); //210
  analogWrite(motDerA2, velMDA);

  analogWrite(motIzqE1, 0); //190
  analogWrite(motIzqE2, velMIE + (40 * velMIE / 100));

  analogWrite(motIzqA1, 0); //190
  analogWrite(motIzqA2, velMIA + (70 * velMIA / 100));
}

void IzquierdaM30()
{
  EncDerE.write(0);
  Dif = -50;
  IzquierdaM();
  int Enc = EncDerE.read();
  while (Enc < 70)
  {
    Enc = EncDerE.read();
  }
  Detenerse();
  Dif = 0;
}

void DerechaM30()
{
  EncDerE.write(0);
  Dif = -50;
  DerechaM();
  int Enc = EncDerE.read();
  while (Enc > -70)
  {
    Enc = EncDerE.read();
  }
  Detenerse();
  Dif = 0;
}

void GiroDer18()
{
  EncDerE.write(0);
  int Enc = EncDerE.read();
  if (Enc < const90 / 5 )
  {
    Derecha();
    while (Enc < const90 / 5 )
    {
      Enc = EncDerE.read();
    }
  }
  Detenerse();
}

void GiroDer5()
{
  EncDerE.write(0);
  int Enc = EncDerE.read();
  if (Enc < const90 / 18 )
  {
    Derecha();
    while (Enc < const90 / 18 )
    {
      Enc = EncDerE.read();
    }
  }
  Detenerse();
}
void GiroDer90()
{
  delay(500);
  GiroDer18();
  Detectar();
  GiroDer18();
  Detectar();
  GiroDer18();
  Detectar();
  GiroDer18();
  Detectar();
  GiroDer18();
  Detectar();
  Detenerse();
}

void GiroIzq18()
{
  EncDerE.write(0);
  int Enc = EncDerE.read();
  if (Enc > const90 / 5 * -1 )
  {
    Izquierda();
    while (Enc > const90 / 5 * -1 )
    {
      Enc = EncDerE.read();
    }
  }
  Detenerse();
}

void GiroIzq5()
{
  EncDerE.write(0);
  int Enc = EncDerE.read();
  if (Enc > const90 / 18 * -1 )
  {
    Izquierda();
    while (Enc > const90 / 18 * -1 )
    {
      Enc = EncDerE.read();
    }
  }
  Detenerse();
}
void GiroIzq90()
{
  delay(500);
  GiroIzq18();
  Detectar();
  GiroIzq18();
  Detectar();
  GiroIzq18();
  Detectar();
  GiroIzq18();
  Detectar();
  GiroIzq18();
  Detectar();
  Detenerse();
}

void Adelante5()
{
  EncDerE.write(0);
  int Enc = EncDerE.read();
  int const5 = const30 / 6;

  while (Enc < const5)
  {
    Adelante();
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
  Detectar();
  Adelante10();
  //ultMov();
  Detectar();
  Adelante10();
  //ultMov();
  Detectar();
  Adelante10();
  //ultMov();
  Detectar();
  Detenerse();
  bVictimaDetectada = false;
}

void Atras30()
{
  EncDerE.write(0);
  int Enc = EncDerE.read();
  while (Enc > (const30 * -1) + 500)
  {
    Atras();
    Enc = EncDerE.read();
  }
  Detenerse();
}

bool ParedDer()
{
  bool Pared = true;
  for (int iI = 0; iI < 2; iI ++)
  {
    byte Sharp = SharpDe.distance();
    byte u1 = sonar3.ping_cm();
    delay(30);
    byte u2 = sonar4.ping_cm();
    delay(30);
    if (Sharp >= 16 ||(u1 == 0 || u2 == 0))
    {
      Pared = false;
    }
  }
  return Pared;
}

bool ParedIzq()
{
  bool Pared = true;
  byte Sharp = SharpIz.distance();
  byte u1 = sonar7.ping_cm();
  delay(30);
  byte u2 = sonar8.ping_cm();
  delay(30);
  if (Sharp >= 16 || (u1 == 0 || u2 == 0))
  {
    Pared = false;
  }
  return Pared;
}

bool ParedEnf()
{
  bool Pared = true;
  byte Sharp = SharpEn.distance();
  byte u1 = sonar1.ping_cm();
  byte u2 = sonar2.ping_cm();
  if (Sharp > 50 || (u1 == 0 || u2 == 0))
  {
    Pared = false;
  }
  return Pared;
}

void RampaAbajoIzq()
{
  int Roll = 0;
  if (ParedDer() && ParedEnf() && !ParedIzq())
  {
    IzquierdaM();
    delay(1700);
    Detenerse();
    Roll = MPUR();
    if (Roll < -7)
    {
      IzquierdaM();
      delay(3000);
      Detenerse();
      GiroIzq90();
      Acomodo();
      GiroIzq90();
      Acomodo();
    }
    else
    {
      if (HoyoNegro())
      {
        DerechaM();
        delay(1000);
        Adelante();
        Detenerse();
        Acomodo();
      }
      else
      {
        GiroDer90();
        Acomodo();
        Adelante30();
        GiroIzq90();
        Acomodo();
      }
    }
  }
}

void Detectar()
{
  therm1.read();

  therm3.read();
  byte Therm1 = therm1.object();
  byte Therm3 = therm3.object();
  if ((Therm1 > CalibCalor ||  Therm3 > CalibCalor) && bVictimaDetectada == false)
  {
    Detenerse();
    bVictimaDetectada = true;
    Detenerse();
    lcd.clear();
    lcd.print("VICTIMA");
    lcd.setCursor(0, 1);
    lcd.print("DETECTADA");
    Dispensador.write(113);
    delay(100);
    Dispensador.write(75);
    delay(200);
    Dispensador.write(113);
    delay(100);
    for (int iI = 0; iI < 5; iI++)
    {
      lcd.noBacklight();
      delay(500);
      lcd.backlight();
      delay(500);
    }
  }
  lcd.clear();
}

void Acomodo()
{
  //delay(30);
  Acejarse2();
  //delay(30);
  Ultacomodo();
  //delay(30);
  AcejarseEnfrente2();
  Evadir();
}

void AcomodoRampa()
{
  Acejarse2();
  Ultacomodo();
}

void AcejarseDerecha()
{
  byte Dist = SharpDe.distance();
  delay(5);
  Dist = SharpDe.distance();

  while (Dist != 8 ) {
    if (Dist < 8)
    {

      while (Dist < 8)
      {
        IzquierdaM30();
        //delay(90);
        Dist = SharpDe.distance();
        //delay(10);
        ////Serial.print("1zq    "); ////Serial.println(Dist);
      }
    }
    else if (Dist > 8)
    {

      while (Dist > 8)
      {
        DerechaM30();
        //delay(90);
        Dist = SharpDe.distance();
        //delay(10);
        ////Serial.print("der    "); ////Serial.println(Dist);
      }
    }
    else if (Dist == 8)
      break;
    Detenerse();
  }
}

//izquierda
void AcejarseIzquierda()
{
  byte Dist2 = SharpIz.distance();
  Dist2 = SharpIz.distance();

  while (Dist2 != 8) {
    if (Dist2 < 8)
    {

      while (Dist2 < 8)
      {
        DerechaM30();
        //delay(90);
        Dist2 = SharpIz.distance();
        //delay(10);
      }
    }
    else if (Dist2 > 8)
    {

      while (Dist2 > 8)
      {
        IzquierdaM30();
        //delay(90);
        Dist2 = SharpIz.distance();
        //delay(10);
      }
    }
    else if (Dist2 == 8)
      break;
    Detenerse();
  }
}

void Blink()
{
  for (int iI = 0; iI < 4; iI++)
  {
    lcd.noBacklight();
    delay(100);
    lcd.backlight();
    delay(100);
    iI++;
  }
}

//Ya no se usa esta funcion
void Revisiones()
{
  //delay(300);
  bool Hoyo = HoyoNegro();
  if (Hoyo == false)
  {
    delay(30);
  }
  //Comentar esto si no se esta siguiendo la derecha
  /*
    else
    {
    GiroIzq90();
    if (ParedEnf())
    {
      GiroIzq90();
    }
    else
    {
      Adelante30();
      Revisiones();
    }
    }
  */
}

void Calibracion()
{
  lcd.clear();
  therm1.read();
  therm3.read();

  byte Therm1 = therm1.object();
  byte Therm3 = therm3.object();

  lcd.setCursor(0, 0);
  lcd.print(Therm1);
  lcd.print("-");
  lcd.print(Therm3);

  int color = pulseIn(out, LOW);
  lcd.setCursor(0, 1);
  lcd.print(color);
  delay(1000);
}

void UltIzq()
{
  //delay(30);
  int U2 = sonar8.ping_cm();
  delay(30);
  int U1 = sonar7.ping_cm();
  delay(50);
  //delay(1000);

  while ((U1 - U2) != 0)
  {
    if ((U1 - U2) < 0)
    {
      EncDerE.write(0);
      int Enc = EncDerE.read();
      Dif = -50;
      Izquierda();
      while (Enc > ((const90 * 1 / 90) * -1 ))
      {
        Enc = EncDerE.read();
      }
      Detenerse();
      Dif = 0;
      //delay(90);
    }
    else if ((U1 - U2) > 0)
    {
      EncDerE.write(0);
      int Enc = EncDerE.read();
      Dif = -50;
      Derecha();
      while (Enc < (const90 * 1 / 90))
      {
        Enc = EncDerE.read();
      }
      Detenerse();
      Dif = 0;
      //delay(90);
    }


    //delay(30);
    U1 = sonar7.ping_cm();
    delay(30);
    U2 = sonar8.ping_cm();
    delay(30);
  }
}

void UltDer()
{
  //delay(30);
  int U1 = sonar3.ping_cm();
  delay(30);
  int U2 = sonar4.ping_cm();
  //lcd.setCursor(0, 1);

  while ((U1 - U2) != 0)
  {
    if ((U1 - U2) < 0)
    {
      EncDerE.write(0);
      int Enc = EncDerE.read();
      Dif = -50;
      Izquierda();
      while (Enc > ((const90 * 1 / 90) * -1 ))
      {
        Enc = EncDerE.read();
      }
      Detenerse();
      Dif = 0;
      //delay(90);
    }
    else if ((U1 - U2) > 0)
    {
      EncDerE.write(0);
      int Enc = EncDerE.read();
      Dif = -50;
      Derecha();
      while (Enc < (const90 * 1 / 90))
      {
        Enc = EncDerE.read();
      }
      Detenerse();
      Dif = 0;
      //delay(90);
    }


    //delay(30);
    U1 = sonar3.ping_cm();
    delay(30);
    U2 = sonar4.ping_cm();
    delay(30);
  }
}

void ultMov()
{
  byte count = 0;
  byte U1 = sonar7.ping_cm();
  delay(50);
  byte U2 = sonar8.ping_cm();
  delay(30);
  if (U1 != 0 && U2 != 0 && count == 0)
  {
    UltIzq();
    count++;
  }
  U1 = sonar3.ping_cm();
  delay(30);
  U2 = sonar4.ping_cm();
  delay(30);
  if (U1 != 0 && U2 != 0 && count == 0)
  {
    UltDer();
    count++;
  }
}

void Ultacomodo()
{
  if (ParedIzq())
  {
    UltIzq();
  }
  else if (ParedDer())
  {
    UltDer();
  }
}


void Acejarse2()
{
  if (ParedIzq())
  {
    AcejarseIzquierda();
  }
  else if (ParedDer())
  {
    AcejarseDerecha();
  }
}

//Baja la rampa detectando si hay victima en ella
void RampaB()
{
  Blink();
  unsigned long tim2;
  unsigned long tim3;
  int MPU = MPUP();
  unsigned long tim = millis();
  do
  {
    lcd.clear();
    byte velMD = 10;
    byte velMI = 0;
    do {
      Adelante();
      MPU = MPUP();
      lcd.setCursor(0, 1);
      lcd.print(MPU);
      tim2 = millis();
      tim3 = tim2 - tim;
      if (tim3 > 450)
      {
        Detenerse();
        delay(200);
        AcomodoRampa();
        Detenerse();
        delay(200);
        tim = millis();
      }
    } while (MPU < -2000);
    delay(500);
    MPU = MPUP();
  } while (MPU < -2500);
  Detenerse();
  lcd.setCursor(0, 1);
  lcd.print(MPU);
  Blink();
}

//Regresa cuantas posibilidades tiene de mov en numero y como booleano
byte getPossibility(bool bDir[])
{
  byte bReturn = 1;
  bDir[3] = true;

  if (!ParedDer())
  {
    bReturn++;
    bDir[0] = true;
  }
  if (!ParedEnf())
  {
    bReturn++;
    bDir[1] = true;
  }
  if (!ParedIzq())
  {
    bReturn++;
    bDir[2] = true;
  }
  return bReturn;
}

//LLena los datos de la posicion donde se encuentra
void GetDatos()
{
  Serial.println("----Datos----");
  bool bDir[4] = {false, false, false, false};
  Paso++;
  pasoActual = Paso;
  Serial.print("Paso: ");
  Serial.println(Paso);
  Pos[bZ][bX][bY] = Paso;
  Serial.print("bZ: ");
  Serial.println(bZ);
  Serial.print("bX: ");
  Serial.println(bX);
  Serial.print("bY: ");
  Serial.println(bY);
  Serial.print("Pos[bZ][bX][bY]: ");
  Serial.println(Pos[bZ][bX][bY]);
  Possibility[Paso] = getPossibility(bDir);
  Serial.print("Possibility: ");
  Serial.println(Possibility[Paso]);
  for (byte iI = 0; iI < Possibility[Paso]; iI++)
  {
    if (true == bDir[0])
    {
      if (1 == Direcc)
      {
        Run[Paso][iI] = 'x';
      }
      else if (2 == Direcc)
      {
        Run[Paso][iI] = 'p';
      }
      else if (3 == Direcc)
      {
        Run[Paso][iI] = 'y';
      }
      else if (4 == Direcc)
      {
        Run[Paso][iI] = 'o';
      }
      bDir[0] = false;
    }
    else if (true == bDir[1])
    {
      if (1 == Direcc)
      {
        Run[Paso][iI] = 'y';
      }
      else if (2 == Direcc)
      {
        Run[Paso][iI] = 'x';
      }
      else if (3 == Direcc)
      {
        Run[Paso][iI] = 'o';
      }
      else if (4 == Direcc)
      {
        Run[Paso][iI] = 'p';
      }
      bDir[1] = false;
    }
    else if (true == bDir[2])
    {
      if (1 == Direcc)
      {
        Run[Paso][iI] = 'o';
      }
      else if (2 == Direcc)
      {
        Run[Paso][iI] = 'y';
      }
      else if (3 == Direcc)
      {
        Run[Paso][iI] = 'p';
      }
      else if (4 == Direcc)
      {
        Run[Paso][iI] = 'x';
      }
      bDir[2] = false;
    }
    else if (true == bDir[3])
    {
      if (1 == Direcc)
      {
        Run[Paso][iI] = 'p';
      }
      else if (2 == Direcc)
      {
        Run[Paso][iI] = 'o';
      }
      else if (3 == Direcc)
      {
        Run[Paso][iI] = 'x';
      }
      else if (4 == Direcc)
      {
        Run[Paso][iI] = 'y';
      }
      bDir[3] = false;
    }
  }
  if (true == bARampa)
  {
    if ('a' != Run[Paso][3])
    {
      Run[Paso][3] = cRa;
    }
    else if ('a' != Run[Paso][2])
    {
      Run[Paso][2] = cRa;
    }
    else if ('a' != Run[Paso][1])
    {
      Run[Paso][1] = cRa;
    }
    else if ('a' != Run[Paso][0])
    {
      Run[Paso][0] = cRa;
    }
    bARampa = false;
  }
  for (byte iI = 0; iI < maxNegros; iI++)
  {
    for (byte iJ = 0; iJ < 4; iJ++)
    {
      int iCoord = getCoord(Run[Paso][iJ], Paso);
      if (iCoord == listaX[iI])
      {
        for (byte iK = iJ; iK < 4; iK++)
        {
          Run[Paso][iK] = Run[Paso][1 + iK];
        }
        Run[Paso][3] = 'a';
      }
    }
  }
  //Serial.println("----Datos----");
  //Serial.print("bZ: ");
  //Serial.println(bZ);
  //Serial.print("bX: ");
  //Serial.println(bX);
  //Serial.print("bY: ");
  //Serial.println(bY);
  //Serial.print("Pos: ");
  //Serial.println(Pos[bZ][bX][bY]);
  //Serial.print("Possibility: ");
  //Serial.println(Possibility[Paso]);
  Serial.print("Run 1: ");
  Serial.println(Run[Paso][0]);
  Serial.print("Run 2: ");
  Serial.println(Run[Paso][1]);
  Serial.print("Run 3: ");
  Serial.println(Run[Paso][2]);
  Serial.print("Run 4: ");
  Serial.println(Run[Paso][3]);
}

/*Regresa true si ya se ha estado antes en el Run que se le manda
  como parametro, cRun, y false si no
*/
bool BeenHere(char cRun, byte iPaso)
{
  //Variables que almacenan lo que se va a regresar, X, Y y Z del Paso que se investiga
  bool bReturn = false;
  byte iX = 255;
  byte iY = 255;
  byte iZ = 255;

  //Busca en todos los pasos el iPaso para conseguir X, Y y Z
  for (byte iI = 0; iI < ZZ; iI++)
  {
    for (byte iJ = 0; iJ < XX; iJ++)
    {
      for (byte iK = 0; iK < YY; iK++)
      {
        if (iPaso == Pos[iI][iJ][iK])
        {
          iZ = iI;
          iX = iJ;
          iY = iK;
        }
        if (255 != iX)
          break;
      }
      if (255 != iX)
        break;
    }
    if (255 != iX)
      break;
  }
  //Modifica X o Y dependiendo del cRun
  if ('x' == cRun)
  {
    iX += 1;
  }
  else if ('y' == cRun)
  {
    iY += 1;
  }
  else if ('o' == cRun)
  {
    iX -= 1;
  }
  else if ('p' == cRun)
  {
    iY -= 1;
  }
  else if ('r' == cRun)
  {
    iZ -= 1;
    iX -= 1;
  }
  else if ('s' == cRun)
  {
    iZ -= 1;
    iX += 1;
  }
  else if ('t' == cRun)
  {
    iZ -= 1;
    iY += 1;
  }
  else if ('u' == cRun)
  {
    iZ -= 1;
    iY -= 1;
  }
  else if ('e' == cRun)
  {
    iZ += 1;
    iY -= 1;
  }
  else if ('b' == cRun)
  {
    iZ += 1;
    iX += 1;
  }
  else if ('d' == cRun)
  {
    iZ += 1;
    iY += 1;
  }
  else if ('c' == cRun)
  {
    iZ += 1;
    iX -= 1;
  }
  //Busca si ya se ha estado en esa coordenada
  if (255 != Pos[iZ][iX][iY])
  {
    bReturn = true;
  }
  return bReturn;
}

//Consgiue el numero de paso del Run que se da como parametro
byte getPass(char cRun, byte iPaso)
{
  byte bReturn = 255;
  byte iX = 255;
  byte iY = 255;
  byte iZ = 255;
  //Busca en todos los pasos el iPaso para conseguir X, Y y Z
  for (byte iI = 0; iI < ZZ; iI++)
  {
    for (byte iJ = 0; iJ < XX; iJ++)
    {
      for (byte iK = 0; iK < YY; iK++)
      {
        if (iPaso == Pos[iI][iJ][iK])
        {
          iZ = iI;
          iX = iJ;
          iY = iK;
        }
        if (255 != iX)
          break;
      }
      if (255 != iX)
        break;
    }
    if (255 != iX)
      break;
  }
  //Modifica X o Y dependiendo del cRun
  if ('x' == cRun)
  {
    iX += 1;
  }
  else if ('y' == cRun)
  {
    iY += 1;
  }
  else if ('o' == cRun)
  {
    iX -= 1;
  }
  else if ('p' == cRun)
  {
    iY -= 1;
  }
  else if ('r' == cRun)
  {
    iZ -= 1;
    iX -= 1;
  }
  else if ('s' == cRun)
  {
    iZ -= 1;
    iX += 1;
  }
  else if ('t' == cRun)
  {
    iZ -= 1;
    iY += 1;
  }
  else if ('u' == cRun)
  {
    iZ -= 1;
    iY -= 1;
  }
  else if ('e' == cRun)
  {
    iZ += 1;
    iY -= 1;
  }
  else if ('b' == cRun)
  {
    iZ += 1;
    iX += 1;
  }
  else if ('d' == cRun)
  {
    iZ += 1;
    iY += 1;
  }
  else if ('c' == cRun)
  {
    iZ += 1;
    iX -= 1;
  }
  bReturn = Pos[iZ][iX][iY];
  return bReturn;
}

int getCoord(char cRun, byte iPaso)
{
  int iReturn = 255;
  byte iX = 255;
  byte iY = 255;
  byte iZ = 255;
  //Busca en todos los pasos el iPaso para conseguir X, Y y Z
  for (byte iI = 0; iI < ZZ; iI++)
  {
    for (byte iJ = 0; iJ < XX; iJ++)
    {
      for (byte iK = 0; iK < YY; iK++)
      {
        if (iPaso == Pos[iI][iJ][iK])
        {
          iZ = iI;
          iX = iJ;
          iY = iK;
        }
        if (255 != iX)
          break;
      }
      if (255 != iX)
        break;
    }
    if (255 != iX)
      break;
  }
  //Modifica X o Y dependiendo del cRun
  if ('x' == cRun)
  {
    iX += 1;
  }
  else if ('y' == cRun)
  {
    iY += 1;
  }
  else if ('o' == cRun)
  {
    iX -= 1;
  }
  else if ('p' == cRun)
  {
    iY -= 1;
  }
  else if ('r' == cRun)
  {
    iZ -= 1;
    iX -= 1;
  }
  else if ('s' == cRun)
  {
    iZ -= 1;
    iX += 1;
  }
  else if ('t' == cRun)
  {
    iZ -= 1;
    iY += 1;
  }
  else if ('u' == cRun)
  {
    iZ -= 1;
    iY -= 1;
  }
  else if ('e' == cRun)
  {
    iZ += 1;
    iY -= 1;
  }
  else if ('b' == cRun)
  {
    iZ += 1;
    iX += 1;
  }
  else if ('d' == cRun)
  {
    iZ += 1;
    iY += 1;
  }
  else if ('c' == cRun)
  {
    iZ += 1;
    iX -= 1;
  }
  iReturn = (iZ * 10000) + (iX * 100) + iY;
  return iReturn;
}

//Regresa true si se esta en una rampa
bool Rampa()
{
  bool bReturn = false;
  int iMPUP = MPUP();
  if (iMPUP > 4000 || iMPUP < -2500)
  {
    bReturn = true;
  }
  return bReturn;
}

//Cruza la rampa, ACTUALIZAR bZ aqui!!
void MovRampa()
{
  Serial.println("----Rampa----");
  Serial.print("brRampa, paso: ");
  Serial.println(brRampa);
  Serial.print("biI, run: ");
  Serial.println(biI);
  Serial.println("Run[brRampa][biI]");
  Serial.print("De: ");
  Serial.println(Run[brRampa][biI]);
  //Serial.print("bZ: ");
  //Serial.println(bZ);
  int iMPUP = MPUP();
  //Si detecta que hay que subir la rampa
  if (bRampaArriba == true)
  {
    if (true == Explore)
    {
      if ('x' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 'b';
        cRa = 'r';
      }
      else if ('y' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 'd';
        cRa = 'u';
      }
      else if ('o' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 'c';
        cRa = 's';
      }
      else if ('p' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 'e';
        cRa = 't';
      }
    }
    //AcomodoRampa();
    RampaS();
    Detenerse();
    GiroDer90();
    Acomodo();
    GiroDer90();
    Acomodo();
    bZ += 1;
    bRampaArriba = false;
  }
  //Si esta bajando la rampa
  else if (iMPUP < -2000)
  {
    if (true == Explore)
    {
      if ('x' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 's';
        cRa = 'c';
      }
      else if ('y' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 't';
        cRa = 'e';
      }
      else if ('o' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 'r';
        cRa = 'b';
      }
      else if ('p' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 'u';
        cRa = 'd';
      }
    }
    AcomodoRampa();
    RampaB();
    Detenerse();
    bZ -= 1;
  }
  Serial.print("a: ");
  Serial.println(Run[brRampa][biI]);
}

//Se mueve de la coordenada actuar(iCoordAc) a la coordenada deseada(icCoord)
void Move(int iCoordAc, int icCoord)
{
  Serial.println("---Move---");
  Serial.print("Coordenada Actual: ");
  Serial.println(iCoordAc);
  Serial.print("Coordenada deseada: ");
  Serial.println(icCoord);
  //Copia de iCoordAc para la rampa
  int CopCoordAc = iCoordAc;
  //Copia de icCoord para los hoyos negros
  int CopcCoord = icCoord;

  //Se quita el dato del piso de la coordenada actual y a la que se quiere llegar
  iCoordAc = iCoordAc % 10000;
  icCoord = icCoord % 10000;

  //Copias de X, Y y Z
  byte bCX = bX;
  byte bCY = bY;
  byte bCZ = bZ;

  //Busca a que dirección moverse actualiza la variable correcta de X oY y Direcc
  if (iCoordAc == icCoord - 100)
  {
    if (Direcc == 1)
    {
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 2)
    {
      Adelante30();
    }
    else if (Direcc == 3)
    {
      GiroDer90();
      Acomodo();
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 4)
    {
      GiroIzq90();
      Acomodo();
      Adelante30();
    }
    bX += 1;
    Direcc = 2;
  }
  else if (iCoordAc == icCoord - 1)
  {
    if (Direcc == 1)
    {
      Adelante30();
    }
    else if ( Direcc == 2)
    {
      GiroIzq90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 3)
    {
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 4)
    {
      GiroDer90();
      Acomodo();
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    bY += 1;
    Direcc = 1;
  }
  else if (iCoordAc == icCoord + 100)
  {
    if (Direcc == 1)
    {
      GiroIzq90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 2)
    {
      GiroDer90();
      Acomodo();
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 3)
    {
      Adelante30();
    }
    else if (Direcc == 4)
    {
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    bX -= 1;
    Direcc = 3;
  }
  else if (iCoordAc == icCoord + 1)
  {
    if (Direcc == 1)
    {
      GiroDer90();
      Acomodo();
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 2)
    {
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 3)
    {
      GiroIzq90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 4)
    {
      Adelante30();
    }
    bY -= 1;
    Direcc = 4;
  }
  Detenerse();
  //delay(100);
  //si esa nueva coordenada es un hoyo negro regresa y busca a donde moverse
  if (HoyoNegro())
  {
    Serial.println("Negro");
    listaX[subNegro] = CopcCoord;
    subNegro += 1;
    //Serial.println("----Entra HoyoNegro----");
    bool bListo = false;
    Atras30();
    Acomodo();
    bX = bCX;
    bY = bCY;
    byte iCPaso = Pos[bZ][bX][bY];
    //Serial.print("bZ: ");
    //Serial.println(bZ);
    //Serial.print("bX: ");
    //Serial.println(bX);
    //Serial.print("bY: ");
    //Serial.println(bY);
    //Serial.print("Pos[bZ][bX][bY]= ");
    //Serial.println(Pos[bZ][bX][bY]);
    for (byte iI = 0; iI < Possibility[iCPaso]; iI++)
    {
      //Serial.print("icCoord: ");
      //Serial.println(icCoord);
      int iThis = getCoord(Run[iCPaso][iI], iCPaso);
      //Serial.print("iThis #");
      //Serial.print(iI);
      //Serial.print(": ");
      //Serial.println(getCoord(Run[iCPaso][iI], iCPaso));
      if (iThis == CopcCoord)
      {
        for (byte iJ = iI; iJ < 4; iJ++)
        {
          Run[iCPaso][iJ] = Run[iCPaso][1 + iJ];
          bListo = true;
        }
      }
      if (bListo == true)
      {
        break;
      }
    }
    Run[iCPaso][3] = 'a';
    Possibility[iCPaso] -= 1;
    pasoActual = iCPaso;
    //Serial.print("iCPaso: ");
    //Serial.println(iCPaso);
    //Serial.print("Run 1: ");
    //Serial.println(Run[iCPaso][0]);
    //Serial.print("Run 2: ");
    //Serial.println(Run[iCPaso][1]);
    //Serial.print("Run 3: ");
    //Serial.println(Run[iCPaso][2]);
    //Serial.print("Run 4: ");
    //Serial.println(Run[iCPaso][3]);
    //Serial.print("Possbility: ");
    //Serial.println(Possibility[iCPaso]);
    SearchRouteAndMove();
  }
  //Si esa nueva coordenada es la rampa, la cruza y actualiza bZ
  else if (Rampa())
  {
    Serial.println("Rampa");
    MovRampa();
    bARampa = true;
  }
  Serial.println("acomodo");
  Acomodo();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(bZ);
  lcd.print(" ");
  lcd.print(bX);
  lcd.print(" ");
  lcd.print(bY);
}


//Se mueve hasta el paso que se tiene como parametro
void moveUntil(byte bHere)
{
  Serial.println("---MoveUntil---");
  Serial.print("Ir hasta el paso #");
  Serial.println(bHere);
  byte counter = 0;
  //copia del paso actual
  byte bCPaso = pasoActual;
  //Hasta no encontrarse en el paso deseado se sigue moviendo
  while (bCPaso != bHere)
  {
    /*variables, paso más cercano, posible paso más cercano,
      Paso antiguo, Paso, Coordenada Actual, Coordenada deseada, posible
      coordenada deseada
    */
    byte bClose = 255;
    byte bPosib = 255;
    byte bPassAntiguo = 255;
    int iPass = 255;
    int iCoordAc = 255;
    int icCoord = 255;
    /*Primero revisa los Pasos del los Run de donde te encuentras
      y escoge el Paso más cercano al bHere
    */
    for (byte iI = 0; iI < Possibility[bCPaso]; iI++)
    {
      bPosib = getPass(Run[bCPaso][iI], bCPaso);
      iPass = bPosib - bHere;
      if (iPass < 0)
      {
        iPass *= -1;
      }
      if (iPass < bPassAntiguo)
      {
        bPassAntiguo = iPass;
        bClose = bPosib;
        icCoord = getCoord(Run[bCPaso][iI], bCPaso);
      }
    }
    iCoordAc = (bZ * 10000) + (bX * 100) + bY;
    Move(iCoordAc, icCoord);
    bCPaso = bClose;
    counter++;
    if (counter > 0)
    {
      bARampa = false;
    }
  }
}

//Regresa el punto de inicio
void extractionPoint()
{
  moveUntil(1);
  Detenerse();
  lcd.clear();
  lcd.write("Fin");
  delay(30000);
}

//Se mueve a la coordenada desconocida
void exploreNewWorlds(byte bHere)
{
  Explore = true;
  Serial.println("----exploreNewWorlds----");
  Serial.print("Se encuentra en Paso: ");
  Serial.println(bHere);
  byte iCounter = 0;
  for (byte iI = 0; iI < Possibility[bHere]; iI++)
  {
    int iCoordAc = 9999;
    int icCoord = 9999;
    bool bCheck = BeenHere(Run[bHere][iI], bHere);
    if (bCheck == false && iCounter == 0)
    {
      Serial.print("No ha estado en: ");
      Serial.println(getCoord(Run[bHere][iI], bHere));
      lcd.setCursor(7, 1);
      lcd.print("c ");
      lcd.print(getCoord(Run[bHere][iI], bHere));
      Serial.print("Del Paso: ");
      Serial.println(bHere);
      Serial.print("Posibilidad #");
      Serial.println(iI);
      icCoord = getCoord(Run[bHere][iI], bHere);
      brRampa = bHere;
      biI = iI;
      for (byte iI = 0; iI < ZZ; iI++)
      {
        for (byte iJ = 0; iJ < XX; iJ++)
        {
          for (byte iK = 0; iK < YY; iK++)
          {
            if (Pos[iI][iJ][iK] == bHere)
            {
              iCoordAc = (iI * 10000) + (iJ * 100) + iK;
            }
            if (iCoordAc != 9999)
            {
              break;
            }
          }
          if (iCoordAc != 9999)
          {
            break;
          }
        }
        if (iCoordAc != 9999)
        {
          break;
        }
      }
      //Serial.print("Coordenada Actual: ");
      //Serial.println(iCoordAc);
      //Serial.print("Coordenada Deseada: ");
      //Serial.println(icCoord);
      Move(iCoordAc, icCoord);
      iCounter++;
    }
  }
  Explore = false;
}

//Busca el paso al cual llegar para despues moverse a la coordenada desconocida
byte WhereToGo()
{
  Serial.println("---WhereToGo---");
  //copia del paso actual, de x y de y
  byte bCPaso = pasoActual;
  byte bCX = bX;
  byte bCY = bY;
  //paso al cual llegar
  byte bHere = 255;

  //busca la coordendada desconocida por todos los Run concidos
  for (byte iI = bCPaso; iI > 0; iI--)
  {
    for (byte iJ = 0; iJ < Possibility[iI]; iJ++)
    {
      if (!BeenHere(Run[iI][iJ], iI))
      {
        bool Revision = false;
        for (byte iK = 0; iK < maxNegros; iK++)
        {
          int coordenada = getCoord(Run[iI][iJ], iI);
          if (listaX[iK] == coordenada)
          {
            Revision = true;
          }
        }
        if (Revision == false)
        {

          bHere = iI;
          Serial.print("No ha estado en un Run del paso #");
          Serial.println(iI);
          Serial.print("Numero de Run: ");
          Serial.println(iJ);
          Serial.print("Coordenada: ");
          Serial.println(getCoord(Run[iI][iJ], iI));
        }
      }
      if (255 != bHere)
        break;
    }
    if (255 != bHere)
      break;
  }
  //Significa que se ha acabado de recorrer el laberinto
  if (255 == bHere)
  {
    extractionPoint();
  }
  return bHere;
}


//Busca la ruta y se mueve hasta la posicion desconocida
void SearchRouteAndMove()
{
  Serial.println("---SearchAndMove---");
  byte bData = WhereToGo();
  lcd.setCursor(0, 1);
  lcd.print("ir #");
  lcd.print(bData);
  moveUntil(bData);
  exploreNewWorlds(bData);
}

//Funcion a llamar para completar el laberinto
void Laberinto()
{
  Serial.println("------Laberinto-------");
  GetDatos();
  SearchRouteAndMove();
}

void AcejarseEnfrente2()
{
  for (byte iI = 0; iI < 2; iI++)
  {
    byte Sharp = SharpEn.distance();
    ////Serial.println(Sharp);
    if (Sharp < 48 && Sharp > 20)
    {
      do {
        Adelante5();
        byte Sharp2 = SharpEn.distance();
        if (Sharp2 > Sharp)
        {
          Atras();
          //delay(100);
          Detenerse();
        }
        else
        {
          Adelante5();
        }
        Sharp = SharpEn.distance();
      } while (Sharp < 48 && Sharp > 20);
    }
    else if (Sharp > 14 && Sharp < 21)
    {
      Atras();
      //delay(100);
      Detenerse();
      byte Sharp2 = SharpEn.distance();
      if (Sharp2 > Sharp)
      {
        Adelante();
        //delay(100);
        Detenerse();
      }
    }
  }
}

void RampaS()
{
  Blink();
  int MPU;
  Atras();
  do
  {
    //Atras2(cambioDer, cambioIzq);
    if (ParedDer())
    {
      byte filtro[5];
      for (byte iI = 0; iI < 5; iI++)
      {
        filtro[iI] = sonar3.ping_cm();
        delay(50);
      }
      BubbleSort5(filtro);
      byte dist = filtro[2];
      Serial.print(dist);
      for (byte iI = 0; iI < 5; iI++)
      {
        filtro[iI] = sonar4.ping_cm();
        delay(50);
      }
      BubbleSort5(filtro);
      byte dist2 = filtro[2];
      Serial.print("\t");
      Serial.println(dist2);
      if (dist < 4 || dist2 < 4)
      {
        Serial.println("Izquierda");
        IzquierdaDiagonalAtras();
        //cambioDer = 15;
        cambioIzq = 0;
      }
      else if (dist > 4 || dist2 > 4)
      {
        Serial.println("Derecha");
        DerechaDiagonalAtras();
        cambioDer = 0;
        //cambioIzq = 15;
      }
      else {
        Serial.println("Atras");
        Atras();
        cambioDer = 0;
        cambioIzq = 0;
      }
    }
    MPU = MPUP();
  } while (MPU < -2500);
  Detenerse();
  lcd.setCursor(0, 1);
  lcd.print(MPU);
  //delay(2000);
  Blink();
}

void Evadir()
{
  for (byte iI = 0; iI < 2; iI++)
  {
    byte u1 = sonar1.ping_cm();
    delay(50);
    byte u2 = sonar2.ping_cm();
    if (u1 != 0 && u2 == 0)
    {
      DerechaM();
      delay(200);
      Detenerse();
    }
    else if (u2 != 0 && u1 == 0)
    {
      IzquierdaM();
      delay(200);
      Detenerse();
    }
    else
    {
      iI = 2;
    }
  }
}

void Calor()
{
  therm1.read();
  int Calor = therm1.object();
  CalibCalor = Calor + 3;
  bInicio = false;
}

void BubbleSort5(byte num[])
{
  int i, j, flag = 1;    // set flag to 1 to start first pass
  int temp;             // holding variable
  int numLength = 5;
  for (i = 1; (i <= numLength) && flag; i++)
  {
    flag = 0;
    for (j = 0; j < (numLength - 1); j++)
    {
      if (num[j + 1] > num[j])    // ascending order simply changes to <
      {
        temp = num[j];             // swap elements
        num[j] = num[j + 1];
        num[j + 1] = temp;
        flag = 1;               // indicates that a swap occurred.
      }
    }
  }
}

void loop()
{
  Atras();
  //Adelante();
}
