#include <Wire.h> // I2C library, required for MLX90614
#include <Encoder.h> //para que crees...
#include <LiquidCrystal_I2C.h> //para la pantalla LCD
#include <NewPing.h> //ultrasonicos
#include <SharpIR.h> //sharps
#include <Servo.h>


#define MAX_DISTANCE 200 //distancia max detectada por los ultrasonicos
#define model 1080 //modelo del sharp GP2Y0A21Y

//Sensor de color
int s0  = 32;
int s1  = 34;
int s2  = 36;
int s3  = 38;
int out  = 40;

byte motDerE1 = 6; 
byte motDerE2 = 7; 

byte motDerA1 = 4;
byte motDerA2 = 5;

byte motIzqE1 = 8;
byte motIzqE2 = 9;

byte motIzqA1 = 10;
byte motIzqA2 = 11;

byte TriggEA = 23;
byte EchoEA = 25;

byte TriggEB = 51;
byte EchoEB = 53;

byte TriggDA  =47;
byte EchoDA =49;

byte TriggDB = 43;
byte EchoDB =45;

byte TriggAA = 39;
byte EchoAA = 41;

byte TriggAB = 35;
byte EchoAB = 37;

byte TriggIA = 31;
byte EchoIA = 33;

byte TriggIB = 27;
byte EchoIB = 29;

byte Enf = A3;
byte Der = A5;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

Encoder EncDerE(3, 14);

SharpIR SharpEn(Enf, 25, 93, model);
SharpIR SharpDe(Der, 25, 93, model);

int red = 0;  
int green = 0;  
int blue = 0;  
String colon = "";

long oldPosition  = -999;

int estable = 3280;

int const90 = 4550;

int const30=5700;

int pos = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(s0, OUTPUT);  
  pinMode(s1, OUTPUT);  
  pinMode(s2, OUTPUT);  
  pinMode(s3, OUTPUT);

  digitalWrite(s0, HIGH);  
  digitalWrite(s1, HIGH);

  pinMode(motDerE1, OUTPUT);
  pinMode(motDerE2, OUTPUT);
  
  pinMode(motDerA1, OUTPUT);
  pinMode(motDerA2, OUTPUT);

  pinMode(motIzqE1, OUTPUT);
  pinMode(motIzqE2, OUTPUT);

  pinMode(motIzqA1, OUTPUT);
  pinMode(motIzqA2, OUTPUT);

  lcd.begin(16,2); //la inicializa
  lcd.backlight(); //enciende el led de la pantalla

  pinMode (Enf, INPUT);
  pinMode (Der, INPUT);

}

bool color()  
{    
  bool Negro = false;
  digitalWrite(s2, LOW);  
  digitalWrite(s3, LOW);  
  //count OUT, pRed, RED  
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
  digitalWrite(s3, HIGH);  
  //count OUT, pBLUE, BLUE  
  blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
  digitalWrite(s2, HIGH);  
  //count OUT, pGreen, GREEN  
  green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  

  Serial.println("red: ");
  Serial.println(red);
  Serial.println("green: ");
  Serial.println(green);
  Serial.println("blue: ");
  Serial.println(blue);

  if(red && green && blue)
  {
    Negro = true;
  }

  return Negro;
}

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

//funcion para moverse hacia adelante
void Adelante()
{
  analogWrite(motDerE1, 255);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 255);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 255);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 255);
  analogWrite(motIzqA2, 0);
}

//funcion para moverse hacia atras
void Atras()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 255);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 255);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 255);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 255);

}

void DerechaM()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 255);

  analogWrite(motDerA1, 255);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 255);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 255);

}

void Derecha()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 255);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 255);

  analogWrite(motIzqE1, 255);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 255);
  analogWrite(motIzqA2, 0);

}

//funcion para moverse hacia izquierda
void IzquierdaM()
{
  analogWrite(motDerE1, 255);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 255);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 255);

  analogWrite(motIzqA1, 255);
  analogWrite(motIzqA2, 0);
}

void Izquierda()
{
  analogWrite(motDerE1, 255);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 255);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 255);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 255);
}

//Giros
void GiroDer90()
{
    
  delay(1000);
  EncDerE.write(0);
 while (Encoder1() > const90*-1)
  {
    Derecha();
    Encoder1();
  }
   Detenerse();
}

void GiroIzq90()
{
 
  delay(1000);
  EncDerE.write(0);

  while (Encoder1() < const90)
  {
    Izquierda();
    Encoder1();
  }
   Detenerse();
}

//Avances de 30


void Atras30()
{
    

  EncDerE.write(0);

  while (Encoder1() > const30*-1)
  {
    Atras();
    Encoder1();
  }
   Detenerse();
}

void Adelante30()
{
    

  EncDerE.write(0);

  while (Encoder1() > const30*1)
  {
    Atras();
    Encoder1();
  }
   Detenerse();
}


//Cuentas del encoder
int Encoder1()
  {
    long newPosition = EncDerE.read();
  
    if (newPosition != oldPosition) {
      oldPosition = newPosition;
  
    }
    return newPosition;
  }

int SharpEnf()
{
  return SharpEn.distance();
}

int SharpDer()
{
  return SharpDe.distance();
}

void Blink()
{
  for(int iI = 0; iI < 6; iI++)
  {
    lcd.noBacklight();
    delay(500);
    lcd.backlight();
    delay(500);
  }
}

void Negro()
{
  Atras30();
  GiroIzq90();
  Adelante30();
}

void AgujeroNegro()
{
  if(color() == true)
  {
    Negro();
  }
}

bool ParedDer()
{
  bool Pared = true;
  int Sharp = SharpDe.distance();
  if(Sharp > 25)
  {
    Pared = false;
  }
  return Pared;
}

bool ParedEnf()
{
  bool Pared = true;
  int Sharp = SharpEn.distance();
  if(Sharp > 25)
  {
    Pared = false;
  }
  return Pared;
}

void Rampa()
{

  int y;
  byte piny = 1;
  bool rampa;

  y = map(analogRead(piny), 0, 1023, 0, 10000);

  if (y < estable - 200 || y > estable + 200)
  {
    while(y<estable-50||y>estable+50)
    {
      Adelante();
      y = map(analogRead(piny), 0, 1023, 0, 10000);
    }
    Detenerse();
  }
}

void SeguirDerecha()
{
  bool ParedD = ParedDer();
  bool ParedE = ParedEnf();
  if (ParedD == false)
  {
    GiroDer90();
    Adelante30();
    Rampa();
  }
  else if (ParedE == false)
  {
    Adelante30();
    Rampa();
  }
  else if (ParedD==true && ParedE==true)
  {
    GiroIzq90();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  SeguirDerecha();
}
