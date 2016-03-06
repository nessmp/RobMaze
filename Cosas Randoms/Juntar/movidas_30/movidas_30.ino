#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
#define MAX_DISTANCE 200
#include <Encoder.h>

Encoder myEnc(2, 3); //atras derecha
Encoder myEnc2(18, 19);
Encoder myEnc3(20, 21);

long oldPosition  = -999;

//Ultrasonico enfrente A
byte TriggEA = 44;
byte EchoEA = 42;

NewPing UltEA(TriggEA, EchoEA, MAX_DISTANCE);

//Ultrasonico Enfrente B

byte TriggEB = 46;
byte EchoEB = 48;

NewPing UltEB(TriggEB, EchoEB, MAX_DISTANCE);

//Ultrasonico Derecha A

byte TriggDA = 52;
byte EchoDA = 50;

NewPing UltDA(TriggDA, EchoDA, MAX_DISTANCE);

//Ultrasonico Derecha B

byte TriggDB = 24;
byte EchoDB = 22;

NewPing UltDB(TriggDB, EchoDB, MAX_DISTANCE);

//Ultrasonico Atras A

byte TriggAA = 28;
byte EchoAA = 26;

NewPing UltAA(TriggAA, EchoAA, MAX_DISTANCE);

//Ultrasonico Atrás B

byte TriggAB = 30;
byte EchoAB = 32;

NewPing UltAB(TriggAB, EchoAB, MAX_DISTANCE);

//Ultrasonico Izquierda A

byte TriggIA = 36;
byte EchoIA = 34;

NewPing UltIA(TriggIA, EchoIA, MAX_DISTANCE);

//Ultrasonico Izquierda B

byte TriggIB = 40;
byte EchoIB = 38;

NewPing UltIB(TriggIB, EchoIB, MAX_DISTANCE);

//Pantalla LCD

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

/////////////////////////
//pines de los motores///
/////////////////////////

byte motDerE1 = 5;
byte motDerE2 = 4;

byte motDerA1 = 6;
byte motDerA2 = 7;

byte motIzqE1 = 8;
byte motIzqE2 = 9;

byte motIzqA1 = 10;
byte motIzqA2 = 11;


void setup() {
  Serial.begin(9600);
  
  lcd.begin(16,2);
  lcd.backlight();
  
  ////////////////////////
  //Setup de los motores//
  ////////////////////////
  
  pinMode(motDerE1, OUTPUT);
  pinMode(motDerE2, OUTPUT);
  
  pinMode(motDerA1, OUTPUT);
  pinMode(motDerA2, OUTPUT);

  pinMode(motIzqE1, OUTPUT);
  pinMode(motIzqE2, OUTPUT);

  pinMode(motIzqA1, OUTPUT);
  pinMode(motIzqA2, OUTPUT);
}

///////////////////////////
//funciones de movimiento//
///////////////////////////

//Funcion para apagar motores
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

//funcion para moverse hacia adelante
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


int Encoder1()
{

  long newPosition = myEnc.read();


  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);

  }
  return newPosition;
}


void GiroDer90()
{
    int const90 = 4550;
  
  myEnc.write(0);
 while (Encoder1() > const90*-1)
  {
    Derecha();
    Encoder1();
  }
   Detenerse();
}


void GiroIsq90()
{
  int const90 = 4550;

  myEnc.write(0);

 
 
  while (Encoder1() < const90)
  {
    Izquierda();
    Encoder1();
  }
   Detenerse();
}

void DerechaAcomodo()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 255);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 255);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 0);
}

void IzquierdaAcomodo()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 255);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 255);
}

void Acomodarse()
{
  int UltA = UltDA.ping_cm();
  int UltB = UltDB.ping_cm();
  if(UltA > UltB + 1 || UltB > UltA +1)
  {
    do{
      if(UltA > UltB + 1)
      {
        DerechaAcomodo(); 
        delay(200);
      }
      else if(UltB > UltA +1)
      {
        IzquierdaAcomodo(); 
        delay(200);
      }
    }while(UltA > UltB + 1 || UltB > UltA +1);
  }
}

void Adelante30()
{
  int const30=5700;

  myEnc.write(0);

  while (Encoder1() < const30)
  {
    Adelante();
    Encoder1();
  }
   Detenerse();
}

void Atras30()
{
    int const30=5700;

  myEnc.write(0);

  while (Encoder1() > const30*-1)
  {
    Atras();
    Encoder1();
  }
   Detenerse();
}



void loop() {
 delay(1000);
 GiroDer90();
 delay(100);
 GiroDer90();

  
}

