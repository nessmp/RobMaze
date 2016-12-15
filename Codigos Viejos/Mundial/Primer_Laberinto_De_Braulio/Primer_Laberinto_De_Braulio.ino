#include <ZX_Sensor.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <Sharp.h>

byte PROGMEM pinSharpDerEnf = 2;
Sharp SharpDerEnf(pinSharpDerEnf, 30); //Sharp de la Derecha Enf en el pin 0 con dist 30
byte PROGMEM pinSharpDerAtr = 0;
Sharp SharpDerAtr(pinSharpDerAtr, 30); //Sharp de la Derecha Atr en el pin 1 con dist 30
byte PROGMEM pinSharpIzqEnf = 3;
Sharp SharpIzqEnf(pinSharpIzqEnf, 30); //Sharp de la Izquierda Enf en el pin 2 con dist 30
byte PROGMEM pinSharpIzqAtr = 1;
Sharp SharpIzqAtr(pinSharpIzqAtr, 30); //Sharp de la Izquierda Atr en el pin 3 con dist 30

Adafruit_MotorShield MotDerEnf = Adafruit_MotorShield();
Adafruit_MotorShield MotDerAtr = Adafruit_MotorShield();

Adafruit_DCMotor *MDE = MotDerEnf.getMotor(3);
Adafruit_DCMotor *MDA = MotDerAtr.getMotor(1);

Adafruit_MotorShield MotIzqEnf = Adafruit_MotorShield();
Adafruit_MotorShield MotIzqAtr = Adafruit_MotorShield();

Adafruit_DCMotor *MIE = MotIzqEnf.getMotor(4);
Adafruit_DCMotor *MIA = MotIzqAtr.getMotor(2);

ZX_Sensor ZXDer = ZX_Sensor(0x10);

ZX_Sensor ZXIzq = ZX_Sensor(0x11);

Encoder encoder(2, 3);

int trigger2 = 8;
int echo2 = 7;

int trigger1 = 10;
int echo1 = 9;

int const30 =  50;

byte velMIA = 74;//72
byte velMIE = 156; //152
byte velMDA = 160;
byte velMDE = 72;
byte Dif = 0;

void setup() {
  Serial.begin(9600);

  ZXDer.init();
  ZXIzq.init();

  MotDerEnf.begin();
  MotDerAtr.begin();
  MotIzqEnf.begin();
  MotIzqAtr.begin();

  MDE->run(RELEASE);
  MDA->run(RELEASE);
  MIE->run(RELEASE);
  MIA->run(RELEASE);

  pinMode(11, INPUT);
  pinMode(trigger1, OUTPUT); /*activación del pin 9 como salida: para el pulso ultrasónico*/
  pinMode(echo1, INPUT); /*activación del pin 8 como entrada: tiempo del rebote del ultrasonido*/
  pinMode(trigger2, OUTPUT); /*activación del pin 9 como salida: para el pulso ultrasónico*/
  pinMode(echo2, INPUT); /*activación del pin 8 como entrada: tiempo del rebote del ultrasonido*/
}

//Funcion que pone a los motores para avanzar
void Adelante()
{
  MDE->run(FORWARD);
  MDA->run(FORWARD);
  MIE->run(FORWARD);
  MIA->run(FORWARD);

  MDE->setSpeed(velMDE);
  MDA->setSpeed(velMDA);
  MIE->setSpeed(velMIE);
  MIA->setSpeed(velMIA);
}

//Funcion que detiene los motores
void Detenerse()
{
  MDE->run(RELEASE);
  MDA->run(RELEASE);
  MIE->run(RELEASE);
  MIA->run(RELEASE);
}

//Funcion que pone a los motores para ir de reversa
void Atras()
{
  MDE->run(BACKWARD);
  MDA->run(BACKWARD);
  MIE->run(BACKWARD);
  MIA->run(BACKWARD);

  MDE->setSpeed(velMIA);
  MDA->setSpeed(velMDA);
  MIE->setSpeed(velMDE);
  MIA->setSpeed(velMIE);
}

//analogWrite de los motores para girar a la derecha
void Derecha()
{
  MDE->run(BACKWARD);
  MDA->run(BACKWARD);
  MIE->run(FORWARD);
  MIA->run(FORWARD);

  MDE->setSpeed(velMIA);
  MDA->setSpeed(velMDA);
  MIE->setSpeed(velMDE);
  MIA->setSpeed(velMIE);
}
//analogWrite de los motores para girar a la izquierda
void Izquierda()
{
  MDE->run(FORWARD);
  MDA->run(FORWARD);
  MIE->run(BACKWARD);
  MIA->run(BACKWARD);

  MDE->setSpeed(velMIA);
  MDA->setSpeed(velMDA);
  MIE->setSpeed(velMDE);
  MIA->setSpeed(velMIE);
}

void GiroIzq90()
{
  Serial.println("Entra GiroIzq90");
  int counter = 0;
  Serial.println(counter);
  encoder.write(0);
  long newPosition  = encoder.read();
  int oldPosition  = newPosition;
  Izquierda();
  while (counter < 100)
  {
    newPosition = encoder.read();
    if (newPosition != oldPosition)
    {
      oldPosition = newPosition;
      counter++;
    }
    Serial.println(counter);
  }
  Detenerse();
}
void GiroDer90()
{
  Serial.println("Entra GiroIzq90");
  int counter = 0;
  Serial.println(counter);
  encoder.write(0);
  long newPosition  = encoder.read();
  int oldPosition  = newPosition;
  Derecha();
  while (counter < 100)
  {
    newPosition = encoder.read();
    if (newPosition != oldPosition)
    {
      oldPosition = newPosition;
      counter++;
    }
    Serial.println(counter);
  }
  Detenerse();
}
void Acomodo() {}

void Adelante30()
{
  Serial.println("Entra Adelante 30");
  int counter = 0;
  Serial.println(counter);
  encoder.write(0);
  long newPosition  = encoder.read();
  int oldPosition  = newPosition;
  Adelante();
  while (counter < 110)
  {
    newPosition = encoder.read();
    if (newPosition != oldPosition)
    {
      oldPosition = newPosition;
      counter++;
    }
    Serial.println(counter);
  }
  Detenerse();
}

void Atras30()
{
  Serial.println("Entra Atras 30");
  int counter = 0;
  Serial.println(counter);
  encoder.write(0);
  long newPosition  = encoder.read();
  int oldPosition  = newPosition;
  Atras();
  while (counter < 110)
  {
    newPosition = encoder.read();
    if (newPosition != oldPosition)
    {
      oldPosition = newPosition;
      counter++;
    }
    Serial.println(counter);
  }
  Detenerse();
}

//Regresa verdadero si detecta pared en la derecha
bool ParedDer()
{
  bool bPared = false; //Variable a regresar
  byte bX = ZXDer.readX(); //Lectura del eje X
  byte bZ = ZXDer.readZ(); //Lectura del eje Y
  Serial.print(bX);
  Serial.print("\t");
  Serial.print(bZ);
  Serial.print("\t");
  if (bZ < 9 && (bX > 118 && bX < 122)) //Si Z es pequeño y X esta entre 119 y 121 hay pared
  {
    bPared = true;
  }
  return bPared;
}

//Regresa verdadero si detecta pared en la izquierda
bool ParedIzq()
{
  bool bPared = false;
  byte dist = SharpIzqEnf.distance();
  if (dist < 24)
  {
    bPared = true;
  }
  return bPared;
  /*
    bool bPared = false; //Variable a regresar
    byte bX = ZXIzq.readX(); //Lectura del eje X
    byte bZ = ZXIzq.readZ(); //Lectura del eje Y
    Serial.print(bX);
    Serial.print("\t");
    Serial.print(bZ);
    Serial.print("\t");
    if (bZ < 30 && (bX > 118 && bX < 122)) //Si Z es pequeño y X esta entre 119 y 121 hay pared
    {
    bPared = true;
    }
    return bPared;
  */
}

bool ParedEnf()
{
  bool bPared = false;
  digitalWrite(10, LOW);
  delayMicroseconds(5);
  digitalWrite(10, HIGH);
  delayMicroseconds(10);
  int tiempo = pulseIn(9, HIGH);
  int distancia1 = int(0.017 * tiempo);
  delay(30);
  digitalWrite(8, LOW);
  delayMicroseconds(5);
  digitalWrite(8, HIGH);
  delayMicroseconds(10);
  tiempo = pulseIn(7, HIGH);

  int distancia2 = int(0.017 * tiempo);

  Serial.print(distancia1);
  Serial.print("\t");
  Serial.print(distancia2);
  Serial.print("\t");

  if(distancia1 < 20 && distancia2 < 20)
  {
    bPared = true;
  }
  return bPared;
}

bool negro()
{
  bool bNegro = false;
  int color = pulseIn(11, LOW);
  if (color > 1200)
  {
    bNegro = true;
  }
  return bNegro;
}

void SeguirDerecha()
{
  bool Pd = ParedDer();
  bool Pi;
  bool Pe;

  if (false == Pe)
  {
    GiroDer90();
    delay(100);
    Acomodo();
    delay(100);
  }
  else
  {
    Pe = ParedEnf();
    if (false == Pe)
    {
    }
    else
    {
      Pi = ParedIzq();
      if (false == Pi)
      {
        GiroIzq90();
        delay(100);
        Acomodo();
        delay(100);
      }
      else
      {
        GiroDer90();
        delay(100);
        Acomodo();
        delay(100);
        GiroDer90();
        delay(100);
        Acomodo();
        delay(100);
      }
    }
  }
  Adelante30();
  if (negro())
  {
    Atras30();
    delay(100);
    Acomodo();
    delay(100);
    Pi = ParedIzq();
    if (false == Pi)
    {
      GiroIzq90();
      delay(100);
      Acomodo();
      delay(100);
      Adelante30();
    }
    else
    {
      GiroDer90();
      delay(100);
      Acomodo();
      delay(100);
      GiroDer90();
      delay(100);
      Acomodo();
      delay(100);
      Adelante30();
    }
  }
  Acomodo();
  delay(100);
}

void loop() {
  digitalWrite(8, LOW);
  delayMicroseconds(5);
  digitalWrite(8, HIGH);
  delayMicroseconds(10);
  int tiempo = pulseIn(7, HIGH);
  Serial.println(tiempo);
  int distancia2 = int(0.017 * tiempo);
  Serial.println(distancia2);
  delay(200);
  /*
  Serial.println(ParedEnf());
  /*
     Rampa();
  */
}
