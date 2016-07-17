#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

Servo servo;

Adafruit_MotorShield MotDerEnf = Adafruit_MotorShield();
Adafruit_MotorShield MotDerAtr = Adafruit_MotorShield();

Adafruit_DCMotor *MDE = MotDerEnf.getMotor(1);
Adafruit_DCMotor *MDA = MotDerAtr.getMotor(2);

Adafruit_MotorShield MotIzqEnf = Adafruit_MotorShield();
Adafruit_MotorShield MotIzqAtr = Adafruit_MotorShield();

Adafruit_DCMotor *MIE = MotIzqEnf.getMotor(3);
Adafruit_DCMotor *MIA = MotIzqAtr.getMotor(4);

byte velMDE = 200; //MIA
byte velMDA = 200; //MDA
byte velMIE = 200; //MDE
byte velMIA = 200; //MIE

int encoderValue = 0;

void count(void); // code for counting the increasing values of encoder ticks void setup()


void setup() {
  Serial.begin(9600);
  while(!Serial);
  servo.attach(4);

  MotDerEnf.begin();
  MotDerAtr.begin();
  MotIzqEnf.begin();
  MotIzqAtr.begin();

  MDE->run(RELEASE);
  MDA->run(RELEASE);
  MIE->run(RELEASE);
  MIA->run(RELEASE);

  pinMode(8, INPUT);

  attachInterrupt(8, count, RISING);

  encoderValue = 0;

}

void count()
{
  encoderValue++;
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

  MDE->setSpeed(velMDE);
  MDA->setSpeed(velMDA);
  MIE->setSpeed(velMIE);
  MIA->setSpeed(velMIA);
}

//analogWrite de los motores para girar a la derecha
void Derecha()
{
  MDE->run(BACKWARD);
  MDA->run(BACKWARD);
  MIE->run(FORWARD);
  MIA->run(FORWARD);

  MDE->setSpeed(velMDE);
  MDA->setSpeed(velMDA);
  MIE->setSpeed(velMIE);
  MIA->setSpeed(velMIA);
}
//analogWrite de los motores para girar a la izquierda
void Izquierda()
{
  MDE->run(FORWARD);
  MDA->run(FORWARD);
  MIE->run(BACKWARD);
  MIA->run(BACKWARD);

  MDE->setSpeed(velMDE);
  MDA->setSpeed(velMDA);
  MIE->setSpeed(velMIE);
  MIA->setSpeed(velMIA);
}

void Adelate30()
{
  encoderValue = 0;
  Adelante();
  while (encoderValue < 100)
  {
    Serial.println(encoderValue);
  }
  Detenerse();
}
void loop() {
  Adelate30();
  Serial.println(encoderValue);
  delay(1000);
}
