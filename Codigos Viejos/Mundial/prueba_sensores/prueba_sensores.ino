#include <Adafruit_MotorShield.h> //libreria del shield de los motores
#include <Encoder.h> //para que crees...
#include <Servo.h> //Yep para el servo
#include "utility/Adafruit_MS_PWMServoDriver.h" //Shield de los motores
#include "CurieIMU.h" //libreria del IMU del Curie
#include <Sharp.h> //libreria de los Sharps, duh
#include <ZX_Sensor.h> //¿Para los sensores ZX?
#include <Wire.h> //Libreria para I2C

//Motores
Adafruit_MotorShield MotDerEnf = Adafruit_MotorShield(0x60); //Motor Derecha Enfrete
Adafruit_MotorShield MotDerAtr = Adafruit_MotorShield(0x60); //Motor Derecha Atras
Adafruit_DCMotor *MDE = MotDerEnf.getMotor(1); //Apuntador al motor de la derecha enfrente
Adafruit_DCMotor *MDA = MotDerAtr.getMotor(2); //Apuntador al motor de la derecha atras
Adafruit_MotorShield MotIzqEnf = Adafruit_MotorShield(0x60); //Motor Izquierda Enfrete
Adafruit_MotorShield MotIzqAtr = Adafruit_MotorShield(0x60); //Motor Izquierda Atras
Adafruit_DCMotor *MIE = MotIzqEnf.getMotor(3); //Apuntador al motor de la izquierda enfrente
Adafruit_DCMotor *MIA = MotIzqAtr.getMotor(4); //Apuntador al motor de la izquierda atras

//Encoder
Encoder EncDerE(18, 19); //Encoder en los pines 18 y 19

//Servo
Servo tiraKits; //objeto del servo

//Sensores ZX
ZX_Sensor ZXDer(0x10); //ZX de la Der con I2C address 0x10
ZX_Sensor ZXIzq(0x11); //ZX de la Izq con I2C address 0x11

//Sharps
Sharp SharpDerEnf(2, 30); //Sharp de la Derecha Enf en el pin 0 con dist 30
Sharp SharpDerAtr(0, 30); //Sharp de la Derecha Atr en el pin 1 con dist 30
Sharp SharpIzqEnf(3, 30); //Sharp de la Izquierda Enf en el pin 2 con dist 30
Sharp SharpIzqAtr(1, 30); //Sharp de la Izquierda Atr en el pin 3 con dist 30


//Color
const int PROGMEM CalibNegro = 1000; //Constante del color negro
const byte PROGMEM out = 11; //Pin para el sensor de color

byte dif = 0; //Diferencia para hacer cambios en motores
byte velMDE = 60; //velocidad para el motor de la derecha enfrente
byte velMDA = 200; //velocidad para el motor de la derecha atras
byte velMIE = 65; //velocidad para el motor de la izquierda enfrente
byte velMIA = 200; //velocidad para el motor de la izquierda atras

void setup() {
  Serial.begin(9600);
  
  ZXDer.init(); //Inicializa sensores ZX
  ZXIzq.init();

  tiraKits.attach(8); //Servo al pin 8

  MotDerEnf.begin(); //inicializa el bus del I2C para los motores
  MotDerAtr.begin();
  MotIzqEnf.begin();
  MotIzqAtr.begin();

  MDE->run(RELEASE); //Apaga los motores, por seguridad
  MDA->run(RELEASE);
  MIE->run(RELEASE);
  MIA->run(RELEASE);

}

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

void loop() {
  Atras();
  Serial.println("----------ZX----------");
  Serial.print(ZXDer.readZ());
  Serial.print("\t");
  Serial.println(ZXDer.readX());
  Serial.print(ZXIzq.readZ());
  Serial.print("\t");
  Serial.println(ZXIzq.readX());
  Serial.println("----------Sharps----------");
  Serial.print(SharpDerEnf.distance());
  Serial.print("\t");
  Serial.println(SharpIzqEnf.distance());
  Serial.println("----------Color----------");
  Serial.println(pulseIn(11, LOW));
  tiraKits.write(0);
  delay(1000);
  tiraKits.write(180);
  delay(1000);
  Adelante();
  delay(1000);

}
