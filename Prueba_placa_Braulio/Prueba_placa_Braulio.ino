#include <Adafruit_MotorShield.h> //libreria del shield de los motores
#include <Encoder.h> //para que crees...
#include <Servo.h> //Yep para el servo
#include "utility/Adafruit_MS_PWMServoDriver.h" //Shield de los motores
#include "CurieIMU.h" //libreria del IMU del Curie
#include <SparkFunMLX90614.h> //libreria de los MLX
#include <Sharp.h> //libreria de los Sharps, duh
#include <ZX_Sensor.h> //¿Para los sensores ZX?
#include <Ultrasonico.h> //Libreria para los Ultrasonicos
#include <Wire.h> //Libreria para I2C
#include <CurieTime.h> //Libreria para medir el tiempo del IMU

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
byte PROGMEM pinEncDerA1 = 6;
byte PROGMEM pinEncDerA2 = 5;
Encoder EncDerA(pinEncDerA1, pinEncDerA2); //Encoder en los pines 18 y 19
byte PROGMEM pinEncIzqE1 = 2;
byte PROGMEM pinEncIzqE2 = 3;
Encoder EncIzqE(pinEncIzqE1, pinEncIzqE2); //Encoder en los pines 18 y 19

//Servo
byte PROGMEM pinServo = 4;
Servo tiraKits; //objeto del servo

//MLX
int PROGMEM I2C_Address_MLX1 = 0x3C;
IRTherm therm1; //Primer MLX
int PROGMEM I2C_Address_MLX2 = 0x4C;
IRTherm therm2; //Segundo MLX

//Sensores ZX
ZX_Sensor ZXDer(0x10); //ZX de la Der con I2C address 0x10
ZX_Sensor ZXIzq(0x11); //ZX de la Izq con I2C address 0x11

//Sharps
byte PROGMEM pinSharpDerEnf = 2;
Sharp SharpDerEnf(pinSharpDerEnf, 30); //Sharp de la Derecha Enf en el pin 0 con dist 30
byte PROGMEM pinSharpDerAtr = 0;
Sharp SharpDerAtr(pinSharpDerAtr, 30); //Sharp de la Derecha Atr en el pin 1 con dist 30
byte PROGMEM pinSharpIzqEnf = 3;
Sharp SharpIzqEnf(pinSharpIzqEnf, 30); //Sharp de la Izquierda Enf en el pin 2 con dist 30
byte PROGMEM pinSharpIzqAtr = 1;
Sharp SharpIzqAtr(pinSharpIzqAtr, 30); //Sharp de la Izquierda Atr en el pin 3 con dist 30

//Ultrasonico
byte PROGMEM pinTriggUltIzq = 2;
byte PROGMEM pinEchoUltIzq = 3;
Ultrasonico UltIzq(pinTriggUltIzq, pinEchoUltIzq); //Ult de la Izq, trigger en pin 2 y echo en pin 3
byte PROGMEM pinTriggUltDer = 5;
byte PROGMEM pinEchoUltDer = 6;
Ultrasonico UltDer(pinTriggUltDer, pinEchoUltDer); //Ult de la Der, trigger en pin 4 y echo en pin 5

byte dif = 0; //Diferencia para hacer cambios en motores
byte velMDE = 60; //velocidad para el motor de la derecha enfrente
byte velMDA = 200; //velocidad para el motor de la derecha atras
byte velMIE = 65; //velocidad para el motor de la izquierda enfrente
byte velMIA = 200; //velocidad para el motor de la izquierda atras

byte PROGMEM out = 11; //pin del out del sensor de color

void setup() {
  Serial.begin(9600);

  ZXDer.init(); //Inicializa sensores ZX
  ZXIzq.init();

  tiraKits.attach(pinServo); //Servo al pin 8

  MotDerEnf.begin(); //inicializa el bus del I2C para los motores
  MotDerAtr.begin();
  MotIzqEnf.begin();
  MotIzqAtr.begin();

  //MLX
  therm1.begin(I2C_Address_MLX1); //Primer MLX
  therm1.setUnit(TEMP_C);
  therm2.begin(I2C_Address_MLX2); //Segundo MLX
  therm2.setUnit(TEMP_C);

  MDE->run(RELEASE); //Apaga los motores, por seguridad
  MDA->run(RELEASE);
  MIE->run(RELEASE);
  MIA->run(RELEASE);

  CurieIMU.begin(); //inicia el IMU
  CurieIMU.setGyroRange(250); //Da el rango para el IMU
  CurieIMU.autoCalibrateGyroOffset(); //Autocalibra el gyro
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0); //Autocalibra el acelerometro
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

  pinMode(out, INPUT); //preparar el pin del sensor de color para recibir información
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

void sharp()
{
  Serial.print(F("Sharps"));
  Serial.print("\t");
  Serial.print(F("Distancia/Raw"));
  Serial.print("\t");
  Serial.print(F("DerEnf"));
  Serial.print("\t");
  Serial.print(SharpDerEnf.distance());
  Serial.print("\t");
  Serial.print(analogRead(pinSharpDerEnf));
  Serial.print("\t");
  Serial.print(F("DerAtr"));
  Serial.print("\t");
  Serial.print(SharpDerAtr.distance());
  Serial.print("\t");
  Serial.print(analogRead(pinSharpDerAtr));
  Serial.print("\t");
  Serial.print(F("IzqEnf"));
  Serial.print("\t");
  Serial.print(SharpIzqEnf.distance());
  Serial.print("\t");
  Serial.print(analogRead(pinSharpIzqEnf));
  Serial.print("\t");
  Serial.print(F("IzqAtr"));
  Serial.print("\t");
  Serial.print(SharpIzqAtr.distance());
  Serial.print("\t");
  Serial.println(analogRead(pinSharpIzqAtr));
}

void zx()
{
  Serial.print(F("ZX"));
  Serial.print("\t");
  Serial.print(F("Z/X"));
  Serial.print("\t");
  Serial.print(F("Der"));
  Serial.print("\t");
  Serial.print(ZXDer.readZ());
  Serial.print("\t");
  Serial.print(ZXDer.readX());
  Serial.print("\t");
  Serial.print(F("Izq"));
  Serial.print("\t");
  Serial.print(ZXIzq.readZ());
  Serial.print("\t");
  Serial.println(ZXIzq.readX());
}

void servo()
{
  tiraKits.write(0);
  delay(1000);
  tiraKits.write(180);
  delay(1000);
}

void color()
{
  Serial.print(F("Color"));
  Serial.print("\t");
  Serial.println(pulseIn(out, LOW));
}

void ultrasonicos()
{
  Serial.print(F("Ultrasonicos"));
  Serial.print("\t");
  digitalWrite(pinTriggUltIzq, LOW); //Por cuestión de estabilización del sensor
  delayMicroseconds(5);
  digitalWrite(pinTriggUltIzq, HIGH); //envío del pulso ultrasónico
  delayMicroseconds(10);
  byte dist1 = pulseIn(pinEchoUltIzq, HIGH);
  delay(100);
  digitalWrite(pinTriggUltDer, LOW); //Por cuestión de estabilización del sensor
  delayMicroseconds(5);
  digitalWrite(pinTriggUltDer, HIGH); //envío del pulso ultrasónico
  delayMicroseconds(10);
  byte dist2 = pulseIn(pinEchoUltDer, HIGH);
  Serial.print(F("Distancia/Raw"));
  Serial.print("\t");
  Serial.print(F("Izq"));
  Serial.print("\t");
  Serial.print(UltIzq.distance());
  Serial.print("\t");
  Serial.print(dist1);
  Serial.print(F("Der"));
  Serial.print("\t");
  Serial.print(UltDer.distance());
  Serial.print("\t");
  Serial.print(dist2);
}

void encoder()
{
  Serial.print(F("Encoder"));
  Serial.print("\t");
  Serial.print(EncDerA.read());
  Serial.print("\t");
  Serial.println(EncIzqE.read());
}

void motores()
{
  Adelante();
  delay(1000);
  Atras();
  delay(1000);
  Detenerse();
}

void mlx()
{
  therm1.read();
  therm2.read();
  Serial.print(F("MLX"));
  Serial.print("\t");
  Serial.print(therm1.object());
  Serial.print("\t");
  Serial.println(therm2.object());
}

void loop() {
  //Llamar a la funcion(es) del sensor que se quiere probar
  /*
    sharp();
    color();
    servo();
    zx();
    encoder();
    motores();
    ultrasonicos();
    mlx();
  */
}
