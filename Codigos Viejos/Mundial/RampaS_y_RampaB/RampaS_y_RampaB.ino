#include "CurieIMU.h" //libreria del IMU del Curie
#include <MadgwickAHRS.h> //Libreria del algoritmo de Madgwick para el IMU
#include <Wire.h> //Libreria para I2C
#include <Adafruit_MotorShield.h> //libreria del shield de los motores
#include "utility/Adafruit_MS_PWMServoDriver.h" //Shield de los motores
#include <SparkFunMLX90614.h> //libreria de los MLX
#define M_PI   3.14159265358979323846264338327950288 //Numero PI

// algoritmo de Madgwik para el IMU
Madgwick madG; //Objeto del algoritmo
int ax, ay, az; //valores del acelerometro
int gx, gy, gz; //valores del gyro
int factor = 200; //factor para el algoritmo

//Motores
Adafruit_MotorShield MotDerEnf = Adafruit_MotorShield(0x60); //Motor Derecha Enfrete
Adafruit_MotorShield MotDerAtr = Adafruit_MotorShield(0x60); //Motor Derecha Atras
Adafruit_DCMotor *MDE = MotDerEnf.getMotor(1); //Apuntador al motor de la derecha enfrente
Adafruit_DCMotor *MDA = MotDerAtr.getMotor(2); //Apuntador al motor de la derecha atras
Adafruit_MotorShield MotIzqEnf = Adafruit_MotorShield(0x60); //Motor Izquierda Enfrete
Adafruit_MotorShield MotIzqAtr = Adafruit_MotorShield(0x60); //Motor Izquierda Atras
Adafruit_DCMotor *MIE = MotIzqEnf.getMotor(3); //Apuntador al motor de la izquierda enfrente
Adafruit_DCMotor *MIA = MotIzqAtr.getMotor(4); //Apuntador al motor de la izquierda atras

//MLX
IRTherm therm1; //Primer MLX
IRTherm therm2; //Segundo MLX

byte dif = 0; //Diferencia para hacer cambios en motores
byte velMDE = 60; //velocidad para el motor de la derecha enfrente
byte velMDA = 200; //velocidad para el motor de la derecha atras
byte velMIE = 65; //velocidad para el motor de la izquierda enfrente
byte velMIA = 200; //velocidad para el motor de la izquierda atras

byte CalibCalor = 27; //Temperatura a detectar victimas

void setup() {
  Serial.begin(9600);

  CurieIMU.begin(); //inicia el IMU
  CurieIMU.setGyroRange(250); //Da el rango para el IMU
  CurieIMU.autoCalibrateGyroOffset(); //Autocalibra el gyro
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0); //Autocalibra el acelerometro
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

  MotDerEnf.begin(); //inicializa el bus del I2C para los motores
  MotDerAtr.begin();
  MotIzqEnf.begin();
  MotIzqAtr.begin();

  MDE->run(RELEASE); //Apaga los motores, por seguridad
  MDA->run(RELEASE);
  MIE->run(RELEASE);
  MIA->run(RELEASE);
}

//Funcion que detiene los motores
void Detenerse()
{
  MDE->run(RELEASE); //Apaga los motores, por seguridad
  MDA->run(RELEASE);
  MIE->run(RELEASE);
  MIA->run(RELEASE);
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

//Funcion que detecta si hay victima
void Detectar()
{
  therm1.read(); //comienza lecturas del primer MLX
  therm2.read(); //comienza lecturas del segundo MLX

  int Therm1 = therm1.object(); //Lectura del objeto al cual apunta los MLX
  int Therm2 = therm2.object();

  if (Therm1 > CalibCalor || Therm2 > CalibCalor) //Si la lectura de calor es mayor a la const de victimas dispensa kit
  {
    Detenerse(); //Se detiene antes de dispensar kit

    /*
       Falta programacion de la OLED para indicar que se encontro una victima
    */

    /*
       Falta Programacion del dispensador aqui
    */
  }
}

//Sube la rampa
void RampaS()
{
  byte CvelMDE = velMDE; //Copias de las velocidades standard de los motores
  byte CvelMDA = velMDA;
  byte CvelMIE = velMIE;
  byte CvelMIA = velMIA;
  int setPoint = 0; //Punto medio en el cual se quiere estar del gyro Z
  int actualPoint = 0; //punto actual del gyro Z
  CurieIMU.readAccelerometer(ax, ay, az); //lectura del acelerometro
  float g = (ax / 32768.0) * CurieIMU.getAccelerometerRange(); //convierte datos raw del acelerometro
  if (g > 0.38) //si detecta inclinacion de rampa
  {
    Detectar(); //Detecta si hay victima
    ax = CurieIMU.getAccelerationX(); //datos raw del IMU
    ay = CurieIMU.getAccelerationY();
    az = CurieIMU.getAccelerationZ();
    gx = CurieIMU.getRotationX();
    gy = CurieIMU.getRotationY();
    gz = CurieIMU.getRotationZ();
    madG.updateIMU(gx / factor, gy / factor, gz / factor, ax, ay, az); //Actualiza datos del IMU para el algoritmo Madgwick
    setPoint = madG.getYaw() * 180.0 / M_PI; //Punto medio en el cual se quiere estar del gyro Z
    Adelante(); //Inicia Movimiento
    do { //Sube la rampa haciendo correcciones de movimiento hasta dejar de detectar la inclinación
      ax = CurieIMU.getAccelerationX(); //Datos Raw del IMU
      ay = CurieIMU.getAccelerationY();
      az = CurieIMU.getAccelerationZ();
      gx = CurieIMU.getRotationX();
      gy = CurieIMU.getRotationY();
      gz = CurieIMU.getRotationZ();
      madG.updateIMU(gx / factor, gy / factor, gz / factor, ax, ay, az); //Actualiza datos del IMU para el algoritmo Madgwick
      actualPoint = madG.getYaw() * 180.0 / M_PI; //Punto actual del gyro Z
      if (actualPoint > setPoint + 12) //En caso de estar llendo hacia la derecha aumenta la velocidad de la derecha
      {
        dif++;
        velMDE += map(dif, 0, velMDA, 0, velMDE);
        velMDA += dif;
        velMIE = CvelMIE;
        velMIA = CvelMIA;
        Adelante();
      }
      else if (actualPoint < setPoint - 12) //En caso de estar llendo hacia la izquierda aumenta la velocidad de la izquierda
      {
        dif++;
        velMIE += map(dif, 0, velMIA, 0, velMIE);
        velMIA += dif;
        velMDE = CvelMDE;
        velMDA = CvelMDA;
        Adelante();
      }
      else //En caso de ir bien regresa a los valores standard
      {
        dif = 0;
        velMIE = CvelMIE;
        velMIA = CvelMIA;
        velMDE = CvelMDE;
        velMDA = CvelMDA;
      }
      CurieIMU.readAccelerometer(ax, ay, az); //Datos Raw del acelerometro
      g = (ax / 32768.0) * CurieIMU.getAccelerometerRange(); //Revisa si sigue la inclinación de la rampa
    } while (g > 0.38);
    Detenerse(); //Termina Mov
  }
  else //En caso de no detectar ramoa
  {
    Detenerse();
  }
  velMDE = CvelMDE; //Regresa los valores standard a los motores
  velMDA = CvelMDA;
  velMIE = CvelMIE;
  velMIA = CvelMIA;
}

//Baja Rampa
void RampaB()
{
  CurieIMU.readAccelerometer(ax, ay, az); //lectura del acelerometro
  float g = (ax / 32768.0) * CurieIMU.getAccelerometerRange(); //convierte datos raw del gyro
  Serial.println(g);
  if (g < -0.38)
  {
    Adelante();
    do {
      Detectar();
      CurieIMU.readAccelerometer(ax, ay, az); //lectura del acelerometro
      g = (ax / 32768.0) * CurieIMU.getAccelerometerRange(); //convierte datos raw del gyro
    } while (g < -0.38);
  }
}

void loop() {
  RampaB();
}
