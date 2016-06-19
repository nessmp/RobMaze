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
#include <MadgwickAHRS.h> //Libreria del algoritmo de Madgwick para el IMU
#include <CurieTime.h> //Libreria para medir el tiempo del IMU
#include <OLED.h> //Oled...
#define M_PI   3.14159265358979323846264338327950288 //Numero PI

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

// algoritmo de Madgwik para el IMU
Madgwick madG; //Objeto del algoritmo

//MLX
IRTherm therm1; //Primer MLX
IRTherm therm2; //Segundo MLX

//Sensores ZX
ZX_Sensor ZXDer(0x10); //ZX de la Der con I2C address 0x10
ZX_Sensor ZXIzq(0x11); //ZX de la Izq con I2C address 0x11

//Oled
OLED oled;

//Sharps
Sharp SharpDerEnf(0, 30); //Sharp de la Derecha Enf en el pin 0 con dist 30
Sharp SharpDerAtr(1, 30); //Sharp de la Derecha Atr en el pin 1 con dist 30
Sharp SharpIzqEnf(2, 30); //Sharp de la Izquierda Enf en el pin 2 con dist 30
Sharp SharpIzqAtr(3, 30); //Sharp de la Izquierda Atr en el pin 3 con dist 30

//Ultrasonico
Ultrasonico UltIzq(2, 3); //Ult de la Izq, trigger en pin 2 y echo en pin 3
Ultrasonico UltDer(4, 5); //Ult de la Der, trigger en pin 4 y echo en pin 5

byte CalibCalor = 27; //Temperatura a detectar victimas

int const30 = 2000; //constante de los encoders para moverse 30 cm
int const const90 = 3000; //Constante de los encoders para girar 90 grados

int ax, ay, az; //valores del acelerometro
int gx, gy, gz; //valores del gyro
int factor = 200; //factor para el algoritmo

byte dif = 0; //Diferencia para hacer cambios en motores
byte velMDE = 60; //velocidad para el motor de la derecha enfrente
byte velMDA = 200; //velocidad para el motor de la derecha atras
byte velMIE = 65; //velocidad para el motor de la izquierda enfrente
byte velMIA = 200; //velocidad para el motor de la izquierda atras

void setup() {
  Serial.begin(9600);

  oled.Init(); //Inicializa OLED

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

  CurieIMU.begin(); //inicia el IMU
  CurieIMU.setGyroRange(250); //Da el rango para el IMU
  CurieIMU.autoCalibrateGyroOffset(); //Autocalibra el gyro
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0); //Autocalibra el acelerometro
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
}

//Regresa verdadero si detecta pared en la derecha
bool ParedDer()
{
  bool bPared = false; //Variable a regresar
  byte bX = ZXDer.readX(); //Lectura del eje X
  byte bZ = ZXDer.readZ(); //Lectura del eje Y
  if (bZ < 30 && (bX > 118 && bX < 122)) //Si Z es pequeño y X esta entre 119 y 121 hay pared
  {
    bPared = true;
  }
  return bPared;
}

//Regresa verdadero si detecta pared en la izquierda
bool ParedIzq()
{
  bool bPared = false; //Variable a regresar
  byte bX = ZXIzq.readX(); //Lectura del eje X
  byte bZ = ZXIzq.readZ(); //Lectura del eje Y
  if (bZ < 30 && (bX > 118 && bX < 122)) //Si Z es pequeño y X esta entre 119 y 121 hay pared
  {
    bPared = true;
  }
  return bPared;
}

//Regresa verdadero si detecta pared enfrente
bool ParedEnf()
{
  bool bPared = false; //Variable a regresar
  byte U1 = UltIzq.distance(); //Lectura del ultrasonico de la Izquierda
  byte U2 = UltDer.distance(); //Lectura del ultrasonico de la Derecha
  if (U1 < 20 && U2 < 20) //Si ambos ultrasonicos detectan una corta distancia, hay pared
  {
    bPared = true;
  }
  return bPared;
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

//Funcion para ir de reversa 30 cm
void Atras30()
{
  EncDerE.write(0);
  int Enc = EncDerE.read();
  Atras();
  while (Enc > (const30 * -1) + 500)
  {
    Enc = EncDerE.read();
  }
  Detenerse();
}

//Funcion que detecta si hay victima
void Detectar()
{
  therm1.read(); //comienza lecturas del primer MLX
  therm2.read(); //comienza lecturas del segundo MLX

  int Therm1 = therm1.object(); //Lectura del objeto al cual apunta los MLX
  int Therm2 = therm2.object();

  int pos = tiraKits.read();
  if (Therm1 > CalibCalor)
  {
    Detenerse(); //Se detiene antes de dispensar kit
    for(byte iI = 0; iI < 2; iI++) //Despliega en la Oled que se detecto una victima
    {
      oled.Kit();
      delay(500);
      oled.KitNegro();
      delay(500);
    }
    while (pos != 90) //Se mueve para tomar un kit
    {
      tiraKits.write(90);
      pos = tiraKits.read();
    }
    for(byte iI = 0; iI < 2; iI++) //Despliega en la Oled que se detecto una victima
    {
      oled.Kit();
      delay(500);
      oled.KitNegro();
      delay(500);
    }
    while (pos != 0) //Se mueve para tirar el kit
    {
      tiraKits.write(0);
      pos = tiraKits.read();
    }
  }
  else if (Therm2 > CalibCalor)
  {
    Detenerse(); //Se detiene antes de dispensar kit
    for(byte iI = 0; iI < 2; iI++) //Despliega en la Oled que se detecto una victima
    {
      oled.Kit();
      delay(500);
      oled.KitNegro();
      delay(500);
    }
    while (pos != 90) //Se mueve para tomar un kit
    {
      tiraKits.write(90);
      pos = tiraKits.read();
    }
    for(byte iI = 0; iI < 2; iI++) //Despliega en la Oled que se detecto una victima
    {
      oled.Kit();
      delay(500);
      oled.KitNegro();
      delay(500);
    }
    while (pos != 180) //Se mueve para tirar el kit
    {
      tiraKits.write(180);
      pos = tiraKits.read();
    }
  }
  while (pos != 90) //Se mueve para tomar un kit
  {
    tiraKits.write(90);
    pos = tiraKits.read();
  }
}

//funcion que avanza 10 cm
void Adelante10()
{
  EncDerE.write(0); //Inicia la cuenta de los encoders en 0
  int Enc = EncDerE.read(); //Primer lectura de los encoders, debe ser 0
  int const10 = const30 / 3; //Divide entre 3 la const 30
  Adelante(); //Comienza movimiento
  while (Enc < const10) //Mientras la lectura del encoder sea menor a la const sigue moviendose
  {
    Enc = EncDerE.read(); //lectura del encoder para salir del loop
  }
  Detenerse(); //Acaba Mov
}

//funcion para moverse 30 cm
void Adelante30()
{
  bool ProxPD = false; //Pared a la der despues de haber avanzado 30
  bool ProxPI = false; //Pared a la izq despues de haber avanzado 30
  bool inicioPD = ParedDer(); //Saber si al inicio hay pared a la derecha
  bool inicioPI = ParedIzq(); //Saber si al inicio hay pared a la izquierda
  Detectar(); //Detecta si hay victima
  Adelante10(); //Se mueve 10 cm
  byte ShD = SharpDerEnf.distance(); //Distancia detectada por el Sharp de la Derecha Enfrente
  if (ShD < 20) //Si la distancia es menor a 20 se espera una pared despues de moverse 30 cm
  {
    ProxPD = true;
  }
  byte ShI = SharpIzqEnf.distance(); //Distancia detectada por el Sharp de la Izquierda Enfrente
  if (ShI < 20) //Si la distancia es menor a 20 se espera una pared despues de moverse 30 cm
  {
    ProxPI = true;
  }
  Detectar(); //Detecta si hay victima
  Adelante10(); //Se mueve 10 cm
  ShD = SharpDerEnf.distance(); //Distancia detectada por el Sharp de la Derecha Enfrente
  if (ShD < 20) //Si la distancia es menor a 20 se espera una pared despues de moverse 30 cm
  {
    ProxPD = true;
  }
  ShI = SharpIzqEnf.distance(); //Distancia detectada por el Sharp de la Izquierda Enfrente
  if (ShI < 20) //Si la distancia es menor a 20 se espera una pared despues de moverse 30 cm
  {
    ProxPI = true;
  }
  Detectar(); //Detecta si hay victima
  Adelante10(); //Se mueve 10 cm
  bool finalPD = ParedDer(); //Saber si al final hay pared a la derecha
  bool finalPI = ParedIzq(); //Saber si al final hay pared a la izquierda
  if (finalPD != ProxPD) //Si la pared esperada no es igual a la actual se hace correccion
  {
    if (true == ProxPD) //Si se esperaba que hubiera pared
    {
      if (!BotellaDer()) //Se verifica no haberse equivocado al detectar una botella como pared
      {
        do {
          Adelante10();
          finalPD = ParedDer();
        } while (true != finalPD); //Avanza de 10cm en 10 cm hasta encontrar la pared
      }
    }
    else //En caso que no se esperara pared
    {
      do {
        Adelante10();
        finalPD = ParedDer();
      } while (false != finalPD); //Se avanza hasta no detectar pared
    }
  }
  if (finalPI != ProxPI) //Si la pared esperada no es igual a la actual se hace correccion
  {
    if (true == ProxPI) //Si se esperaba que hubiera pared
    {
      if (!BotellaIzq()) //Se verifica no haberse equivocado al detectar una botella como pared
      {
        do {
          Adelante10();
          finalPI = ParedIzq();
        } while (true != finalPI); //Avanza de 10cm en 10 cm hasta encontrar la pared
      }
    }
    else //En caso que no se esperara pared
    {
      do {
        Adelante10();
        finalPI = ParedIzq();
      } while (false != finalPI); //Se avanza hasta no detectar pared
    }
  }
  Detectar(); //Detecta si hay victima
  Detenerse(); //Termina mov de 30 cm
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

//Funcion para girar a la izquierda 90 grados apoyandose de los encoders y del IMU
void giroIzq90()
{
  EncDerE.write(0); //iniciar cueta de los encoders desde 0
  int iCuentas = EncDerE.read(); //Realiza una lectura
  int iCounter = 0; //Contador para el avg del IMU
  int gxRaw; //almacena el dato en el eje X del gyro, no nos sirve
  int gyRaw; //almacena el dato en el eje Y del gyro, no nos sirve
  int gzRaw; //almacena el dato en el eje Z del gyro
  int avgGzRaw = 0; //Almacena el avg de los Raw del eje Z del gyro
  int Degrees = 0; //Almacena los grados que nos hemos movido
  byte bTime1 = second(); //Segundo en el que se empezo el movimiento
  if (bTime1 > 50) //En caso de ser mayor de 50 se regresa a 0, por seguridad
  {
    setTime(0); //inicializa los segundos en 0
    while (bTime1 != 0) //La funcion de setTime se tarda en surtir efecto, por lo tanto esperamos a que funcione
    {
      bTime1 = second();
    }
  }
  Izquierda(); //Iniciamos el moviemiento a la izquierda
  while (iCuentas < const90) //hasta que las cuentas de encoders den la constante de 90
  {
    CurieIMU.readGyro(gxRaw, gyRaw, gzRaw); //lee los datos del gyro
    avgGzRaw += gzRaw; //sumamos los datos del eje Z para despues hacer un avg
    iCounter++; //ayuda para el avg
    iCuentas = EncDerE.read(); //Actualizamos las cuentas del encoder para llegar a salir del while
  }
  Detenerse(); //Termina el movimiento
  byte bTime2 = second(); //Segundo en el que se acabo el movimiento
  byte bTime = bTime2 - bTime1; //Tiempo que se tardo en efectuar el movimiento
  avgGzRaw = avgGzRaw / iCounter; // avg de los datos del eje Z del gyro
  Degrees = (avgGzRaw * 250.0) / 32768.0; //convertimos el avg a velocidad angular
  Degrees = (Degrees / bTime) * 10 - 2; //Sacamos los grados con la velocidad angular y se le hace una pequeña corrección, observada experimentalmente
  while (Degrees < 78 || Degrees > 112) //Correcciones si el IMU no detecto el mov aproximado de 90 grados
  {
    while (Degrees < 78) //en caso de haberse movido menos de 90 grados
    {
      avgGzRaw = 0; //volvemos a iniciar todos los valores necesarios a 0
      iCounter = 0;
      EncDerE.write(0);
      bTime1 = second(); //segundo antes del mov
      if (bTime1 > 50)
      {
        setTime(0);
        while (bTime1 != 0)
        {
          bTime1 = second();
        }
      }
      Izquierda(); //inicia mov
      while (iCuentas < (const90 / 9)) //nos movemos 10 grados en vez de 90
      {
        CurieIMU.readGyro(gxRaw, gyRaw, gzRaw);
        avgGzRaw += gzRaw;
        iCounter++;
        iCuentas = EncDerE.read();
      }
      Detenerse(); //finaliza mov
      bTime2 = second(); //segundo despues del mov
      bTime = bTime2 - bTime1; //timepo en que se tardo realizar el mov
      avgGzRaw = avgGzRaw / iCounter; //avg del eje Z
      int Degrees2 = (avgGzRaw * 250.0) / 32768.0; //grados del mov de correccion
      Degrees2 = (Degrees2 / bTime) * 10 - 2;
      Degrees += Degrees2; //se suma estos grados a los grados que se movieron inicialmente
    }
    while (Degrees > 112) //En caso de haberse movido más de 90 grados
    {
      avgGzRaw = 0; //volvemos a iniciar todos los valores necesarios a 0
      iCounter = 0;
      EncDerE.write(0);
      bTime1 = second(); //segundo antes del mov
      if (bTime1 > 50)
      {
        setTime(0);
        while (bTime1 != 0)
        {
          bTime1 = second();
        }
      }
      Derecha(); //inicia mov
      while (iCuentas < (const90 / 9)) //nos movemos 10 grados en vez de 90
      {
        CurieIMU.readGyro(gxRaw, gyRaw, gzRaw);
        avgGzRaw += gzRaw;
        iCounter++;
        iCuentas = EncDerE.read();
      }
      Detenerse(); //finaliza mov
      bTime2 = second(); //segundo despues del mov
      bTime = bTime2 - bTime1; //timepo en que se tardo realizar el mov
      avgGzRaw = avgGzRaw / iCounter; //avg del eje Z
      int Degrees2 = (avgGzRaw * 250.0) / 32768.0; //grados del mov de correccion
      Degrees2 = (Degrees2 / bTime) * 10 - 2;
      Degrees += Degrees2; //se suma estos grados a los grados que se movieron inicialmente
    }
  }
}

//Funcion para girar a la derecha 90 grados apoyandose de los encoders y del IMU
void giroDer90()
{
  EncDerE.write(0); //iniciar cueta de los encoders desde 0
  int iCuentas = EncDerE.read(); //Realiza una lectura
  int iCounter = 0; //Contador para el avg del IMU
  int gxRaw; //almacena el dato en el eje X del gyro, no nos sirve
  int gyRaw; //almacena el dato en el eje Y del gyro, no nos sirve
  int gzRaw; //almacena el dato en el eje Z del gyro
  int avgGzRaw = 0; //Almacena el avg de los Raw del eje Z del gyro
  int Degrees = 0; //Almacena los grados que nos hemos movido
  byte bTime1 = second(); //Segundo en el que se empezo el movimiento
  if (bTime1 > 50) //En caso de ser mayor de 50 se regresa a 0, por seguridad
  {
    setTime(0); //inicializa los segundos en 0
    while (bTime1 != 0) //La funcion de setTime se tarda en surtir efecto, por lo tanto esperamos a que funcione
    {
      bTime1 = second();
    }
  }
  Derecha(); //Iniciamos el moviemiento a la izquierda
  while (iCuentas < const90) //hasta que las cuentas de encoders den la constante de 90
  {
    CurieIMU.readGyro(gxRaw, gyRaw, gzRaw); //lee los datos del gyro
    avgGzRaw += gzRaw; //sumamos los datos del eje Z para despues hacer un avg
    iCounter++; //ayuda para el avg
    iCuentas = EncDerE.read(); //Actualizamos las cuentas del encoder para llegar a salir del while
  }
  Detenerse(); //Termina el movimiento
  byte bTime2 = second(); //Segundo en el que se acabo el movimiento
  byte bTime = bTime2 - bTime1; //Tiempo que se tardo en efectuar el movimiento
  avgGzRaw = avgGzRaw / iCounter; // avg de los datos del eje Z del gyro
  Degrees = (avgGzRaw * 250.0) / 32768.0; //convertimos el avg a velocidad angular
  Degrees = (Degrees / bTime) * 10 - 2; //Sacamos los grados con la velocidad angular y se le hace una pequeña corrección, observada experimentalmente
  while (Degrees < -78 || Degrees > -112) //Correcciones si el IMU no detecto el mov aproximado de 90 grados
  {
    while (Degrees < -78) //en caso de haberse movido menos de 90 grados
    {
      avgGzRaw = 0; //volvemos a iniciar todos los valores necesarios a 0
      iCounter = 0;
      EncDerE.write(0);
      bTime1 = second(); //segundo antes del mov
      if (bTime1 > 50)
      {
        setTime(0);
        while (bTime1 != 0)
        {
          bTime1 = second();
        }
      }
      Derecha(); //inicia mov
      while (iCuentas < (const90 / 9)) //nos movemos 10 grados en vez de 90
      {
        CurieIMU.readGyro(gxRaw, gyRaw, gzRaw);
        avgGzRaw += gzRaw;
        iCounter++;
        iCuentas = EncDerE.read();
      }
      Detenerse(); //finaliza mov
      bTime2 = second(); //segundo despues del mov
      bTime = bTime2 - bTime1; //timepo en que se tardo realizar el mov
      avgGzRaw = avgGzRaw / iCounter; //avg del eje Z
      int Degrees2 = (avgGzRaw * 250.0) / 32768.0; //grados del mov de correccion
      Degrees2 = (Degrees2 / bTime) * 10 - 2;
      Degrees += Degrees2; //se suma estos grados a los grados que se movieron inicialmente
    }
    while (Degrees > -112) //En caso de haberse movido más de 90 grados
    {
      avgGzRaw = 0; //volvemos a iniciar todos los valores necesarios a 0
      iCounter = 0;
      EncDerE.write(0);
      bTime1 = second(); //segundo antes del mov
      if (bTime1 > 50)
      {
        setTime(0);
        while (bTime1 != 0)
        {
          bTime1 = second();
        }
      }
      Izquierda(); //inicia mov
      while (iCuentas < (const90 / 9)) //nos movemos 10 grados en vez de 90
      {
        CurieIMU.readGyro(gxRaw, gyRaw, gzRaw);
        avgGzRaw += gzRaw;
        iCounter++;
        iCuentas = EncDerE.read();
      }
      Detenerse(); //finaliza mov
      bTime2 = second(); //segundo despues del mov
      bTime = bTime2 - bTime1; //timepo en que se tardo realizar el mov
      avgGzRaw = avgGzRaw / iCounter; //avg del eje Z
      int Degrees2 = (avgGzRaw * 250.0) / 32768.0; //grados del mov de correccion
      Degrees2 = (Degrees2 / bTime) * 10 - 2;
      Degrees += Degrees2; //se suma estos grados a los grados que se movieron inicialmente
    }
  }
}

//Detecta si hay botella a la derecha
bool BotellaDer() 
{
  bool bBotella = false; //Variable a regresar, verdadero si detecta botella
  byte Z = ZXDer.readZ(); //Lectura de Z
  byte X = ZXDer.readX(); //Lectura de X
  byte SEnf = SharpDerEnf.distance(); //Lectura del sharp de enf
  byte SAtr = SharpDerAtr.distance(); //Lectura del sharp de atr
  if(Z < 30 && (X != 119 || X != 120 || X != 121)) //Si el sensor ZX detecta algo en Z pero no en X
  {
    bBotella = true;
  }
  if((SEnf < 30 && SAtr > 30) || (SAtr < 30 && SEnf > 30)) //Si un sharp detecta pared pero el otro no
  {
    bBotella = true;
  }
  return bBotella; //Regresa si hay botella o no
}

//Detecta si hay botella a la izquierda
bool BotellaIzq() 
{
  bool bBotella = false; //Variable a regresar, verdadero si detecta botella
  byte Z = ZXIzq.readZ(); //Lectura de Z
  byte X = ZXIzq.readX(); //Lectura de X
  byte SEnf = SharpIzqEnf.distance(); //Lectura del sharp de enf
  byte SAtr = SharpIzqAtr.distance(); //Lectura del sharp de atr
  if(Z < 30 && (X != 119 || X != 120 || X != 121)) //Si el sensor ZX detecta algo en Z pero no en X
  {
    bBotella = true;
  }
  if((SEnf < 30 && SAtr > 30) || (SAtr < 30 && SEnf > 30)) //Si un sharp detecta pared pero el otro no
  {
    bBotella = true;
  }
  return bBotella; //Regresa si hay botella o no
}

void loop() {
  //Probaria estas funciones en mi robot.... Si tuviera uno :(
  ParedDer();
  ParedIzq();
  ParedEnf();
  Adelante();
  Detenerse();
  Atras();
  Atras30();
  Detectar();
  Adelante10();
  Adelante30();
  RampaS();
  RampaB();
  giroIzq90();
  giroDer90();
  BotellaDer();
  BotellaIzq();
}
