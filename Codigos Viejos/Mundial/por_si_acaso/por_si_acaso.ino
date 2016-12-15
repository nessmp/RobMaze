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
//#include <MadgwickAHRS.h> //Libreria del algoritmo de Madgwick para el IMU
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
//Madgwick madG; //Objeto del algoritmo

//MLX
IRTherm therm1; //Primer MLX
IRTherm therm2; //Segundo MLX

//Sensores ZX
ZX_Sensor ZXDer(0x10); //ZX de la Der con I2C address 0x10
ZX_Sensor ZXIzq(0x11); //ZX de la Izq con I2C address 0x11

//Oled
OLED oled;

//Sharps
Sharp SharpDerEnf(2, 30); //Sharp de la Derecha Enf en el pin 0 con dist 30
Sharp SharpDerAtr(0, 30); //Sharp de la Derecha Atr en el pin 1 con dist 30
Sharp SharpIzqEnf(3, 30); //Sharp de la Izquierda Enf en el pin 2 con dist 30
Sharp SharpIzqAtr(1, 30); //Sharp de la Izquierda Atr en el pin 3 con dist 30

//Ultrasonico
Ultrasonico UltIzq(2, 3); //Ult de la Izq, trigger en pin 2 y echo en pin 3
Ultrasonico UltDer(4, 5); //Ult de la Der, trigger en pin 4 y echo en pin 5

//Color
const int PROGMEM CalibNegro = 1000; //Constante del color negro
const byte PROGMEM out = 8; //Pin para el sensor de color

byte CalibCalor = 27; //Temperatura a detectar victimas

int const PROGMEM const30 = 20; //constante de los encoders para moverse 30 cm
int const PROGMEM const90 = 30; //Constante de los encoders para girar 90 grados

int ax, ay, az; //valores del acelerometro
int gx, gy, gz; //valores del gyro
int factor = 200; //factor para el algoritmo

byte dif = 0; //Diferencia para hacer cambios en motores
byte velMDE = 60; //velocidad para el motor de la derecha enfrente
byte velMDA = 200; //velocidad para el motor de la derecha atras
byte velMIE = 65; //velocidad para el motor de la izquierda enfrente
byte velMIA = 200; //velocidad para el motor de la izquierda atras

//Max de arreglo de x
byte const PROGMEM XX = 40;
//Maximo de arreglo de y
byte const PROGMEM YY = 40;
//Maximo de arreglos de z
byte const PROGMEM ZZ = 4;
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
byte bZ = 2;
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

bool bVictimaDetectada = false;

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

  Serial.begin(9600);
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
    for (byte iI = 0; iI < 2; iI++) //Despliega en la Oled que se detecto una victima
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
    for (byte iI = 0; iI < 2; iI++) //Despliega en la Oled que se detecto una victima
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
    for (byte iI = 0; iI < 2; iI++) //Despliega en la Oled que se detecto una victima
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
    for (byte iI = 0; iI < 2; iI++) //Despliega en la Oled que se detecto una victima
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

//Regresa el valor de Pitch que detecta el MPU
int IMUP()
{
  CurieIMU.readAccelerometer(ax, ay, az); //lectura del acelerometro
  float g = (ax / 32768.0) * CurieIMU.getAccelerometerRange(); //convierte datos raw del acelerometro
  return g;
}

/*
   Esta funcion se encuentra más abajo
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
*/

//Sube la rampa
void RampaS()
{
  byte CvelMDE = velMDE; //Copias de las velocidades standard de los motores
  byte CvelMDA = velMDA;
  byte CvelMIE = velMIE;
  byte CvelMIA = velMIA;
  byte SharpDE; //Lectura de los sharps
  byte SharpDA;
  byte SharpIE;
  byte SharpIA;
  float IMU = IMUP(); //Lectura del Pitch detectada por el IMU
  if (IMU > 0.38) //Si se detecta inclinación de rampa
  {
    Adelante(); //Comienza movimiento
    do {
      SharpDE = SharpDerEnf.distance(); //Lectura de los sharps
      SharpDA = SharpDerAtr.distance();
      SharpIE = SharpIzqEnf.distance();
      SharpIA = SharpIzqAtr.distance();
      if (SharpIE < 8 || SharpIA < 8)//Si se detecta que esta muy cerca la pared de la izq aumenta la vel de las mecanums de la izq y regresa al standard las de la der
      {
        velMIE += map(dif, 0, velMIA, 0, velMIE);
        velMIA += dif;
        velMDE = CvelMDE;
        velMDA = CvelMDA;
      }
      else if (SharpDE < 8 || SharpDA < 8)//Si se detecta que esta muy cerca la pared de la der aumenta la vel de las mecanums de la der y regresa al standard las de la izq
      {
        velMDE += map(dif, 0, velMDA, 0, velMDE);
        velMDA += dif;
        velMIE = CvelMIE;
        velMIA = CvelMIA;
      }
      else //Si se ecuentra bien regresa todos los motores a vel standard
      {
        velMDE = CvelMDE;
        velMDA = CvelMDA;
        velMIE = CvelMIE;
        velMIA = CvelMIA;
      }
      Detectar(); //Detecta si hay victimas en la rampa
      IMU = IMUP(); //Lectura del Pitch del IMU por si ya se salio de la rampa
    } while (IMU > 0.38);//Si ya no detecta inclinación de la rampa sale del loop
    Detenerse();//Fin del mov
  }
}

//Baja Rampa
void RampaB()
{
  float IMU = IMUP();
  if (IMU < -0.38)
  {
    Adelante();
    do {
      Detectar();
      float IMU = IMUP();
    } while (IMU < -0.38);
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
  if (Z < 30 && (X != 119 || X != 120 || X != 121)) //Si el sensor ZX detecta algo en Z pero no en X
  {
    bBotella = true;
  }
  if ((SEnf < 30 && SAtr > 30) || (SAtr < 30 && SEnf > 30)) //Si un sharp detecta pared pero el otro no
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
  if (Z < 30 && (X != 119 || X != 120 || X != 121)) //Si el sensor ZX detecta algo en Z pero no en X
  {
    bBotella = true;
  }
  if ((SEnf < 30 && SAtr > 30) || (SAtr < 30 && SEnf > 30)) //Si un sharp detecta pared pero el otro no
  {
    bBotella = true;
  }
  return bBotella; //Regresa si hay botella o no
}

//Regresa verdadero si se encuentra sobre un cuadro negro
bool HoyoNegro()
{
  bool iReturn = false; //Variable a regresar
  for (byte iI = 0; iI < 2; iI++) //Hace lecturas dos veces
  {
    int color = pulseIn(out, LOW); //Pulse in para conseguir datos del sensor de color
    if (color > CalibNegro) //Si es mayor a la calib del negro se esta sobre un cuadro negro
    {
      iReturn = true;
    }
  }
  return iReturn;
}

//Calibrar la constante de calor
byte CalibrarCalor()
{
  therm1.read(); //Inicia comunicacion del mlx
  byte calor = therm1.object(); //Lectura del objeto al cual apunta el MLX
  return calor + 3; //Regresa el valor + 3
}

//Se acomoda en el cuadro en el que esta
void Acomodo() {} //Usar Encoders, Sharps, ZX, Ultrasonicos, MPU

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
  bool bDir[4] = {false, false, false, false};
  Paso++;
  pasoActual = Paso;
  Pos[bZ][bX][bY] = Paso;
  Possibility[Paso] = getPossibility(bDir);
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
  Serial.println(F("----Datos----"));
  Serial.print("bZ: ");
  Serial.println(bZ);
  Serial.print("bX: ");
  Serial.println(bX);
  Serial.print("bY: ");
  Serial.println(bY);
  Serial.print("Pos: ");
  Serial.println(Pos[bZ][bX][bY]);
  Serial.print("Possibility: ");
  Serial.println(Possibility[Paso]);
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
  int iMPUP = IMUP();
  if (iMPUP > 14 || iMPUP < -2)
  {
    bReturn = true;
  }
  return bReturn;
}

//Cruza la rampa, ACTUALIZAR bZ aqui!!
void MovRampa()
{
  Serial.println(F("----Rampa----"));
  Serial.print("bZ: ");
  Serial.println(bZ);
  int iMPUP = IMUP();
  //Si detecta que hay que subir la rampa
  if (iMPUP > 14)
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
    while (iMPUP > 14)
    {
      RampaS();
      iMPUP = IMUP();
    }
    Detenerse();
    bZ += 1;
  }
  //Si esta bajando la rampa
  else if (iMPUP < -2)
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
    while (iMPUP < -2)
    {
      RampaB();
      iMPUP = IMUP();
    }
    Detenerse();
    bZ -= 1;
  }
  Serial.print("brRampa: ");
  Serial.println(brRampa);
  Serial.print("biI: ");
  Serial.println(biI);
  Serial.print("Run[brRampa][biI]: ");
  Serial.println(Run[brRampa][biI]);
  Serial.print("bZ: ");
  Serial.println(bZ);
}

//Se mueve de la coordenada actuar(iCoordAc) a la coordenada deseada(icCoord)
void Move(int iCoordAc, int icCoord)
{
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
      giroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 2)
    {
      Adelante30();
    }
    else if (Direcc == 3)
    {
      giroDer90();
      Acomodo();
      giroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 4)
    {
      giroIzq90();
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
      giroIzq90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 3)
    {
      giroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 4)
    {
      giroDer90();
      Acomodo();
      giroDer90();
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
      giroIzq90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 2)
    {
      giroDer90();
      Acomodo();
      giroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 3)
    {
      Adelante30();
    }
    else if (Direcc == 4)
    {
      giroDer90();
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
      giroDer90();
      Acomodo();
      giroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 2)
    {
      giroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 3)
    {
      giroIzq90();
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
  //si esa nueva coordenada es un hoyo negro regresa y busca a donde moverse
  if (HoyoNegro())
  {
    listaX[subNegro] = CopcCoord;
    subNegro += 1;
    Serial.println(F("----Entra HoyoNegro----"));
    bool bListo = false;
    Atras30();
    Acomodo();
    bX = bCX;
    bY = bCY;
    byte iCPaso = Pos[bZ][bX][bY];
    Serial.print("bZ: ");
    Serial.println(bZ);
    Serial.print("bX: ");
    Serial.println(bX);
    Serial.print("bY: ");
    Serial.println(bY);
    Serial.print("Pos[bZ][bX][bY]= ");
    Serial.println(Pos[bZ][bX][bY]);
    for (byte iI = 0; iI < Possibility[iCPaso]; iI++)
    {
      Serial.print("icCoord: ");
      Serial.println(icCoord);
      int iThis = getCoord(Run[iCPaso][iI], iCPaso);
      Serial.print("iThis #");
      Serial.print(iI);
      Serial.print(": ");
      Serial.println(getCoord(Run[iCPaso][iI], iCPaso));
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
    Serial.print("iCPaso: ");
    Serial.println(iCPaso);
    Serial.print("Run 1: ");
    Serial.println(Run[iCPaso][0]);
    Serial.print("Run 2: ");
    Serial.println(Run[iCPaso][1]);
    Serial.print("Run 3: ");
    Serial.println(Run[iCPaso][2]);
    Serial.print("Run 4: ");
    Serial.println(Run[iCPaso][3]);
    Serial.print("Possbility: ");
    Serial.println(Possibility[iCPaso]);
    SearchRouteAndMove();
  }
  //Si esa nueva coordenada es la rampa, la cruza y actualiza bZ
  else if (Rampa())
  {
    MovRampa();
    bARampa = true;
  }
  Acomodo();
}


//Se mueve hasta el paso que se tiene como parametro
void moveUntil(byte bHere)
{
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
  }
}

//Regresa el punto de inicio
void extractionPoint()
{
  moveUntil(1);
  Detenerse();
  delay(30000);
}

//Se mueve a la coordenada desconocida
void exploreNewWorlds(byte bHere)
{
  Serial.println(F("----exploreNewWorlds----"));
  Serial.print("En Paso: ");
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
      Serial.print("Del Paso: ");
      Serial.println(bHere);
      Serial.print("Posibilidad #= ");
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
      Serial.print("Coordenada Actual: ");
      Serial.println(iCoordAc);
      Serial.print("Coordenada Deseada: ");
      Serial.println(icCoord);
      Move(iCoordAc, icCoord);
      iCounter++;
    }
  }
}

//Busca el paso al cual llegar para despues moverse a la coordenada desconocida
byte WhereToGo()
{
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
          Serial.println(F("----WhereToGo----"));
          Serial.print("Paso: ");
          Serial.println(iI);
          Serial.print(F("Numero de Run: "));
          Serial.println(iJ);
          Serial.print(F("No ha estado en: "));
          Serial.println(getCoord(Run[iI][iJ], iI));
        }
        Serial.println(F("----WhereToGo----"));
        Serial.print(F("Paso: "));
        Serial.println(iI);
        Serial.print(F("Numero de Run: "));
        Serial.println(iJ);
        Serial.print(F("No ha estado en: "));
        Serial.println(getCoord(Run[iI][iJ], iI));
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
  byte bData = WhereToGo();
  moveUntil(bData);
  exploreNewWorlds(bData);
}

//Funcion a llamar para completar el laberinto
void Laberinto()
{
  GetDatos();
  SearchRouteAndMove();
}

//Giro de 90 grados viendo unicamente paredes
void giroParedes90Der()
{
  bool Der = ParedDer(); //Paredes antes del giro
  bool Izq = ParedIzq();
  bool Enf = ParedEnf();
  byte counter = 0; //Contador
  bool Atr = false; //Siempre hay un camino atras del robot
  bool Der2; //Paredes despues del giro
  bool Izq2;
  bool Enf2;
  bool Atr2;
  do {
    int Enc; //Almacena cuenta de encoder
    if (counter == 0) //Si es el primer mov de giro da un giro completo de 90 grados
    {
      EncDerE.write(0);
      Enc = EncDerE.read();
      Derecha(); //Inicia giro
      while (Enc < const90)
      {
        Enc = EncDerE.read();
      }
      Detenerse(); //Termina giro
    }
    else //En caso de haber intentado girar 90 grados antes ahora solo gira 10
    {
      EncDerE.write(0);
      Enc = EncDerE.read();
      Derecha(); //inicia mov
      while (Enc < const90 / 9)
      {
        Enc = EncDerE.read();
      }
      Detenerse(); //Termina mov
    }
    Der2 = ParedDer(); //Lectura de nuevas paredes
    Enf2 = ParedEnf();
    Izq2 = ParedIzq();
    Atr2 = Izq; //No se puede comprobar si hay una pared atras por lo que se supone que el mov fue correcto
    counter++; //Aumenta contador por se se tiene que hacer una correccion ahora moverse 10 grados y no 90
  } while (Der2 != Atr && Enf2 != Der && Atr2 != Izq && Izq2 != Enf); //Las lecturas de las nuevas paredes tienen que tener realación con las lecturas previas al giro
}

void giroParedes90Izq()
{
  bool Der = ParedDer(); //Paredes antes del giro
  bool Izq = ParedIzq();
  bool Enf = ParedEnf();
  byte counter = 0; //Contador
  bool Atr = false; //Siempre hay un camino atras del robot
  bool Der2; //Paredes despues del giro
  bool Izq2;
  bool Enf2;
  bool Atr2;
  do {
    int Enc; //Almacena cuenta de encoder
    if (counter == 0) //Si es el primer mov de giro da un giro completo de 90 grados
    {
      EncDerE.write(0);
      Enc = EncDerE.read();
      Izquierda(); //Inicia giro
      while (Enc < const90)
      {
        Enc = EncDerE.read();
      }
      Detenerse(); //Termina giro
    }
    else //En caso de haber intentado girar 90 grados antes ahora solo gira 10
    {
      EncDerE.write(0);
      Enc = EncDerE.read();
      Izquierda(); //inicia mov
      while (Enc < const90 / 9)
      {
        Enc = EncDerE.read();
      }
      Detenerse(); //Termina mov
    }
    Der2 = ParedDer(); //Lectura de nuevas paredes
    Enf2 = ParedEnf();
    Izq2 = ParedIzq();
    Atr2 = Der; //No se puede detectar si hay pared de atras por lo que se supone que el mov fue correcto
    counter++; //Aumenta contador por se se tiene que hacer una correccion ahora moverse 10 grados y no 90
  } while (Der2 != Enf && Enf2 != Izq && Atr2 != Der && Izq2 != Atr); //Las lecturas de las nuevas paredes tienen que tener realación con las lecturas previas al giro
}

void giroParedes180()
{
  bool Der = ParedDer(); //Paredes antes del giro
  bool Izq = ParedIzq();
  bool Enf = ParedEnf();
  byte counter = 0; //Contador
  bool Atr = false; //Siempre hay un camino atras del robot
  bool Der2; //Paredes despues del giro
  bool Izq2;
  bool Enf2;
  bool Atr2;
  do {
    int Enc; //Almacena cuenta de encoder
    if (counter == 0) //Si es el primer mov de giro da un giro completo de 180 grados
    {
      EncDerE.write(0);
      Enc = EncDerE.read();
      Izquierda(); //Inicia giro
      while (Enc < const90 * 2)
      {
        Enc = EncDerE.read();
      }
      Detenerse(); //Termina giro
    }
    else //En caso de haber intentado girar 180 grados antes ahora solo gira 10
    {
      EncDerE.write(0);
      Enc = EncDerE.read();
      Izquierda(); //inicia mov
      while (Enc < const90 / 9)
      {
        Enc = EncDerE.read();
      }
      Detenerse(); //Termina mov
    }
    Der2 = ParedDer(); //Lectura de nuevas paredes
    Enf2 = ParedEnf();
    Izq2 = ParedIzq();
    Atr2 = Enf; //No se puede detectar si hay pared de atras por lo que se supone que el mov fue correcto
    counter++; //Aumenta contador por se se tiene que hacer una correccion ahora moverse 10 grados y no 180
  } while (Der2 != Izq && Enf2 != Atr && Atr2 != Enf && Izq2 != Der); //Las lecturas de las nuevas paredes tienen que tener realación con las lecturas previas al giro

}

void loop() {
  //Probaria estas funciones en mi robot.... Si tuviera uno :(
  /*
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

    Laberinto();
  */
}
