#include <Encoder.h> //para que crees...
#include <NewPing.h>
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>
#include <SharpIR.h> //sharps
#include <Servo.h>
#include "I2Cdev.h" //MPU
#include "MPU6050_6Axis_MotionApps20.h" //MPU
#include <SparkFunMLX90614.h>
#define MAX_DISTANCE 30
#define model 1080 //modelo del sharp GP2Y0A21Y
#define OUTPUT_READABLE_YAWPITCHROLL //Yaw Pitch Roll del MPU

//Max de arreglo de x
byte const XX = 40;
//Maximo de arreglo de y
byte const YY = 40;
//Maximo de arreglos de z
byte const ZZ = 3;
//Maximo de pasos posibles
byte const maxPasos = 100;
//Maximo de cuadros negros
byte const maxNegros = 20;
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
byte bZ = 0;
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
bool bInicio = true;

//////////////////
///Calibracion////
//////////////////

int CalibCalor = 28;
int CalibNegro = 1000;

//////////////////
///////MPU////////
//////////////////

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

//////////////////
/////MOTORES//////
//////////////////

byte motDerE1 = 8;
byte motDerE2 = 9;

byte motDerA1 = 11;
byte motDerA2 = 10;

byte motIzqE1 = 7;
byte motIzqE2 = 6;

byte motIzqA1 = 5;
byte motIzqA2 = 4;

const byte MotD = 255;
const byte MotI = 255;
const byte MotDM = 150;
const byte MotIM = 90;

byte Dif = 0;

//////////////////
/////ENCODERS/////
//////////////////

Encoder EncDerE(18, 19);
Encoder Enc2(17, 27);

long oldPosition  = -999;

int const PROGMEM const90 = 3725;

int const PROGMEM const30 = 6000;

//////////////////
//////CALOR///////
//////////////////

IRTherm therm1; //IZQUIERDA ADELANTE

IRTherm therm3; //IZQUIERDA ATRAS
IRTherm therm4; //DERECHA ENFRENTE

//////////////////
//////SHARPS//////
//////////////////

byte Enf = A0;
byte Izq = A1;
byte Der =  A2;

SharpIR SharpEn(Enf, 25, 93, model);
SharpIR SharpIz(Izq, 25, 93, model);
SharpIR SharpDe(Der, 25, 93, model);

/////////////////
//////SERVO//////
/////////////////

Servo Dispensador;
const int pinservo = 47;
const int PulsoMinimo = 650;
const int PulsoMaximo = 2550;

//////////////////
//////COLOR//////
//////////////////
const byte s0 = 13;
const byte s1 = 12;
const byte s2 = 29;
const byte s3 = 1;
const byte out = 14;

//////////////////
////////LCD///////
//////////////////

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

//////////////////
///ULTRASONICOS///
//////////////////

byte Trigger1 = 28;
byte Echo1 = 26;

NewPing sonar1(Trigger1, Echo1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar1.ping_cm();

byte Trigger2 = 38;
byte Echo2 = 36;

NewPing sonar2(Trigger2, Echo2, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar1.ping_cm();

byte Trigger3 = 42;
byte Echo3 = 40;

NewPing sonar3(Trigger3, Echo3, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte Trigger4 = 51;
byte Echo4 = 53;

NewPing sonar4(Trigger4, Echo4, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();


byte Trigger7 = 23;
byte Echo7 = 25;

NewPing sonar7(Trigger7, Echo7, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte Trigger8 = 24;
byte Echo8 = 22;

NewPing sonar8(Trigger8, Echo8, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //Serial.println("hola1");

  //MPU
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(4);
  mpu.setYGyroOffset(-6);
  mpu.setZGyroOffset(101);
  mpu.setZAccelOffset(1357); // 1688 factory default for my test chip

  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {

    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }

  //CALOR
  therm1.begin(0x1C);
  therm1.setUnit(TEMP_C);



  therm3.begin(0x3C);
  therm3.setUnit(TEMP_C);

  therm4.begin(0x4C);
  therm4.setUnit(TEMP_C);

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
  lcd.noBacklight(); // finish with backlight on
  lcd.setCursor(0, 0); //Start at character 4 on line 0
  lcd.print("OLIVER");

  //SERVO
  Dispensador.attach(pinservo, PulsoMinimo, PulsoMaximo);

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
  //Serial.println("hola");

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
  lcd.backlight();
}

int MPUY()
{
  int iReturn;
  do {

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      // other program behavior stuff here
      // .
      // .
      // .
      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
      // .
      // .
      // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      ////Serial.println(F("FIFO overflow!"));
      iReturn = 99999;

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      //Serial.print("quat\t");
      //Serial.print(q.w);
      //Serial.print("\t");
      //Serial.print(q.x);
      //Serial.print("\t");
      //Serial.print(q.y);
      //Serial.print("\t");
      //Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      //Serial.print("euler\t");
      //Serial.print(euler[0] * 180 / M_PI);
      //Serial.print("\t");
      //Serial.print(euler[1] * 180 / M_PI);
      //Serial.print("\t");
      //Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //Serial.print("ypr\t");
      //Serial.print(ypr[0] * 180 / M_PI);
      //Serial.print("\t");
      //Serial.print(ypr[1] * 180 / M_PI);
      //Serial.print("\t");
      //Serial.println(ypr[2] * 180 / M_PI);
      iReturn = ypr[0] * 180 / M_PI;
#endif
    }
  } while (iReturn == 99999 || iReturn == -31073);
  return iReturn;
}

double MPUP()
{
  double iReturn;
  do {

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      // other program behavior stuff here
      // .
      // .
      // .
      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
      // .
      // .
      // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      ////Serial.println(F("FIFO overflow!"));
      iReturn = 99999;

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      //Serial.print("quat\t");
      //Serial.print(q.w);
      //Serial.print("\t");
      //Serial.print(q.x);
      //Serial.print("\t");
      //Serial.print(q.y);
      //Serial.print("\t");
      //Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      //Serial.print("euler\t");
      //Serial.print(euler[0] * 180 / M_PI);
      //Serial.print("\t");
      //Serial.print(euler[1] * 180 / M_PI);
      //Serial.print("\t");
      //Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //Serial.print("ypr\t");
      //Serial.print(ypr[0] * 180 / M_PI);
      //Serial.print("\t");
      //Serial.print(ypr[1] * 180 / M_PI);
      //Serial.print("\t");
      //Serial.println(ypr[2] * 180 / M_PI);
      iReturn = ypr[1] * 180 / M_PI;
#endif
    }
  } while (iReturn == 99999 || iReturn < 0);
  return iReturn;
}

double MPUR()
{
  double iReturn;
  do {

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      // other program behavior stuff here
      // .
      // .
      // .
      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
      // .
      // .
      // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      ////Serial.println(F("FIFO overflow!"));
      iReturn = 99999;

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      //Serial.print("quat\t");
      //Serial.print(q.w);
      //Serial.print("\t");
      //Serial.print(q.x);
      //Serial.print("\t");
      //Serial.print(q.y);
      //Serial.print("\t");
      //Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      //Serial.print("euler\t");
      //Serial.print(euler[0] * 180 / M_PI);
      //Serial.print("\t");
      //Serial.print(euler[1] * 180 / M_PI);
      //Serial.print("\t");
      //Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //Serial.print("ypr\t");
      //Serial.print(ypr[0] * 180 / M_PI);
      //Serial.print("\t");
      //Serial.print(ypr[1] * 180 / M_PI);
      //Serial.print("\t");
      //Serial.println(ypr[2] * 180 / M_PI);
      iReturn = ypr[2] * 180 / M_PI;
#endif
    }
  } while (iReturn == 99999 || iReturn == -31073 || iReturn > 1000);
  return iReturn;
}

bool Negro()
{
  bool iReturn = false;
  int color = pulseIn(out, LOW);
  ////Serial.println("Entro color");
  //Serial.println(color);
  if (color > CalibNegro)
  {
    iReturn = true;
  }
  color = pulseIn(out, LOW);
  lcd.setCursor(0, 1);
  //lcd.print(color);
  if (color > CalibNegro)
  {
    iReturn = true;
  }
  return iReturn;
}

int velMDE = 180;
int velMDA = 186;
int velMIE = 162;
int velMIA = 162;

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
  analogWrite(motIzqE2, velMIE + 9);

  analogWrite(motIzqA1, 0); //172  //192
  analogWrite(motIzqA2, velMIA + 9);
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

void IzquierdaM30()
{
  EncDerE.write(0);
  Dif = -50;
  IzquierdaM();
  int Enc = EncDerE.read();
  while (Enc < 70)
  {
    //Serial.println(Enc);
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
    ////Serial.println(Enc);
    Enc = EncDerE.read();
  }
  Detenerse();
  Dif = 0;
}

void GiroDer18()
{
  EncDerE.write(0);
  int Enc = EncDerE.read();
  ////Serial.println(EncDerE.read());
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
  ////Serial.println(EncDerE.read());
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
  //lcd.clear();
  //lcd.print("GiroDer90");
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
  ////Serial.println(EncDerE.read());
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
  ////Serial.println(EncDerE.read());
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
  //lcd.clear();
  //lcd.print("GiroIzq90");
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
  //lcd.clear();
  //lcd.print("Adelante30");
  Detectar();
  Adelante10();
  Detectar();
  Adelante10();
  Detectar();
  Adelante10();
  Detectar();
  Detenerse();
  bVictimaDetectada = false;
}

void Atras30()
{
  //lcd.clear();
  //lcd.print("Atras30");
  EncDerE.write(0);
  int Enc = EncDerE.read();
  while (Enc > (const30 * -1) + 500)
  {
    Atras();
    Enc = EncDerE.read();
    //Serial.println(Enc);
  }
  Detenerse();
}

bool ParedDer()
{
  bool Pared = true;
  for (int iI = 0; iI < 2; iI ++)
  {
    int Sharp = SharpDe.distance();
    Serial.print(Sharp);
    Serial.print("\t");
    int u1 = sonar3.ping_cm();
    delay(30);
    int u2 = sonar4.ping_cm();
    delay(30);
    if (Sharp >= 24)
    {
      Pared = false;
    }
  }
  return Pared;
}

bool ParedIzq()
{
  bool Pared = true;
  int Sharp = SharpIz.distance();
  ////Serial.println(Sharp);
  if (Sharp >= 18)
  {
    Pared = false;
  }
  return Pared;
}

bool ParedEnf()
{
  bool Pared = true;
  int Sharp = SharpEn.distance();
  Serial.print(F("Sharp: "));
  Serial.print(Sharp);
  Serial.print("\t");
  if (Sharp > 50)
  {
    Pared = false;
  }
  return Pared;
}

void RampaAbajoIzq()
{
  lcd.clear();
  lcd.print("RampaAbajoIzq");
  int Roll = 0;
  if (ParedDer() && ParedEnf() && !ParedIzq())
  {
    lcd.print("entro");
    IzquierdaM();
    delay(1700);
    Detenerse();
    Roll = MPUR();
    lcd.setCursor(0, 1);
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
      if (Negro())
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

bool HoyoNegro()
{
  //Serial.println("Entro hoyo negro");
  bool Hoyo = false;
  delay(30);
  ////lcd.clear();
  ////lcd.print("HoyoNegro");
  int DistIzq = SharpIz.distance();
  if (Negro())
  {
    Hoyo = true;
  }
  return Hoyo;
}

void Detectar()
{
  therm1.read();

  therm3.read();
  therm4.read();
  int Therm1 = therm1.object();
  int Therm3 = therm3.object();
  int Therm4 = therm4.object(); //Derecha
  //Serial.println();
  //Serial.print("Izq Adelante: ");
  //Serial.println(Therm1);
  //Serial.print("Izq Atras: ");
  //Serial.println(Therm3);
  //Serial.print("Der Adelante: ");
  //Serial.println(Therm4);
  //int Temp = 27;
  if ((Therm1 > CalibCalor ||  Therm3 > CalibCalor || Therm4 > CalibCalor) && bVictimaDetectada == false)
  {
    bVictimaDetectada = true;
    Detenerse();
    lcd.clear();
    lcd.print("VICTIMA");
    lcd.setCursor(0, 1);
    lcd.print("DETECTADA");
    for (int iI = 75; iI < 113; iI++)
    {
      Dispensador.write(iI);
      delay(30);
    }
    delay(30);
    for (int iI = 113; iI > 75; iI--)
    {
      Dispensador.write(iI);
      delay(30);
    }
    delay(30);
    for (int iI = 0; iI < 10; iI++)
    {
      lcd.noBacklight();
      delay(30);
      lcd.backlight();
      delay(30);
    }
  }
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

void AcejarseDerecha()
{
  delay(5);
  ////Serial.println("entro 1 if");
  int Dist = SharpDe.distance();
  //Serial.print("antes    "); //Serial.println(Dist);

  while (Dist != 8 ) {
    if (Dist < 8)
    {

      while (Dist < 8)
      {
        IzquierdaM30();
        //delay(90);
        Dist = SharpDe.distance();
        //delay(10);
        //Serial.print("1zq    "); //Serial.println(Dist);
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
        //Serial.print("der    "); //Serial.println(Dist);
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
  int Dist2 = SharpIz.distance();

  ////Serial.println("entro 1 if");
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
        //Serial.println(SharpIz.distance());
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
        //Serial.println(SharpIz.distance());
      }
    }
    else if (Dist2 == 8)
      break;
    Detenerse();
  }
}

//enfrente
void AcejarseEnfrente()
{
  /*
    int SharpEnf = SharpEn.distance();
    int U = sonar1.ping_cm();
    //Serial.print(SharpEnf); //Serial.print("\t"); //Serial.println(U);
    if (SharpEnf < 5 || (U < 4 && U != 0))
    {
    Atras();
    while (SharpEnf < 5 || U < 4)
    {
      SharpEnf = SharpEn.distance();
      U = sonar1.ping_cm();
      if (U >= 4 || U == 0)
      {
        break;
      }
      delay(30);
    }
    Detenerse();
    }
    else
    {

    Adelante();
    while (SharpEnf > 6 && SharpEnf < 14)
    {
      SharpEnf = SharpEn.distance();
      int U = sonar1.ping_cm();
      if (U < 4 && U != 0)
      {
        break;
      }
    }
    Detenerse();
    }
  */
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

void Revisiones()
{
  //Blink();
  //delay(300);
  bool Hoyo = HoyoNegro();
  if (Hoyo == false)
  {
    //bool Rampa = RampaArriba();
    //RampaAbajoIzq();
    delay(30);
    /*
      if (Rampa == false)
      {
      RampaAbajoIzq();
      }
    */
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
  //lcd.clear();
  therm1.read();
  //therm2.read();
  therm3.read();
  therm4.read();

  int Therm1 = therm1.object();
  //int //therm2 = //therm2.object();
  int Therm3 = therm3.object();
  int Therm4 = therm4.object();

  lcd.setCursor(0, 0);
  //lcd.print(Therm1);
  //lcd.print("-");
  //lcd.print(Therm3);
  //lcd.print("-");
  //lcd.print(Therm4);
  //lcd.print("-");
  ////lcd.print(//therm2);

  Negro();

  delay(1000);
}

void UltIzq()
{
  //lcd.clear();
  //delay(30);
  int U1 = sonar7.ping_cm();
  delay(50);
  int U2 = sonar8.ping_cm();
  delay(30);
  //lcd.print(U1);
  lcd.setCursor(0, 1);
  //lcd.print(U2);
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
      ////Serial.println(EncDerE.read());
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
  //lcd.clear();
  //delay(30);
  int U1 = sonar3.ping_cm();
  delay(30);
  int U2 = sonar4.ping_cm();
  Serial.println();
  Serial.print("U1: ");
  Serial.println(U1);
  Serial.print("U2: ");
  Serial.println(U2);
  lcd.print(U1);
  lcd.setCursor(0, 1);

  while ((U1 - U2) != 0)
  {
    if ((U1 - U2) < 0)
    {
      EncDerE.write(0);
      int Enc = EncDerE.read();
      ////Serial.println(EncDerE.read());
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
      ////Serial.println(EncDerE.read());
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
  if (ParedDer())
  {
    AcejarseDerecha();
  }
  else if (ParedIzq())
  {
    AcejarseIzquierda();
  }
}

//Sube la rampa detectando si hay victima en ella
void RampaS()
{
  int MPU = MPUP();
  AdelanteRampa();
  do
  {
    MPU = MPUP();
    Serial.println(MPU);
    if (MPU == 0 || MPU == 7)
    {
      MPU = 19;
    }
  } while (MPU > 18);
  delay(400);
  Detenerse();
}
//Baja la rampa detectando si hay victima en ella
void RampaB() {} //Usar MPU, MLX

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
  Serial.println("----Datos----");
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
  int iMPUP = MPUP();
  if (iMPUP > 14 || iMPUP < -2)
  {
    bReturn = true;
  }
  return bReturn;
}

//Cruza la rampa, ACTUALIZAR bZ aqui!!
void MovRampa()
{
  Serial.println("----Rampa----");
  Serial.print("bZ: ");
  Serial.println(bZ);
  int iMPUP = MPUP();
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
      iMPUP = MPUP();
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
      iMPUP = MPUP();
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
  //si esa nueva coordenada es un hoyo negro regresa y busca a donde moverse
  if (HoyoNegro())
  {
    listaX[subNegro] = CopcCoord;
    subNegro += 1;
    Serial.println("----Entra HoyoNegro----");
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
  lcd.write("Fin");
  delay(30000);
}

//Se mueve a la coordenada desconocida
void exploreNewWorlds(byte bHere)
{
  Serial.println("----exploreNewWorlds----");
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
          Serial.println("----WhereToGo----");
          Serial.print("Paso: ");
          Serial.println(iI);
          Serial.print("Numero de Run: ");
          Serial.println(iJ);
          Serial.print("No ha estado en: ");
          Serial.println(getCoord(Run[iI][iJ], iI));
        }
        Serial.println("----WhereToGo----");
        Serial.print("Paso: ");
        Serial.println(iI);
        Serial.print("Numero de Run: ");
        Serial.println(iJ);
        Serial.print("No ha estado en: ");
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
  lcd.setCursor(0, 1);
  lcd.print("ir #");
  lcd.print(bData);
  moveUntil(bData);
  exploreNewWorlds(bData);
}

//Funcion a llamar para completar el laberinto
void Laberinto()
{
  GetDatos();
  SearchRouteAndMove();
}

void AcejarseEnfrente2()
{
  for (byte iI = 0; iI < 2; iI++)
  {
    byte Sharp = SharpEn.distance();
    Serial.println(Sharp);
    if (Sharp < 48 && Sharp > 20)
    {
      do {
        Adelante5();
        byte Sharp2 = SharpEn.distance();
        if (Sharp2 > Sharp)
        {
          Atras();
          delay(100);
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
      delay(100);
      Detenerse();
      byte Sharp2 = SharpEn.distance();
      if (Sharp2 > Sharp)
      {
        Adelante();
        delay(100);
        Detenerse();
      }
    }
  }
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

//3, 17, 18 y 19
void loop() {
  Laberinto();
}
