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

//////////////////
///Calibracion////
//////////////////

int CalibCalor = 40;
int CalibNegro = 1000;

//////////////////
////Algoritmo/////
//////////////////

bool bPos[20][20];
int iX = 20;
int iY = 20;
int iOption = 1;
int iAnterior = 1;
bool bNegro[10][10];
bool bAbriba = false;
int iAbriba = 0;

bool bVictimaDetectada = false;
bool bInicio = false;

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

//////////////////
/////ENCODERS/////
//////////////////

Encoder EncDerE(18, 19);
Encoder Enc2(17, 27);

long oldPosition  = -999;

int const90 = 3750;

const int const30 = 5800;

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
  Serial.begin(115200);
  Serial.println("hola1");
  
  //MPU
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(4);
  mpu.setYGyroOffset(-6);
  mpu.setZGyroOffset(101);
  mpu.setZAccelOffset(1357); // 1688 factory default for my test chip

  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {

    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
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
  lcd.print("OLIVER 2.0");

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

  /*
  for (int iI = 0; iI < 50; iI ++)
  {
    for (int iJ = 0; iJ < 50; iJ++)
    {
      bPos[iI][iJ] = false;
    }
  }
  bPos[20][20] = true;
  for (int iI = 0; iI < 10; iI++)
  {
    for (int iJ = 0; iI < 10; iI++)
    {
      bNegro[iI][iJ] = false;
    }
  }
  */
  Serial.println("hola");
  
}

void Reset()
{
  iX = 20;
  iY = 20;
  for (int iI = 0; iI < 50; iI ++)
  {
    for (int iJ = 0; iJ < 50; iJ++)
    {
      bPos[iI][iJ] = false;
    }
  }
  bPos[20][20] = true;
}

double MPUY()
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
      //Serial.println(F("FIFO overflow!"));
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
      Serial.print("quat\t");
      Serial.print(q.w);
      Serial.print("\t");
      Serial.print(q.x);
      Serial.print("\t");
      Serial.print(q.y);
      Serial.print("\t");
      Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      Serial.print("euler\t");
      Serial.print(euler[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(euler[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180 / M_PI);
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
      //Serial.println(F("FIFO overflow!"));
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
      Serial.print("quat\t");
      Serial.print(q.w);
      Serial.print("\t");
      Serial.print(q.x);
      Serial.print("\t");
      Serial.print(q.y);
      Serial.print("\t");
      Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      Serial.print("euler\t");
      Serial.print(euler[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(euler[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180 / M_PI);
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
      //Serial.println(F("FIFO overflow!"));
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
      Serial.print("quat\t");
      Serial.print(q.w);
      Serial.print("\t");
      Serial.print(q.x);
      Serial.print("\t");
      Serial.print(q.y);
      Serial.print("\t");
      Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      Serial.print("euler\t");
      Serial.print(euler[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(euler[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180 / M_PI);
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
  //Serial.println("Entro color");
  Serial.println(color);
  if (color > CalibNegro)
  {
    iReturn = true;
  }
  color = pulseIn(out, LOW);
  lcd.setCursor(0, 1);
  lcd.print(color);
  if (color > CalibNegro)
  {
    iReturn = true;
  }
  return iReturn;
}

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
  analogWrite(motDerE1, 190); //160  //190
  analogWrite(motDerE2, 0);
  
  analogWrite(motDerA1, 210); //220  //240
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 185); //182  //192
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 180); //172  //192
  analogWrite(motIzqA2, 0);
}

void Atras()
{
  analogWrite(motDerE1, 0); //160  //190
  analogWrite(motDerE2, 190);
  
  analogWrite(motDerA1, 0); //220  //240
  analogWrite(motDerA2, 210);

  analogWrite(motIzqE1, 0); //182  //192
  analogWrite(motIzqE2, 200);

  analogWrite(motIzqA1, 0); //172  //192
  analogWrite(motIzqA2, 200);
}

void Izquierda()
{
  analogWrite(motDerE1, 220);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 220);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 182);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 220);
}

void Derecha()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 220);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 220);

  analogWrite(motIzqE1, 182);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 220);
  analogWrite(motIzqA2, 0);
}

void DerechaM() //160 220 182 172
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 220);

  analogWrite(motDerA1, 220);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 182);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 220);
}

void IzquierdaM()//160 220 182 172
{
  analogWrite(motDerE1, 220);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 220);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 182);

  analogWrite(motIzqA1, 182);
  analogWrite(motIzqA2, 0);
}

void IzquierdaM30()
{
  EncDerE.write(0);
  IzquierdaM();
  int Enc = EncDerE.read();
  while (Enc < 8600)
  {
    Serial.println(Enc);
    Enc = EncDerE.read();
  }
  Detenerse();
}

void DerechaM30()
{
  EncDerE.write(0);
  DerechaM();
  int Enc = EncDerE.read();
  while (Enc > -8600)
  {
    //Serial.println(Enc);
    Enc = EncDerE.read();
  }
  Detenerse();
}

void GiroDer18()
{
  EncDerE.write(0);
  int Enc = EncDerE.read();
  //Serial.println(EncDerE.read());
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

void GiroDer90()
{
  lcd.clear();
  lcd.print("GiroDer90");
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
  //Serial.println(EncDerE.read());
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

void GiroIzq90()
{
  lcd.clear();
  lcd.print("GiroIzq90");
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
  lcd.clear();
  lcd.print("Adelante30");
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
  lcd.clear();
  lcd.print("Atras30");
  EncDerE.write(0);
  int Enc = EncDerE.read();
  while (Enc > (const30 * -1) + 500)
  {
    Atras();
    Enc = EncDerE.read();
    Serial.println(Enc);
  }
  Detenerse();
}

bool ParedDer()
{
  bool Pared = true;
  int Sharp = SharpDe.distance();
  Serial.println(Sharp);
  if (Sharp >= 18)
  {
    Pared = false;
  }
  return Pared;
}

bool ParedIzq()
{
  bool Pared = true;
  int Sharp = SharpIz.distance();
  Serial.println(Sharp);
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
  if (Sharp > 19)
  {
    Pared = false;
  }
  return Pared;
}

void RampaAbajoIzq()
{ /*
   lcd.clear();
   lcd.print("RampaAbajoIzq");
   int SharpDer = SharpDe.distance();
   int SharpEnf = SharpEn.distance();
   int SharpIzq = SharpIz.distance();
   int Roll = 0;
   if (SharpEnf < 19 && SharpDer < 18 && SharpIzq > 14)
   {
     lcd.print("entro");
     IzquierdaM30();
     delay(30);
     Roll = MPUR();
     lcd.setCursor(0, 1);
     lcd.print(Roll);
     if (Roll < -7)
     {
       IzquierdaM();
       delay(5000);
       lcd.clear();
       Detenerse();
       delay(30);
       GiroIzq90();
       delay(30);
       Acomodo();
       //delay(30);
       GiroIzq90();
       //delay(30);
       Acomodo();
       //delay(30);
       //Acejarse();
       //delay(80);
       Adelante30();
       //delay(30);
       Reset();
     }
     else
     {
       if (Negro())
       {
         DerechaM30();
         delay(30);
         Adelante();
         delay(30);
         Detenerse();
         //delay(30);
         Acomodo();
         //delay(30);
       }
       else
       {
         GiroDer90();
         //delay(30);
         Acomodo();
         delay(30);
         Adelante30();
         //delay(30);
         GiroIzq90();
         //delay(30);
         Acomodo();
         //delay(30);
       }
     }
   }
   */
}

bool HoyoNegro()
{
  Serial.println("Entro hoyo negro");
  bool Hoyo = false;
  lcd.clear();
  lcd.print(iX);
  lcd.print(", ");
  lcd.print(iY);
  lcd.setCursor(0, 1);
  lcd.print(iOption);
  delay(30);
  //lcd.clear();
  //lcd.print("HoyoNegro");
  int DistIzq = SharpIz.distance();
  iAnterior = iOption;
  if (Negro())
  {
    Hoyo = true;
    Atras30();
    bNegro [iX][iY] = true;
    bPos [iX][iY] = true;
    Acomodo();
    if (iAnterior == 1)
    {
      iY -= 1;
    }
    else if (iAnterior == 2)
    {
      iX -= 1;
    }
    else if (iAnterior == 3)
    {
      iX += 1;
    }
    else if (iAnterior == 4)
    {
      iY += 1;
    }
    //delay(30);
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
  Serial.println(Therm1);

  Serial.println(Therm3);
  Serial.println(Therm4);
  int Temp = 27;
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
  delay(30);
  Acejarse2();
  delay(30);
  Ultacomodo();
}

void AcejarseDerecha()
{
  int Dist = SharpDe.distance();

  //Serial.println("entro 1 if");
  Dist = SharpDe.distance();
  Serial.print("antes    "); Serial.println(Dist);

  while (Dist != 8 ) {
    if (Dist < 8)
    {
      IzquierdaM();
      while (Dist < 8)
      {

        Dist = SharpDe.distance();
        Serial.print("1zq    "); Serial.println(Dist);
      }
    }
    else if (Dist > 8)
    {
      DerechaM();
      while (Dist > 8)
      {

        Dist = SharpDe.distance();
        Serial.print("der    "); Serial.println(Dist);
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

  //Serial.println("entro 1 if");
  Dist2 = SharpIz.distance();

  while (Dist2 != 8) {
    if (Dist2 < 8)
    {
      DerechaM();
      while (Dist2 < 8)
      {

        Dist2 = SharpIz.distance();
        Serial.println(SharpIz.distance());
      }
    }
    else if (Dist2 > 8)
    {
      IzquierdaM();
      while (Dist2 > 8)
      {

        Dist2 = SharpIz.distance();
        Serial.println(SharpIz.distance());
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
  Serial.print(SharpEnf); Serial.print("\t"); Serial.println(U);
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
  for(int iI = 0; iI < 4; iI++)
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
    RampaAbajoIzq();
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

void SeguirDerecha()
{
  bool ParedD = ParedDer();
  bool ParedE = ParedEnf();

  if (ParedD == false)
  {
    GiroDer90();
    Acomodo();
    delay(30);
    Adelante30();
    Detenerse();
    delay(30);
  }
  else if (ParedE == false)
  {
    Adelante30();
    delay(30);
    Detenerse();
    delay(30);

  }
  else if (ParedD == true && ParedE == true)
  {
    GiroIzq90();
    Acomodo();
    delay(30);
    //Revisiones();
  }
  Revisiones();
  Acomodo();
}

/*
      1
      ^
      |
  3<-POS-> 2
      |
      v
      4
*/

void Pos1()
{
  bool ParedD = ParedDer();
  bool ParedE = ParedEnf();
  bool ParedI = ParedIzq();

  if (ParedD == false && bPos[iX + 1] [iY] == false)
  {
    //delay(80);
    GiroDer90();
    //delay(80);
    Acomodo();
    //delay(80);
    //Acomodo();
    //delay(80);
    Adelante30();
    iX += 1;
    bPos[iX][iY] = true;
    iOption = 2;
  }
  else if (ParedE == false && bPos[iX][iY + 1] == false)
  {
    //delay(80);
    Adelante30();
    //delay(80);
    iY += 1;
    bPos[iX][iY] = true;
    iOption = 1;
  }
  else if (ParedI == false && bPos[iX - 1] [iY] == false)
  {
    //delay(80);
    GiroIzq90();
    //delay(80);
    Acomodo();
    //delay(80);
    //Acomodo();
    //delay(80);
    Adelante30();
    iX -= 1;
    bPos[iX][iY] = true;
    iOption = 3;
  }
  else if (ParedI == true && ParedE == true && ParedD == true)
  {
    //delay(80);
    GiroDer90();
    //delay(80);
    Acomodo();
    //delay(60);
    GiroDer90();
    //delay(80);
    Acomodo();
    //delay(80);
    //Acomodo();
    //delay(80);
    Adelante30();
    iY -= 1;
    iOption = 4;
  }
  //En caso de que todo haya sido visitado se mueve a una aunque haya sido visitada con la unica condición que no sea negro
  else
  {
    if (ParedD == false && bNegro[iX + 1] [iY] == false)
    {
      //delay(80);
      GiroDer90();
      //delay(80);
      Acomodo();
      //delay(80);
      //Acomodo();
      //delay(80);
      Adelante30();
      iX += 1;
      iOption = 2;
    }
    else if (ParedE == false && bNegro[iX][iY + 1] == false)
    {
      //delay(80);
      Adelante30();
      //delay(80);
      iY += 1;
      iOption = 1;
    }
    else if (ParedI == false && bNegro[iX - 1] [iY] == false)
    {
      //delay(80);
      GiroIzq90();
      //delay(80);
      Acomodo();
      //delay(80);
      //Acomodo();
      //delay(80);
      Adelante30();
      iX -= 1;
      iOption = 3;
    }
    else
    {
      GiroDer90();
      //delay(80);
      Acomodo();
      GiroDer90();
      //delay(80);
      iOption = 4;
    }
  }
}

void Pos2()
{
  bool ParedD = ParedDer();
  bool ParedE = ParedEnf();
  bool ParedI = ParedIzq();

  if (ParedD == false && bPos[iX] [iY - 1] == false)
  {
    //delay(80);
    GiroDer90();
    //delay(80);
    Acomodo();
    //delay(80);
    //Acomodo();
    ////delay(80);
    Adelante30();
    iY -= 1;
    bPos[iX][iY] = true;
    iOption = 4;
  }
  else if (ParedE == false && bPos[iX + 1][iY] == false)
  {
    //delay(80);
    Adelante30();
    //delay(80);
    iX += 1;
    bPos[iX][iY] = true;
    iOption = 2;
  }
  else if (ParedI == false && bPos[iX] [iY + 1] == false)
  {
    //delay(80);
    GiroIzq90();
    //delay(80);
    Acomodo();
    //delay(80);
    //Acomodo();
    //delay(80);
    Adelante30();
    iY += 1;
    bPos[iX][iY] = true;
    iOption = 1;
  }
  else if (ParedI == true && ParedE == true && ParedD == true)
  {
    //delay(80);
    GiroDer90();
    Acomodo();
    //delay(60);
    GiroDer90();
    //delay(80);
    Acomodo();
    //delay(60);
    //Acomodo();
    //delay(80);
    Adelante30();
    iX -= 1;
    iOption = 3;
  }
  else
  {
    if (ParedD == false && bNegro[iX] [iY - 1] == false)
    {
      //delay(80);
      GiroDer90();
      //delay(80);
      Acomodo();
      //delay(80);
      //Acomodo();
      //delay(80);
      Adelante30();
      iY -= 1;
      iOption = 4;
    }
    else if (ParedE == false && bNegro[iX + 1][iY] == false)
    {
      //delay(80);
      Adelante30();
      //delay(80);
      iX += 1;
      iOption = 2;
    }
    else if (ParedI == false && bNegro[iX] [iY + 1] == false)
    {
      //delay(80);
      GiroIzq90();
      //delay(80);
      Acomodo();
      //delay(80);
      //Acomodo();
      //delay(80);
      Adelante30();
      iY += 1;
      iOption = 1;
    }
    else
    {
      GiroDer90();
      //delay(80);
      Acomodo();
      GiroDer90();
      //delay(80);
      iOption = 3;
    }
  }
}

void Pos3()
{
  bool  ParedD = ParedDer();
  bool ParedE = ParedEnf();
  bool ParedI = ParedIzq();

  if (ParedD == false && bPos[iX] [iY + 1] == false)
  {
    //delay(80);
    GiroDer90();
    //delay(80);
    Acomodo();
    //delay(80);
    //Acomodo();
    //delay(80);
    Adelante30();
    iY += 1;
    bPos[iX][iY] = true;
    iOption = 1;
  }
  else if (ParedE == false && bPos[iX - 1][iY] == false)
  {
    //delay(80);
    Adelante30();
    //delay(80);
    iX -= 1;
    bPos[iX][iY] = true;
    iOption = 3;
  }
  else if (ParedI == false && bPos[iX] [iY - 1] == false)
  {
    //delay(80);
    GiroIzq90();
    //delay(80);
    Acomodo();
    //delay(80);
    //Acomodo();
    //delay(80);
    Adelante30();
    iY -= 1;
    bPos[iX][iY] = true;
    iOption = 4;
  }
  else if (ParedI == true && ParedE == true && ParedD == true)
  {
    //delay(80);
    GiroDer90();
    Acomodo();
    //delay(60);
    GiroDer90();
    //delay(80);
    Acomodo();
    //delay(60);
    //Acomodo();
    //delay(80);
    Adelante30();
    iX += 1;
    iOption = 2;
  }
  else
  {
    if (ParedD == false && bNegro[iX] [iY + 1] == false)
    {
      //delay(80);
      GiroDer90();
      //delay(80);
      Acomodo();
      //delay(80);
      //Acomodo();
      //delay(80);
      Adelante30();
      iY += 1;
      iOption = 1;
    }
    else if (ParedE == false && bNegro[iX - 1][iY] == false)
    {
      //delay(80);
      Adelante30();
      //delay(80);
      iX -= 1;
      iOption = 3;
    }
    else if (ParedI == false && bNegro[iX] [iY - 1] == false)
    {
      //delay(80);
      GiroIzq90();
      //delay(80);
      //Acomodo();
      //delay(80);
      Acomodo();
      //delay(80);
      Adelante30();
      iY -= 1;
      iOption = 4;
    }
    else
    {
      GiroDer90();
      Acomodo();
      //delay(800);
      GiroDer90();
      iOption = 2;
    }
  }
}

//Opcion si esta observando hacia abajo
void Pos4()
{
  bool  ParedD = ParedDer();
  bool  ParedE = ParedEnf();
  bool ParedI = ParedIzq();

  if (ParedD == false && bPos[iX - 1] [iY] == false)
  {
    //delay(80);
    GiroDer90();
    //delay(80);
    Acomodo();
    //delay(80);
    //Acomodo();
    //delay(80);
    Adelante30();
    iX -= 1;
    bPos[iX][iY] = true;
    iOption = 3;
  }
  else if (ParedE == false && bPos[iX][iY - 1] == false)
  {
    //delay(80);
    Adelante30();
    //delay(80);
    iY -= 1;
    bPos[iX][iY] = true;
    iOption = 4;
  }
  else if (ParedI == false && bPos[iX + 1] [iY] == false)
  {
    //delay(80);
    GiroIzq90();
    //delay(80);
    Acomodo();
    //delay(80);
    //Acomodo();
    //delay(80);
    Adelante30();
    iX += 1;
    bPos[iX][iY] = true;
    iOption = 2;
  }
  else if (ParedI == true && ParedE == true && ParedD == true)
  {
    //delay(80);
    GiroDer90();
    Acomodo();
    //delay(60);
    GiroDer90();
    //delay(80);
    Acomodo();
    //delay(60);
    //Acomodo();
    //delay(80);
    Adelante30();
    iY += 1;
    iOption = 1;
  }
  else
  {
    if (ParedD == false && bNegro[iX - 1] [iY] == false)
    {
      //delay(80);
      GiroDer90();
      //delay(80);
      Acomodo();
      //delay(80);
      //Acomodo();
      //delay(80);
      Adelante30();
      iX -= 1;
      iOption = 3;
    }
    else if (ParedE == false && bNegro[iX][iY - 1] == false)
    {
      //delay(80);
      Adelante30();
      //delay(80);
      iY -= 1;
      iOption = 4;
    }
    else if (ParedI == false && bNegro[iX + 1] [iY] == false)
    {
      //delay(80);
      GiroIzq90();
      //delay(80);
      Acomodo();
      //delay(80);
      //Acomodo();
      //delay(80);
      Adelante30();
      iX += 1;
      iOption = 2;
    }
    else
    {
      GiroDer90();
      Acomodo();
      //delay(800);
      GiroDer90();
      iOption = 1;
    }
  }
}

void Algoritmo()
{
  iAnterior = iOption;
  /*
  lcd.clear();
  lcd.print(iX);
  lcd.print(", ");
  lcd.print(iY);
  lcd.setCursor(0, 1);
  lcd.print(iOption);
  */
  //delay(500);
  
  //Derecha
  if (iOption == 1)
  {
    Pos1();
  }
  //Enfrente
  else if (iOption == 2)
  {
    Pos2();
  }
  //Izquierda
  else if (iOption == 3)
  {
    Pos3();
  }
  //Atras
  else if (iOption == 4)
  {
    Pos4();
  }
  //delay(80);
  Revisiones();
  Acomodo();
  ////delay(500);
}


void Calibracion()
{
  lcd.clear();
  therm1.read();
  //therm2.read();
  therm3.read();
  therm4.read();

  int Therm1 = therm1.object();
  //int //therm2 = //therm2.object();
  int Therm3 = therm3.object();
  int Therm4 = therm4.object();

  lcd.setCursor(0, 0);
  lcd.print(Therm1);
  lcd.print("-");
  lcd.print(Therm3);
  lcd.print("-");
  lcd.print(Therm4);
  lcd.print("-");
  //lcd.print(//therm2);

  Negro();

  delay(1000);
}

void UltIzq()
{
  lcd.clear();
  delay(30);
  int U1 = sonar7.ping_cm();
  delay(30);
  int U2 = sonar8.ping_cm();
  lcd.print(U1);
  lcd.setCursor(0, 1);
  lcd.print(U2);
  //delay(1000);
  bool False = false;
  while (False == false)
  {
    if ((U1 - U2) < 0)
    {
      EncDerE.write(0);
      int Enc = EncDerE.read();
      Izquierda();
      while (Enc > ((const90 * 3 / 90) * -1 ))
      {
        Enc = EncDerE.read();
      }
      Detenerse();
    }
    else if ((U1 - U2) > 0)
    {
      EncDerE.write(0);
      int Enc = EncDerE.read();
      //Serial.println(EncDerE.read());
      Derecha();
      while (Enc < (const90 * 3 / 90))
      {
        Enc = EncDerE.read();
      }
    }
    else
    {
      False = true;
    }
    Detenerse();
    delay(30);
    U1 = sonar7.ping_cm();
    delay(30);
    U2 = sonar8.ping_cm();
  }
}

void UltDer()
{
  lcd.clear();
  delay(30);
  int U1 = sonar3.ping_cm();
  delay(30);
  int U2 = sonar4.ping_cm();
  lcd.print(U1);
  lcd.setCursor(0, 1);
  lcd.print(U2);
  //delay(1000);
  bool False = false;
  while (False == false)
  {
    if ((U1 - U2) < 0)
    {
      EncDerE.write(0);
      int Enc = EncDerE.read();
      //Serial.println(EncDerE.read());
      Izquierda();
      while (Enc > ((const90 * 3 / 90) * -1 ))
      {
        Enc = EncDerE.read();
      }
      Detenerse();
    }
    else if ((U1 - U2) > 0)
    {
      EncDerE.write(0);
      int Enc = EncDerE.read();
      //Serial.println(EncDerE.read());
      Derecha();
      while (Enc < (const90 * 3 / 90))
      {
        Enc = EncDerE.read();
      }
      Detenerse();
    }
    else {
      False = true;
    }
    delay(30);
    U1 = sonar3.ping_cm();
    delay(30);
    U2 = sonar4.ping_cm();
  }
}

void Ultacomodo()
{
  int iCounter = 0;
  // while(iCounter < 2)
  //{
  if (ParedIzq())
  {
    UltIzq();
  }
  else if (ParedDer())
  {
    UltDer();
  }
  iCounter++;
  //}
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

void loop() {
  // put your main code here, to run repeatedly:
  lcd.backlight();
  Algoritmo();
}
