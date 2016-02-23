#include <Encoder.h> //para que crees...
#include <SharpIR.h> //sharps
#include <Wire.h> //comunicaion
#include "I2Cdev.h" //MPU
#include "MPU6050_6Axis_MotionApps20.h" //MPU

#define MAX_DISTANCE 200 //distancia max detectada por los ultrasonicos
#define model 1080 //modelo del sharp GP2Y0A21Y
#define OUTPUT_READABLE_YAWPITCHROLL //Yaw Pitch Roll del MPU

//MPU
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

Quaternion q;  
VectorFloat gravity; 

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

float MPU_Y;

byte motDerE1 = 7;
byte motDerE2 = 6;

byte motDerA1 = 4;
byte motDerA2 = 5;

byte motIzqE1 = 8;
byte motIzqE2 = 9;

byte motIzqA1 = 10;
byte motIzqA2 = 11;

byte Enf = A15;
byte Der =  A14;

Encoder EncDerE(19, 18);
SharpIR SharpEn(Enf, 25, 93, model);
SharpIR SharpDe(Der, 25, 93, model);

long oldPosition  = -999;

int const90 = 4500;

int const30 = 5700;

void setup() {
  Serial.begin(115200);

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

  pinMode(motDerE1, OUTPUT);
  pinMode(motDerE2, OUTPUT);

  pinMode(motDerA1, OUTPUT);
  pinMode(motDerA2, OUTPUT);

  pinMode(motIzqE1, OUTPUT);
  pinMode(motIzqE2, OUTPUT);

  pinMode(motIzqA1, OUTPUT);
  pinMode(motIzqA2, OUTPUT);

  //MPU

  Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(20);
    mpu.setYGyroOffset(-7);
    mpu.setZGyroOffset(87);
    mpu.setZAccelOffset(1402); // 1688 factory default for my test chip

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
}

//Regresa el valor de YAW del MPU
int MPUY()
{
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
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
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            /*
            Serial.print("ypr\t");
            Serial.println(ypr[0] * 180/M_PI);
            */
            MPU_Y = ypr[0] * 180/M_PI;
        #endif
    }
    return MPU_Y;
}

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
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 255);

  analogWrite(motDerA1, 255);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 255);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 255);
}

//funcion para moverse hacia atras
void Atras()
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

void DerechaM()
{
  analogWrite(motDerE1, 130);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 130);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 150);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 150);
  analogWrite(motIzqA2, 0);

}

void Derecha()
{
  analogWrite(motDerE1, 255);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 255);

  analogWrite(motIzqE1, 255);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 255);

}

//funcion para moverse hacia izquierda
void IzquierdaM()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 150);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 150);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 150);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 150);
}

void Izquierda()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 255);

  analogWrite(motDerA1, 255);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 255);

  analogWrite(motIzqA1, 255);
  analogWrite(motIzqA2, 0);
}

//Giros
void GiroDer90()
{

  delay(1000);
  EncDerE.write(0);
  while (Encoder1() > const90 * -1)
  {
    Derecha();
    Encoder1();
  }
  Detenerse();
}

void GiroIzq90()
{

  delay(1000);
  EncDerE.write(0);

  while (Encoder1() < const90)
  {
    Izquierda();
    Encoder1();
  }
  Detenerse();
}

//Giros con MPU
void GiroDer90MPU()
{
  int giro = MPUY();
  int meta = giro + 90;
  //Serial.println(giro);
  if(meta >= 180)
  {
    meta -= 359;
  }
  while (giro != meta)
  {
    Derecha();
    giro = MPUY();
  }
}

void GiroIzq90MPU()
{  
  int giro = MPUY();
  int meta = giro - 90;
  //Serial.println(giro);
  if(meta <= -180)
  {
    meta += 359;
  }
  while (giro != meta)
  {
    Izquierda();
    giro = MPUY();
  }
}

//INCOMPLETA
void AcomodoMpu()
{
  int giro = MPUY();
  int P90 = 90;
  int P179 = 179;
  int P0 = 0;
  int N90 = -90;
  int Meta;
}

//Avances de 30


void Atras30()
{


  EncDerE.write(0);

  while (Encoder1() > const30 * -1)
  {
    Atras();
    Encoder1();
  }
  Detenerse();
}

void Adelante30()
{
  EncDerE.write(0);
  int Enc = EncDerE.read();

  while (Enc < const30)
  {
    Adelante();
    Enc = EncDerE.read();
  }
  Detenerse();
}

bool ParedDer()
{
  bool Pared = true;
  int Sharp = SharpDe.distance();
  if (Sharp > 25)
  {
    Pared = false;
  }
  return Pared;
}

bool ParedEnf()
{
  bool Pared = true;
  int Sharp = SharpEn.distance();
  if (Sharp > 25)
  {
    Pared = false;
  }
  return Pared;
}

void SeguirDerecha()
{
  bool ParedD = ParedDer();
  bool ParedE = ParedEnf();
  if (ParedD == false)
  {
    GiroDer90MPU();
    delay(200);
    Adelante30();
    delay(1000);
    Detenerse();
    delay(500);
  }
  else if (ParedE == false)
  {
    Adelante30();
    delay(100);
    Detenerse();
    delay(1000);
  }
  else if (ParedD == true && ParedE == true)
  {
    GiroIzq90MPU();
    delay(100);
  }
}

void loop() {
  
}
