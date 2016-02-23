#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

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

///////////
//MOTORES//
///////////

byte MotDer = 4;
byte MotDer2 = 5;

byte MotIzq = 6;
byte MotIzq2 = 7;

void setup() {
  Serial.begin(115200);

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
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

    pinMode(MotDer, OUTPUT);
    pinMode(MotDer2, OUTPUT);

    pinMode(MotIzq, OUTPUT);
    pinMode(MotIzq2, OUTPUT);
}

void adelante()
{
  analogWrite(MotDer, 0);
  analogWrite(MotDer2, 130);

  analogWrite(MotIzq, 0);
  analogWrite(MotIzq2, 150);
}

void izquierda()
{
  analogWrite(MotDer, 0);
  analogWrite(MotDer2, 140);

  analogWrite(MotIzq, 140);
  analogWrite(MotIzq2, 0);
}

void derecha()
{
  analogWrite(MotDer, 140);
  analogWrite(MotDer2, 0);

  analogWrite(MotIzq, 0);
  analogWrite(MotIzq2, 140);
}

void detenerse()
{
  analogWrite(MotDer, 0);
  analogWrite(MotDer2, 0);

  analogWrite(MotIzq, 0);
  analogWrite(MotIzq2, 0);
}

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

void GiroDer90()
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
    derecha();
    giro = MPUY();
  }
  detenerse();
}

void GiroIzq90()
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
    izquierda();
    giro = MPUY();
  }
  detenerse();
}

void loop() {
  delay(5000);
  GiroIzq90();
  Serial.println("Salio");
  delay(5000);
  //Serial.println(MPUY());
}
