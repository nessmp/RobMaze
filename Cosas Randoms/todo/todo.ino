#include <Encoder.h> //para que crees...
#include <NewPing.h> //ultrasonicos
#include <SharpIR.h> //sharps
#include <Servo.h> //Servos duh.
#include <LiquidCrystal_I2C.h> //lcd
#include <Wire.h> //comunicaion
#include <SparkFunMLX90614.h> //melexis
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

//Sensor de color
int s0  = 32;
int s1  = 34;
int s2  = 36;
int s3  = 38;
int out  = 40;

byte motDerE1 = 7;
byte motDerE2 = 6;

byte motDerA1 = 4;
byte motDerA2 = 5;

byte motIzqE1 = 8;
byte motIzqE2 = 9;

byte motIzqA1 = 10;
byte motIzqA2 = 11;

byte TriggEA = 23;
byte EchoEA = 25;

byte TriggEB = 51;
byte EchoEB = 53;

byte TriggDA  = 47;
byte EchoDA = 49;

byte TriggDB = 43;
byte EchoDB = 45;

byte TriggAA = 39;
byte EchoAA = 41;

byte TriggAB = 35;
byte EchoAB = 37;

byte TriggIA = 31;
byte EchoIA = 33;

byte TriggIB = 27;
byte EchoIB = 29;

byte Enf = A3;
byte Der = A5;
byte Der2 = A4;

//LED
int Led = 33;

Encoder EncDerE(3, 14);

SharpIR SharpEn(Enf, 25, 93, model);
SharpIR SharpDe(Der, 25, 93, model);
SharpIR SharpDe2(Der2, 25, 93, model);

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

IRTherm therm;

NewPing Ult(TriggAB, EchoAB, MAX_DISTANCE);

int red = 0;
int green = 0;
int blue = 0;
String colon = "";

long oldPosition  = -999;

int estable = 2750;

int const90 = 4500;

int const30 = 5700;

int pos = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);

  pinMode(motDerE1, OUTPUT);
  pinMode(motDerE2, OUTPUT);

  pinMode(motDerA1, OUTPUT);
  pinMode(motDerA2, OUTPUT);

  pinMode(motIzqE1, OUTPUT);
  pinMode(motIzqE2, OUTPUT);

  pinMode(motIzqA1, OUTPUT);
  pinMode(motIzqA2, OUTPUT);

  pinMode (Enf, INPUT);
  pinMode (Der, INPUT);

  //LED
  pinMode(Led, OUTPUT);
  digitalWrite(Led, LOW);

  therm.begin(0x2C);
  therm.setUnit(TEMP_C);
  lcd.begin(16,2);
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Listo?");

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

bool color()
{
  bool Negro = false;
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  //count OUT, pRed, RED
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s3, HIGH);
  //count OUT, pBLUE, BLUE
  blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s2, HIGH);
  //count OUT, pGreen, GREEN
  green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);

  Serial.println("red: ");
  Serial.println(red);
  Serial.println("green: ");
  Serial.println(green);
  Serial.println("blue: ");
  Serial.println(blue);
  delay(1000);
  //red: 2350-2440
  //green: 2160-2240
  //blue:1640-1720

  if (red > 1000 && green > 1400 && blue > 1200 )
  {
    Negro = true;
  }

  return Negro;
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

void MovRampa()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 255);

  analogWrite(motDerA1, 255);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 223);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 223);
}

//Calor
int Calor()
{
  float celcius;
  lcd.clear();
  therm.read();
  celcius = therm.object();

  return celcius;
}

void Blink()
{
  for (int iI = 0; iI < 6; iI++)
  {
    digitalWrite(Led, HIGH);
    delay(500);
    digitalWrite(Led, LOW);
    delay(500);
  }
}

void DetectarVictima()
{
  int temp = Calor();
  if (temp > 26)
  {
    Detenerse();
    Blink();
  }
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
    //derecha();
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
    //derecha();
    giro = MPUY();
  }
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
    DetectarVictima();
    Enc = EncDerE.read();
  }
  Detenerse();
}


//Cuentas del encoder
int Encoder1()
{
  long newPosition = EncDerE.read();

  if (newPosition != oldPosition) {
    oldPosition = newPosition;

  }
  return newPosition;
}

int SharpEnf()
{
  return SharpEn.distance();
}

int SharpDer()
{
  return SharpDe.distance();
}

void Negro()
{
  Atras30();
  delay(250);
  GiroIzq90();
  delay(250);
  if (ParedEnf() == true)
  {
    GiroIzq90();
    delay(250);
  }
  else
  {
    Adelante30();
    delay(250);
  }
}

void AgujeroNegro()
{
  if (color() == true)
  {
    Negro();
  }
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

int Y()
{
  int y;
  byte piny = 1;
  bool rampa;

  y = map(analogRead(piny), 0, 1023, 0, 10000);
  return y;
}

void Rampa()
{

  int y;
  byte piny = 1;
  bool rampa;

  y = 0;
  for (int i = 0; i < 10; i++)
  {
    y += (map(analogRead(piny), 0, 123, 0, 1000));
  }
  y = y / 10;

  if (y < estable - 150 || y > estable + 150)
  {
    while (y < estable - 50 || y > estable + 50)
    {
      Adelante();
      y = 0;
      for (int i = 0; i < 10; i++)
      {
        y += (map(analogRead(piny), 0, 123, 0, 1000));
      }
      y = y / 10;
    }
    Detenerse();
  }
}

void RampaAntes()
{
  bool Rampa = false;
  int Ultra = SharpDe.distance();
  int Sharp = SharpDe2.distance();
  int Diff = Sharp - Ultra;
  Serial.println(Diff);
  if (Diff == 11 || Diff == 12 || Diff == 13 || Diff == 14 || Diff == 15)
  {
    GiroDer90();
    Adelante30();
    int y;
    byte piny = 1;
    bool rampa;

    y = 0;
    for (int i = 0; i < 10; i++)
    {
      y += (map(analogRead(piny), 0, 123, 0, 1000));
    }
    y = y / 10;

    if (y < estable - 150 || y > estable + 150)
    {
      while (y < estable - 50 || y > estable + 50)
      {
        MovRampa();
        y = 0;
        for (int i = 0; i < 10; i++)
        {
          y += (map(analogRead(piny), 0, 123, 0, 1000));
        }
        y = y / 10;
      }
      Detenerse();
    }
  }
}

void SeguirDerecha()
{
  bool ParedD = ParedDer();
  bool ParedE = ParedEnf();
  if (ParedD == false)
  {
    GiroDer90();
    delay(200);
    Adelante30();
    delay(1000);
    Detenerse();
    delay(500);
    Rampa();
    delay(100);
    AgujeroNegro();
    delay(100);
  }
  else if (ParedE == false)
  {
    Adelante30();
    delay(100);
    Detenerse();
    delay(1000);
    Rampa();
    delay(100);
    AgujeroNegro();
    delay(100);
  }
  else if (ParedD == true && ParedE == true)
  {
    GiroIzq90();
    delay(100);
  }
}

void loop() {
    Serial.println("entro");
    GiroIzq90MPU();
    Serial.println("salio");
}
