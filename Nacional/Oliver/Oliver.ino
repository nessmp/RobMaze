#include <Encoder.h> //para que crees...
#include <NewPing.h>
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>
#include <SharpIR.h> //sharps
#include <Servo.h>
#include "I2Cdev.h" //MPU
#include "MPU6050_6Axis_MotionApps20.h" //MPU
#include <SparkFunMLX90614.h>
#define MAX_DISTANCE 200
#define model 1080 //modelo del sharp GP2Y0A21Y
#define OUTPUT_READABLE_YAWPITCHROLL //Yaw Pitch Roll del MPU

/////////////////
//////COLOR//////
/////////////////

const byte s0 = 9;
const byte s1 = 10;
const byte s2 = 12;
const byte s3 = 11;
const byte out = 13;

int red = 0;
int green = 0;
int blue = 0;
String colon = "";

/////////////////
//////SERVO//////
/////////////////

Servo Dispensador;
const int pinservo = 52;
const int PulsoMinimo = 650;
const int PulsoMaximo = 2550;

//////////////////
//////CALOR///////
//////////////////

IRTherm therm1;
IRTherm therm2;
IRTherm therm3;
IRTherm therm4;

//////////////////
//////SHARPS//////
//////////////////

byte Enf = A0;
byte Izq = A1;
byte Der =  A2;

SharpIR SharpEn(Enf, 25, 93, model);
SharpIR SharpIz(Izq, 25, 93, model);
SharpIR SharpDe(Der, 25, 93, model);


//////////////////
/////MOTORES//////
//////////////////

byte motDerE1 = 8;
byte motDerE2 = 9;

byte motDerA1 = 10;
byte motDerA2 = 11;

byte motIzqE1 = 4;
byte motIzqE2 = 5;

byte motIzqA1 = 6;
byte motIzqA2 = 7;

const byte MotD = 200;
const byte MotI = 200;

//////////////////
/////ENCODERS/////
//////////////////

Encoder EncDerE(19, 18);
Encoder Enc2(2, 3);

long oldPosition  = -999;

int const90 = 3450;

const int const30 = 5500;

//////////////////
///ULTRASONICOS///
//////////////////

byte Trigger1 = 28;
byte Echo1 = 26;

NewPing sonar1(Trigger1, Echo1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar1.ping_cm();

//NO FUNCIONO
byte Trigger2 = 38;
byte Echo2 = 36;

//NO FUNCIONO
NewPing sonar2(Trigger2, Echo2, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar1.ping_cm();


byte Trigger3 = 42;
byte Echo3 = 40;


NewPing sonar3(Trigger3, Echo3, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte Trigger4 = 51;
byte Echo4 = 53;

NewPing sonar4(Trigger4, Echo4, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte Trigger5 = 47;
byte Echo5 = 49;

NewPing sonar5(Trigger5, Echo5, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte Trigger6 = 27;
byte Echo6 = 29;

NewPing sonar6(Trigger6, Echo6, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte Trigger7 = 23;
byte Echo7 = 25;

NewPing sonar7(Trigger7, Echo7, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();

byte Trigger8 = 24;
byte Echo8 = 22;

NewPing sonar8(Trigger8, Echo8, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonar8.ping_cm();


LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

///////////
////MPU////
///////////

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

void setup() {
  Serial.begin(115200);

  //MPU
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
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

  //CALOR
  therm1.begin(0x1C);
  therm1.setUnit(TEMP_C);

  therm2.begin(0x2C);
  therm2.setUnit(TEMP_C);

  therm3.begin(0x3C);
  therm3.setUnit(TEMP_C);

  therm4.begin(0x4C);
  therm4.setUnit(TEMP_C);

  //SERVO
  Dispensador.attach(pinservo, PulsoMinimo, PulsoMaximo);

  //MOTOR
  pinMode(motDerE1, OUTPUT);
  pinMode(motDerE2, OUTPUT);

  pinMode(motDerA1, OUTPUT);
  pinMode(motDerA2, OUTPUT);

  pinMode(motIzqE1, OUTPUT);
  pinMode(motIzqE2, OUTPUT);

  pinMode(motIzqA1, OUTPUT);
  pinMode(motIzqA2, OUTPUT);

  //COLOR
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);

  //LCD
  lcd.begin(16, 2);  // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.backlight(); // finish with backlight on
  lcd.setCursor(0, 0); //Start at character 4 on line 0
  lcd.print(";)");
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
    return 9988;
    //Serial.println(F("FIFO overflow!"));

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
    MPU_Y = ypr[0] * 180 / M_PI;
#endif
  }
  return MPU_Y;
}

//COLOR
String Color()
{
  String color = "Blanco";
  
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

  if( == red &&  == green &&  == blue)
  {
    color = "Negro";
  }
  
  return color;
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
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, MotD);

  analogWrite(motDerA1, MotD);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, MotD);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, MotD);

  //Victima();
}

void Atras()
{
  analogWrite(motDerE1, 200);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 200);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 90);

  analogWrite(motIzqA1, 90);
  analogWrite(motIzqA2, 0);
}

void DerechaM()
{
  analogWrite(motDerE1, 150);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 150);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 90);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 90);
  analogWrite(motIzqA2, 0);

}

void Derecha()
{
  analogWrite(motDerE1, 200);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 200);

  analogWrite(motIzqE1, 90);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 90);
}

//funcion para moverse hacia izquierda
void IzquierdaM()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 145);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 145);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 80);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 80);
}

void Izquierda()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 200);

  analogWrite(motDerA1, 200); //190
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 90); //100

  analogWrite(motIzqA1, 90); //90
  analogWrite(motIzqA2, 0);
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
  delay(500);
  EncDerE.write(0);
  int Enc = EncDerE.read();

  while (Encoder1() < const30)
  {
    Adelante();
    Encoder1();
    Victima();
  }
  Detenerse();
}

bool ParedDer()
{
  bool Pared = true;
  int Sharp = SharpDe.distance();
  if (Sharp > 17)
  {
    Pared = false;
  }
  return Pared;
}

bool ParedIzq()
{
  bool Pared = true;
  int Sharp = SharpIz.distance();
  if (Sharp > 17)
  {
    Pared = false;
  }
  return Pared;
}

bool ParedEnf()
{
  bool Pared = true;
  int Sharp = SharpEn.distance();
  if (Sharp > 17)
  {
    Pared = false;
  }
  return Pared;
}

bool Victima()
{
  bool Victima = false;
  therm1.read();
  int Calor = therm1.object();
  Serial.println(Calor);
  if (Calor > 23)
  {
    Victima = true;
    Detenerse();
    Dispensador.write(113);
    delay(1000);
    Dispensador.write(75);
    delay(1000);
  }
  return Victima;
}

void Detectar()
{
  if (Victima())
  {
    Detenerse();
    Dispensador.write(119);
    delay(1000);
    Dispensador.write(75);
    delay(1000);
  }
}

bool AgujeroNegro()
{
  String Negro = Color();
  bool AgNegro = false;
  if(Negro == "Negro")
  {
    Detenerse();
    delay(500);
    Atras30();
    AgNegro = true;
  }
  return AgNegro;
}

void Acejarse()
{
  int Dist = SharpDe.distance();

  if (Dist < 7)
  {
    while (Dist < 7)
    {
      IzquierdaM();
      Dist = SharpDe.distance();
    }
    if (Dist > 9)
    {
      while (Dist > 9)
      {
        DerechaM();
        Dist = SharpDe.distance();
      }
    }
    Detenerse();
  }
  else if (Dist > 9)
  {
    while (Dist > 9)
    {
      DerechaM();
      Dist = SharpDe.distance();
    }
    if (Dist < 7)
    {
      while (Dist < 7)
      {
        IzquierdaM();
        Dist = SharpDe.distance();
      }
    }
    Detenerse();
  }
}


void Acomodo()
{
  int pos = MPUY();
  double CountEnc = const90 / 90; //
  bool Listo = false;

  do {
    if (pos > 90 && pos < 135)
    {
      double New = (pos - 90) * CountEnc;
      EncDerE.write(0);
      while (Encoder1() < New)
      {
        Izquierda();
        Encoder1();
      }
      Detenerse();
    }
    else if (pos < 90 && pos > 45)
    {
      double New = (90 - pos) * CountEnc;
      EncDerE.write(0);
      while (Encoder1() < New)
      {
        Derecha();
        Encoder1();
      }
      Detenerse();
    }

    else if (pos > 0 && pos < 45)
    {
      double New = pos * CountEnc;
      EncDerE.write(0);
      while (Encoder1() < New)
      {
        Izquierda();
        Encoder1();
      }
      Detenerse();
    }
    else if (pos < 0 && pos > -45)
    {
      double New = (pos * -1) * CountEnc;
      EncDerE.write(0);
      while (Encoder1() < New)
      {
        Derecha();
        Encoder1();
      }
      Detenerse();
    }

    else if (pos > -90 && pos < -45)
    {
      double New = (pos + 90) * CountEnc;
      EncDerE.write(0);
      while (Encoder1() < New)
      {
        Izquierda();
        Encoder1();
      }
      Detenerse();
    }
    else if (pos < -90 && pos > -135)
    {
      double New = (pos + 90) * -1 * CountEnc;
      EncDerE.write(0);
      while (Encoder1() < New)
      {
        Derecha();
        Encoder1();
      }
      Detenerse();
    }

    else if (pos < 179 && pos > 135)
    {
      double New = (179 - pos) * CountEnc;
      EncDerE.write(0);
      while (Encoder1() < New)
      {
        Derecha();
        Encoder1();
      }
      Detenerse();
    }
    else if (pos > -179 && pos < -135)
    {
      double New = (pos + 179) * CountEnc;
      EncDerE.write(0);
      while (Encoder1() < New)
      {
        Izquierda();
        Encoder1();
      }
      Detenerse();
    }
    Detenerse();
    pos = MPUY();
    if (90 == pos || 91 == pos || 89 == pos || 0 == pos || 1 == pos || 2 == pos || -90 == pos || -91 == pos || -89 == pos || 179 == pos || 178 == pos || -179 == pos || -178 == pos)
    {
      Listo = true;
    }
  } while (Listo = false);
}

void SeguirDerecha()
{
  bool ParedD = ParedDer();
  bool ParedE = ParedEnf();
  delay(100);
  ParedD = ParedDer();
  ParedE = ParedEnf();
  delay(100);
  ParedD = ParedDer();
  ParedE = ParedEnf();

  if (ParedD == false)
  {
    GiroDer90();
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
    //Acejarse();
  }
  else if (ParedD == true && ParedE == true)
  {
    GiroIzq90();
    delay(100);
    //Acejarse();
  }
  Victima();
  Acomodo();
  Acejarse();
  Acomodo();
}

void loop() {
  // put your main code here, to run repeatedly:

}
