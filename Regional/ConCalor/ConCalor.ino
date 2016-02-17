#include <Encoder.h> //para que crees...
#include <NewPing.h> //ultrasonicos
#include <SharpIR.h> //sharps
#include <Servo.h>
#include <i2cmaster.h>

#define MAX_DISTANCE 200 //distancia max detectada por los ultrasonicos
#define model 1080 //modelo del sharp GP2Y0A21Y

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
  Serial.begin(9600);

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

  //calor
  i2c_init(); //Initialise the i2c bus
  PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups
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

//NO FUNCIONA
void Centrar()
{
  //deseada, 8cm
  int Pos = SharpDe.distance();
  if (Pos > 8 || Pos < 8)
  {
    do {
      if (Pos > 8)
      {
        do {
          DerechaM();
          Pos = SharpDe.distance();
        } while (Pos > 8);
      }
      else if (Pos < 8)
      {
        do {
          IzquierdaM();
          Pos = SharpDe.distance();
        } while (Pos < 8);
      }
    } while (Pos > 8 || Pos < 8);
  }
}

//Calor
int Calor()
{
  int dev = 0x1C << 1;
  int data_low = 0;
  int data_high = 0;
  int pec = 0;

  i2c_start_wait(dev + I2C_WRITE);
  i2c_write(0x07);

  // read
  i2c_rep_start(dev + I2C_READ);
  data_low = i2c_readAck(); //Read 1 byte and then send ack
  data_high = i2c_readAck(); //Read 1 byte and then send ack
  pec = i2c_readNak();
  i2c_stop();

  //This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
  double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
  double tempData = 0x0000; // zero out the data
  int frac; // data past the decimal point

  // This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
  tempData = (double)(((data_high & 0x007F) << 8) + data_low);
  tempData = (tempData * tempFactor) - 0.01;

  int celcius = tempData - 273.15;
  Serial.println(celcius);

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
  SeguirDerecha();
  //Calor();
}
