#include <Wire.h> // I2C library, required for MLX90614
#include <SparkFunMLX90614.h> // SparkFunMLX90614 Arduino library
#include <Encoder.h> //para que crees...
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h> //para la pantalla LCD
#include <NewPing.h> //ultrasonicos
#define MAX_DISTANCE 200 //distancia max detectada por los ultrasonicos
#include <SharpIR.h> //sharps
#define model 1080 //modelo del sharp GP2Y0A21Y
#include <i2cmaster.h>

/////////
//pines//
/////////

//Sensor de color
const int s0  
const int s1  
const int s2  
const int s3  
const int out  

//motores
byte motDerE1; 
byte motDerE2; 

byte motDerA1;
byte motDerA2;

byte motIzqE1;
byte motIzqE2;

byte motIzqA1;
byte motIzqA2;

//Ultrasonico enfrente A
byte TriggEA;
byte EchoEA;

//Ultrasonico Enfrente B

byte TriggEB;
byte EchoEB;

//Ultrasonico Derecha A

byte TriggDA;
byte EchoDA;

//Ultrasonico Derecha B

byte TriggDB;
byte EchoDB;


//Ultrasonico Atras A

byte TriggAA;
byte EchoAA;

//Ultrasonico Atrás B

byte TriggAB;
byte EchoAB;

//Ultrasonico Izquierda A

byte TriggIA;
byte EchoIA;

//Ultrasonico Izquierda B

byte TriggIB;
byte EchoIB;

//Sharps
byte Enf;
byte Der;

//////////
//clases//
//////////

//Sensores de calor
IRTherm CalorEnf;
IRTherm CalorDer;
IRTherm CalorAtr;
IRTherm CalorIzq;

//LCD
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

//encoders
Encoder EncDerE(, );
Encoder EncDerA(, );
Encoder EncIzqE(, );
Encoder EncIzqA(, );

//ultrasonicos
NewPing UltEA(TriggEA, EchoEA, MAX_DISTANCE);
NewPing UltEB(TriggEB, EchoEB, MAX_DISTANCE);
NewPing UltDA(TriggDA, EchoDA, MAX_DISTANCE);
NewPing UltDB(TriggDB, EchoDB, MAX_DISTANCE);
NewPing UltAA(TriggAA, EchoAA, MAX_DISTANCE);
NewPing UltAB(TriggAB, EchoAB, MAX_DISTANCE);
NewPing UltIA(TriggIA, EchoIA, MAX_DISTANCE);
NewPing UltIB(TriggIB, EchoIB, MAX_DISTANCE);

//Sharp
SharpIR SharpEnf(Enf, 25, 93, model);
SharpIR SharpDer(Der, 25, 93, model);

//////////////
// Variables//
//////////////

//Variables para el sensor de color
int red = 0;  
int green = 0;  
int blue = 0;  
String colon = "";

//Encoder
long oldPosition  = -999;

/////////
//SetUp//
/////////

void setup() {
  Serial.begin(9600);
  //Direcciones I2C de los sensores de calor
  CalorEnf.begin(0x1C)
  CalorDer.begin(0x2C)
  CalorAtr.begin(0x3C)
  CalorIzq.begin(0x4C)
  //Establece sensores de calor a Celsius
  CalorEnf.setUnit(TEMP_C); 
  CalorDer.setUnit(TEMP_C); 
  CalorAtr.setUnit(TEMP_C); 
  CalorIzq.setUnit(TEMP_C); 

  //Sensor de color
  pinMode(s0, OUTPUT);  
  pinMode(s1, OUTPUT);  
  pinMode(s2, OUTPUT);  
  pinMode(s3, OUTPUT);  

  digitalWrite(s0, HIGH);  
  digitalWrite(s1, HIGH);  

  //motores
  pinMode(motDerE1, OUTPUT);
  pinMode(motDerE2, OUTPUT);
  
  pinMode(motDerA1, OUTPUT);
  pinMode(motDerA2, OUTPUT);

  pinMode(motIzqE1, OUTPUT);
  pinMode(motIzqE2, OUTPUT);

  pinMode(motIzqA1, OUTPUT);
  pinMode(motIzqA2, OUTPUT);

  //LCD
  lcd.begin(16,2); //la inicializa
  lcd.backlight(); //enciende el led de la pantalla

  //Sharps
  pinMode (Enf, INPUT);
  pinMode (Der, INPUT);

  //Calor
  i2c_init(); //Initialise the i2c bus
  PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups
}

/////////////
//Funciones//
/////////////

//función para leer datos del sensor de color
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

  if(red && green && blue)
  {
    Negro = true;
  }

  return Negro;
}

//Acciones al encontrar zona negra
void Negro()
{
  Atras30();
  GiroIzq90();
  Adelante30();
}

////////////////////
//funciones de mov//
////////////////////

//Funcion para apagar motores
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
  analogWrite(motDerE1, 255);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 255);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 255);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 255);
  analogWrite(motIzqA2, 0);
}

//funcion para moverse hacia atras
void Atras()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 255);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 255);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 255);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 255);

}

//funcion para moverse hacia adelante
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

//funcion para ultrasonico de enfrente izq
int UltEA()
{
  return UltEA.ping_cm();
  delay(1);
}

//funcion para ultrasonico de enfrente der
int UltEB()
{
  return UltEB.ping_cm();
  delay(1);
}

//funcion para ultrasonico de derecha izq
int UltDA()
{
  return UltDA.ping_cm();
  delay(1);
}

//funcion para ultrasonico derecha  der
int UltDB()

{
  return UltDB.ping_cm();
  delay(1);
}

//funcion para ultrasonico de atras izq
int UltAA()
{
  return UltAA.ping_cm();
  delay(1);
}

//funcion para ultrasonico de atras der
int UltAB()
{
  return UltAB.ping_cm();
  delay(1);
}

//funcion para ultrasonico de izquierda izq
int UltIA()
{
  return UltIA.ping_cm();
  delay(1);
}

//funcion para ultrasonico de izquierda der
int UltIB()
{
  return UltIB.ping_cm();
  delay(1);
}

int SharpEnf()
{
  return SharpEnf.distance()
}

int SharpDer()
{
  return SharpDer.distance()
}

//regresa el calor detectado por el sensor de enfrente, REVISAR EL ADDRESS
int CalorEnf()
{
    int Calor;
    int dev = 0x1C<<1;
    int data_low = 0;
    int data_high = 0;
    int pec = 0;
    
    i2c_start_wait(dev+I2C_WRITE);
    i2c_write(0x07);
    
    // read
    i2c_rep_start(dev+I2C_READ);
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
    tempData = (tempData * tempFactor)-0.01;
    
    float celcius = tempData - 273.15;
    Calor = celcius;
    return Calor;
}

int CalorDer()
{
    int Calor;
    int dev = 0x2C<<1;
    int data_low = 0;
    int data_high = 0;
    int pec = 0;
    
    i2c_start_wait(dev+I2C_WRITE);
    i2c_write(0x07);
    
    // read
    i2c_rep_start(dev+I2C_READ);
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
    tempData = (tempData * tempFactor)-0.01;
    
    float celcius = tempData - 273.15;
    Calor = celcius;
    return Calor;
}

int CalorAtr()
{
    int Calor;
    int dev = 0x3C<<1;
    int data_low = 0;
    int data_high = 0;
    int pec = 0;
    
    i2c_start_wait(dev+I2C_WRITE);
    i2c_write(0x07);
    
    // read
    i2c_rep_start(dev+I2C_READ);
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
    tempData = (tempData * tempFactor)-0.01;
    
    float celcius = tempData - 273.15;
    Calor = celcius;
    return Calor;
}

int CalorIzq()
{
    int Calor;
    int dev = 0x4C<<1;
    int data_low = 0;
    int data_high = 0;
    int pec = 0;
    
    i2c_start_wait(dev+I2C_WRITE);
    i2c_write(0x07);
    
    // read
    i2c_rep_start(dev+I2C_READ);
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
    tempData = (tempData * tempFactor)-0.01;
    
    float celcius = tempData - 273.15;
    Calor = celcius;
    return Calor;
}

void Blink()
{
  for(int iI = 0; iI < 6; iI++)
  {
    lcd.noBacklight();
    delay(500)
    lcd.backlight();
    delay(500);
  }
}

void EncDerE()
{
  return EncDerE.read();
}

void EncDerA()
{
  return EncDerA.read();
}

void EncIzqE()
{
  return EncIzqE.read();
}

void EncIzqA()
{
  return EncIzqA.read();
}

void AgujeroNegro()
{
  if(color() == true)
  {
    Negro();
  }
}

bool ParedDer()
{
  bool Pared = false
  int Ult1 = UltDA();
  int Ult2 =UltDB();
  if(Ult1 < 15 || Ult2 < 15)
  {
    Pared = true;
  }
  return Pared;
}

bool ParedEnf()
{
  bool Pared = false
  int Ult1 = UltEA();
  int Ult2 =UltEB();
  if(Ult1 < 15 || Ult2 < 15)
  {
    Pared = true;
  }
  return Pared;
}

void loop() {
  // put your main code here, to run repeatedly:
}
