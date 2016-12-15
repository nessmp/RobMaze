#include <OLED.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <SparkFunMLX90614.h> //libreria de los MLX
#include "CurieIMU.h"

//Algoritmo
//Max de arreglo de x
byte const PROGMEM XX = 40;
//Maximo de arreglo de y
byte const PROGMEM YY = 40;
//Maximo de arreglos de z
byte const PROGMEM ZZ = 3;
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
byte bZ = 1;
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
//Para La rampa solo cambie datos cuando se esta explorando nuevo terreno
bool Explore = false;

bool bInicio = false;
bool bVictimaDetectada = false;
bool bVictimaDetectadaDer = false;
bool bVictimaDetectadaIzq = false;
bool bVictimaDerecha = false;
bool bVictimaIzquierda = false;

Servo servo;

Adafruit_MotorShield MotDerEnf = Adafruit_MotorShield();
Adafruit_MotorShield MotDerAtr = Adafruit_MotorShield();

Adafruit_DCMotor *MDE = MotDerEnf.getMotor(1);
Adafruit_DCMotor *MDA = MotDerAtr.getMotor(4);

Adafruit_MotorShield MotIzqEnf = Adafruit_MotorShield();
Adafruit_MotorShield MotIzqAtr = Adafruit_MotorShield();

Adafruit_DCMotor *MIE = MotIzqEnf.getMotor(2);
Adafruit_DCMotor *MIA = MotIzqAtr.getMotor(3);

byte velMDE = 50;
byte velMDA = 110;

byte velMIE = 110;
byte velMIA = 48;
int Dif = 0;

const int const90 = 840;
const int const90Izq = 910;
const int const30 = 1300;

int encoderValue = 0;

//void count(void); // code for counting the increasing values of encoder ticks void setu

//SHARPS
const byte IE = A2;
const byte IA = A0;
const byte DE = A3;
const byte DA = A1;

//MLX
int PROGMEM I2C_Address_MLX1 = 0x5A;
IRTherm therm1; //MLX Izq
int PROGMEM I2C_Address_MLX2 = 0x1C;
IRTherm therm2; //MLX Der

byte CalibCalor = 30;
byte CalibCalor2 = 30;

//COLOR
const byte out = 11;

//ULTRASONICO
const char UltDer = '>';
const char UltIzq = '<';

//OLED
OLED oled;

void setup() {
  Serial.begin(115200);
  Serial.println("Zucaritas");

  CurieIMU.begin();
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

  //ULTRASONICO   //  Izq(tr(10)ech(9)) Der(tr(7)ech(8))
  pinMode(9, INPUT);
  pinMode(10, OUTPUT);
  pinMode(7, INPUT);
  pinMode(8, OUTPUT);

  //MLX
  therm1.begin(I2C_Address_MLX1); //Primer MLX
  therm1.setUnit(TEMP_C);
  therm2.begin(I2C_Address_MLX2); //Primer MLX
  therm2.setUnit(TEMP_C);

  servo.attach(4);

  MotDerEnf.begin();
  MotDerAtr.begin();
  MotIzqEnf.begin();
  MotIzqAtr.begin();

  MDE->run(RELEASE);
  MDA->run(RELEASE);
  MIE->run(RELEASE);
  MIA->run(RELEASE);

  pinMode(2, INPUT);//   atras(2,3)    adelante(5,6)

  attachInterrupt(2, count, RISING);

  pinMode(13, INPUT_PULLUP);

  attachInterrupt(13, victimaDerecha, LOW);

  pinMode(5, INPUT_PULLUP);

  attachInterrupt(5, victimaIzquierda, LOW);

  oled.Init();

  encoderValue = 0;
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
  Serial.println("Nesquik");
  oled.RoBorregos();
}

void count()
{
  encoderValue++;
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

void AdelanteRampa(byte DE, byte DA,  byte IE, byte IA)
{
  MDE->run(FORWARD);
  MDA->run(FORWARD);
  MIE->run(FORWARD);
  MIA->run(FORWARD);
  MDE->setSpeed(velMDE + ((40 + DE) * velMDE / 100));
  MDA->setSpeed(velMDA + ((40 + DA) * velMDA / 100));
  MIE->setSpeed(velMIE + ((40 + IE) * velMIE / 100));
  MIA->setSpeed(velMIA + ((40 + IA) * velMIA / 100));
}

void DerechaDiagonal()
{
  MDE->run(FORWARD);
  MDA->run(FORWARD);
  MIE->run(FORWARD);
  MIA->run(FORWARD);
  MDE->setSpeed(velMDE);
  MDA->setSpeed(255);
  MIE->setSpeed(255);
  MIA->setSpeed(velMIA);
}

void IzquierdaDiagonal()
{
  MDE->run(FORWARD);
  MDA->run(FORWARD);
  MIE->run(FORWARD);
  MIA->run(FORWARD);
  MDE->setSpeed(255);
  MDA->setSpeed(velMDA);
  MIE->setSpeed(velMIE);
  MIA->setSpeed(255);
}

//Funcion que detiene los motores
void Detenerse()
{
  //Serial.println("Detenerse");
  MDA->run(RELEASE);
  MIE->run(RELEASE);
  MDE->run(RELEASE);
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

void CSM()
{
  MDE->run(FORWARD);
  MDA->run(FORWARD);
  MIE->run(FORWARD);
  MIA->run(FORWARD);

  MDE->setSpeed(255);
  MDA->setSpeed(255);
  MIE->setSpeed(255);
  MIA->setSpeed(255);
}

void victimaDerecha()
{
  bVictimaDetectadaDer = true;
}

void victimaIzquierda()
{
  bVictimaDetectadaIzq = true;
}

void VictimaDer()
{
  Detenerse();
  oled.Kit();
  servo.write(65);
  delay(1000);
  oled.KitNegro();
  servo.write(160);
  delay(1000);
  oled.Kit();
  delay(1000);
  oled.KitNegro();
  delay(1000);
  oled.Kit();
  servo.write(65);
  delay(1000);
  oled.Mexico();
  //Adelante();
  bVictimaDetectada = true;
}

void VictimaIzq()
{
  Detenerse();
  oled.Kit();
  servo.write(75);
  delay(1000);
  oled.KitNegro();
  servo.write(4);
  delay(1000);
  oled.Kit();
  delay(1000);
  oled.KitNegro();
  delay(1000);
  servo.write(75);
  oled.Kit();
  delay(1000);
  oled.Mexico();
  //Adelante();

  bVictimaDetectada = true;
}

void Adelante30()
{
  byte counter = 0;
  int gxRaw, gyRaw, gzRaw;         // raw gyro values
  float gz;
  encoderValue = 0;
  Adelante();
  while (encoderValue < const30)
  {
    
    Serial.println(digitalRead(0));
    if (bVictimaDetectadaDer == true && counter == 0)
    {
      VictimaDer();
      Adelante();
      bVictimaDetectadaDer = false;
      counter++;
    }
    if (bVictimaIzquierda == true && counter == 0)
    {
      VictimaIzq();
      Adelante();
      bVictimaIzquierda = false;
      counter++;
    }
    CurieIMU.readGyro(gxRaw, gyRaw, gzRaw);
    if (encoderValue >= const30)
      break;
    if (encoderValue >= const30)
      break;
    gz = (gzRaw * 250.0) / 32768.0;
    if (encoderValue >= const30)
      break;
    if (gz > 30 && counter < 4)
    {
      Detenerse();
      byte u1 = Ult(UltIzq);
      delay(50);
      byte u2 = Ult(UltDer);
      delay(50);
      if (u1 < 15 || u2 < 15 || u1 > 100 || u2 > 100)
        break;
      int copEncoder = encoderValue;
      Atras();
      delay(200);
      //Twerk();
      IzquierdaM();
      delay(200);
      Adelante();
      delay(200);
      encoderValue = copEncoder;
      counter++;
    }
    else if (gz < -30 && counter < 4)
    {
      Detenerse();
      int copEncoder = encoderValue;
      Atras();
      delay(200);
      //Twerk();
      DerechaM();
      delay(200);
      Adelante();
      delay(200);
      encoderValue = copEncoder;
      counter++;
    }
  }
  Detenerse();
}

void DerechaM()
{
  MDE->run(BACKWARD);
  MDA->run(FORWARD);
  MIE->run(FORWARD);
  MIA->run(BACKWARD);

  MDE->setSpeed(velMDE + map(Dif, 0, 100, 0, velMDE));
  MDA->setSpeed(velMDA + map(Dif, 0, 100, 0, velMDA));
  MIE->setSpeed(velMIE + map(Dif, 0, 100, 0, velMIE));
  MIA->setSpeed(velMIA + map(Dif, 0, 100, 0, velMIA));
}
void IzquierdaM()
{
  MDE->run(FORWARD);
  MDA->run(BACKWARD);
  MIE->run(BACKWARD);
  MIA->run(FORWARD);

  MDE->setSpeed(velMDE + map(Dif, 0, 100, 0, velMDE));
  MDA->setSpeed(velMDA + map(Dif, 0, 100, 0, velMDA));
  MIE->setSpeed(velMIE + map(Dif, 0, 100, 0, velMIE));
  MIA->setSpeed(velMIA + map(Dif, 0, 100, 0, velMIA));
}
void IzquierdaMFrac()
{
  encoderValue = 0;
  // Dif = -10;
  velMIA += 25;
  velMDE += 25;

  IzquierdaM();

  while (encoderValue < 5)
  {
    Serial.println(encoderValue);

  }
  Detenerse();
  velMIA -= 25;
  velMDE -= 25;
  // Dif = 0;
}

void DerechaMFrac()
{
  encoderValue = 0;
  //Dif = -10;
  velMIA += 30;
  velMDE += 30;
  DerechaM();

  while (encoderValue < 5)
  {
    Serial.println(encoderValue);

  }
  Detenerse();
  velMIA -= 30;
  velMDE -= 30;
  // Dif = 0;
}
//GIROS
void Derecha()
{
  MDE->run(BACKWARD);
  MDA->run(BACKWARD);
  MIE->run(FORWARD);
  MIA->run(FORWARD);

  MDE->setSpeed(velMDE + (20 * velMDE / 100));
  MDA->setSpeed(velMDA + (20 * velMDA / 100));
  MIE->setSpeed(velMIE + (20 * velMIE / 100));
  MIA->setSpeed(velMIA + (20 * velMIA / 100));
}
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
void GiroDer90()
{
  byte counter = 0;
  encoderValue = 0;

  Derecha();
  while (encoderValue < const90)
  {
    if (bVictimaDetectadaDer == true && counter == 0)
    {
      VictimaDer();
      Derecha();
      bVictimaDetectadaDer = false;
      counter++;
    }
    else if (bVictimaIzquierda == true && counter == 0)
    {
      VictimaIzq();
      Derecha();
      bVictimaIzquierda = false;
      counter++;
    }
  }
  Detenerse();
  bVictimaDetectada = false;
}

void GiroIzq90()
{
  encoderValue = 0;

  Izquierda();
  while (encoderValue < const90Izq  )
  {
    if (bVictimaDetectadaDer == true)
    {
      VictimaDer();
      Izquierda();
      bVictimaDetectadaDer = false;
    }
    else if (bVictimaIzquierda == true)
    {
      VictimaIzq();
      Izquierda();
      bVictimaIzquierda = false;
    }
  }
  Detenerse();
  bVictimaDetectada = false;
}

void GiroDer90Paredes()
{
  bool D = ParedDer();
  bool I = ParedIzq();
  bool E = ParedEnf();

  encoderValue = 0;

  Derecha();
  while (encoderValue < const90  )
  {
    //Detectar();
    Serial.println(encoderValue);
  }
  Detenerse();
  delay(50);
  bool D2 = ParedDer();
  bool I2 = ParedIzq();
  bool E2 = ParedEnf();
  while (E2 != D && I2 != E && D2 != false)
  {
    encoderValue = 0;

    Derecha();
    while (encoderValue < const90 / 9)
    {
      //Detectar();
      Serial.println(encoderValue);
    }
    D2 = ParedDer();
    I2 = ParedIzq();
    E2 = ParedEnf();
  }
  Detenerse();
}

//Sharps
int cm30(byte irPin)
{
  int raw = analogRead(irPin);
  float voltFromRaw = map(raw, 0, 1023, 0, 3300); //Cambiar 5000 por 3300

  int puntualDistance;

  puntualDistance =  27.728  * pow(voltFromRaw / 1000, -1.2045);


  return puntualDistance;
}

void BubbleSort(int num[], int leng)
{
  int i, j, flag = 1;    // set flag to 1 to start first pass
  int temp;             // holding variable
  int numLength = leng;
  for (i = 1; (i <= numLength) && flag; i++)
  {
    flag = 0;
    for (j = 0; j < (numLength - 1); j++)
    {
      if (num[j + 1] > num[j])    // ascending order simply changes to <
      {
        temp = num[j];             // swap elements
        num[j] = num[j + 1];
        num[j + 1] = temp;
        flag = 1;               // indicates that a swap occurred.
      }
    }
  }
}

int Sharp(byte SharpPin)
{
  int dist[5];
  for (byte iI = 0; iI < 5; iI++)
  {
    int _p = 0;
    int _sum = 0;
    int _avg = 25;
    int _tol = 93 / 100;
    int _previousDistance = 0;


    for (int i = 0; i < _avg; i++)
    {
      int foo = cm30(SharpPin);

      if (foo >= (_tol * _previousDistance))
      {
        _previousDistance = foo;
        _sum = _sum + foo;
        _p++;
      }
    }

    int accurateDistance = _sum / _p;
    dist[iI] = accurateDistance;
  }
  BubbleSort(dist, 5);
  return dist[2];
  //return accurateDistance;
}

bool ParedDer()
{
  bool pared = true;
  int dist = Sharp(DE);
  int dist2 = Sharp(DA);
  if (dist > 39 || dist2 > 39)
  {
    pared = false;
  }
  return pared;
}

bool ParedIzq()
{
  bool pared = true;
  int dist = Sharp(IE);
  int dist2 = Sharp(IA);
  if (dist > 39 || dist2 > 39)
  {
    pared = false;
  }
  return pared;
}
//COLOR
bool Negro()
{
  bool negro = false;
  if (pulseIn(out, LOW) > 1000)
  {
    negro = true;
  }
  return negro;
}
//ULTRASONICOS
int Ult(char U)
{
  int Trigger;
  int Echo;
  long distancia;
  long tiempo;

  if (U == '>')
  {
    Trigger = 8;
    Echo = 7;
  }
  else if (U == '<')
  {
    Trigger = 10; //10
    Echo = 9; //9
  }
  int dist[5];
  for (byte iI = 0; iI < 5; iI++)
  {
    // put your main code here, to run repeatedly:
    digitalWrite(Trigger, LOW); /* Por cuestión de estabilización del sensor*/
    delayMicroseconds(5);
    digitalWrite(Trigger, HIGH); /* envío del pulso ultrasónico*/
    delayMicroseconds(10);
    tiempo = pulseIn(Echo, HIGH); /* Función para medir la longitud del pulso entrante. Mide el tiempo que transcurrido entre el envío
  del pulso ultrasónico y cuando el sensor recibe el rebote, es decir: desde que el pin 12 empieza a recibir el rebote, HIGH, hasta que
  deja de hacerlo, LOW, la longitud del pulso entrante*/
    distancia = int(0.017 * tiempo); /*fórmula para calcular la distancia obteniendo un valor entero*/
    /*Monitorización en centímetros por el monitor serial*/
    dist[iI] = distancia;
  }
  BubbleSort(dist, 5);
  return dist[2];
}

bool ParedEnf ()
{
  bool pared = true;
  int UD, UI;
  UD = Ult(UltDer);
  delay(5);
  UI = Ult(UltIzq);
  if ((UD > 20) || (UI > 20) )
  {
    pared = false;
  }
  return pared;

}

//ACOMODOS
void Acomodo()
{
  //delay(500);
  //delay(250);
  // oled.Yazaki();
  // delay(500);
  Acejarse();
  // oled.DefinityFirst();
  //delay(500);
  // Faltante();
  //delay(250);
  Twerk();
  //oled.JohnDeere();
  //delay(500);
  // AcejarseEnfrente();
  oled.Yazaki();
  Evadir();
  Evadir();
  Evadir();
  Evadir();
  Evadir();
  oled.JohnDeere();
}
void Acejarse()
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
void AcejarseDerecha()
{

  int Dist ;
  Dist =  Sharp(DE);


  while (Dist != 12 ) {
    if (Dist < 12)
    {

      while (Dist < 12)
      {
        IzquierdaMFrac();

        //delay(90);       //delay modificable

        Dist = Sharp(DE);


        //      delay(10);       //delay modificable

      }
    }
    else if (Dist > 12)
    {

      while (Dist > 12)
      {
        DerechaMFrac();
        //  delay(90);

        Dist = Sharp(DE);

        //delay(10);

      }
    }
    else if (Dist == 12) {
      break;
    }
    Detenerse();
  }


}
void AcejarseIzquierda()
{
  int DistE, DistA ;
  DistE =  Sharp(DE);
  delay(1);
  // DistA=Sharp(DA);

  ////Serial.println("entro 1 if");


  while (DistE != 10) {
    if (DistE > 10 )
    {

      while (DistE > 10)
      {
        IzquierdaMFrac();
        //   delay(90);

        DistE =  Sharp(IE);

        // delay(10);

      }
    }
    else if (DistE < 10)
    {

      while (DistE < 10)
      {
        DerechaMFrac();
        //  delay(90);

        DistE =  Sharp(IE);

        //delay(10);

      }
    }

    else if (DistE == 10)
      break;
    Detenerse();
  }
}

void Twerk()
{
  if (ParedIzq())
  {
    TwerkIzq();
  }
  else if (ParedDer())
  {
    TwerkDer();
  }
}

void TwerkIzq()
{
  int ShE, ShA;

  ShE = Sharp(IE);
  delay(5);
  ShA = Sharp(IA);
  delay(5);
  int const1 = const90 / 90;

  while ((ShA - ShE) != 0)
  {
    if ((ShA - ShE) < 0)
    {
      encoderValue = 0;
      //   Dif = -20;
      velMIA += 10;
      velMDE += 10;
      Izquierda();
      while (encoderValue < const1)
      {
        Serial.println(encoderValue);
      }
      Detenerse();
      velMIA -= 10;
      velMDE -= 10;
      //   Dif = 0;
      //  delay(90);
    }

    else if ((ShA - ShE) > 0)
    {
      encoderValue = 0;

      // Dif = -20;
      velMIA += 10;
      velMDE += 10;
      Derecha();
      while (encoderValue < const1)
      {
        Serial.println(encoderValue);
      }
      Detenerse();
      velMIA -= 10;
      velMDE -= 10;
      //  Dif = 0;
      // delay(90);
    }


    ShE = Sharp(IE);
    delay(1);
    ShA = Sharp(IA);
    delay(1);
  }
}

void TwerkDer()
{

  int ShE, ShA;

  ShE = Sharp(DE);
  delay(5);
  ShA = Sharp(DA);
  delay(5);
  int const1 = const90 / 90;

  while ((ShE - ShA) != 0)
  {
    if ((ShE - ShA) < 0)
    {
      encoderValue = 0;
      velMIA += 10;
      velMDE += 10;
      // Dif = -20;
      Izquierda();
      while (encoderValue > const1)
      {
        Serial.println(encoderValue);
      }
      Detenerse();
      velMIA -= 10;
      velMDE -= 10;
      // Dif = 0;
      //    delay(90);
    }
    else if ((ShE - ShA) > 0)
    {
      encoderValue = 0;

      // Dif = -20;
      velMIA += 10;
      velMDE += 10;
      Derecha();
      while (encoderValue < const1)
      {
        Serial.println(encoderValue);
      }
      velMIA -= 10;
      velMDE -= 10;
      Detenerse();
      //  Dif = 0;
      //    delay(90);
    }


    ShE = Sharp(DE);
    delay(1);
    ShA = Sharp(DA);
    delay(1);
  }
}
void Faltante()
{
  /* if (ParedIzq())
    {
     FaltanteIzq();
    }
    if (ParedDer())
    {
     FaltanteDer();
    }*/
}

void FaltanteDer()
{
  int ShE, ShA;
  int Max = 30; // Aun no se le da valor!!!!
  int const1 = const30 / 30;
  ShE = Sharp(DE);
  delay(5);
  ShA = Sharp(DA);
  delay(5);

  if (ShE < Max && ShA > Max)
  {
    while ( ShA > Max)//mientras no esté dentro de la pared
    {
      encoderValue = 0;

      while (encoderValue < const1) //adelante de 1 en 1 cm
      {
        Adelante();
        Serial.println(encoderValue);
      }
      Detenerse();
      ShA = Sharp(DA);

    }
  }
}
void FaltanteIzq()
{
  int ShE, ShA;
  int Max = 30; // Aun no se le da valor!!!!
  int const1 = const30 / 30;

  ShE = Sharp(IE);
  delay(5);
  ShA = Sharp(IA);
  delay(5);

  if (ShE < Max && ShA > Max)
  {




    while ( ShA > Max)//mientras no esté dentro de la pared
    {
      encoderValue = 0;

      while (encoderValue < const1) //adelante de 1 en 1 cm
      {
        Adelante();
        Serial.println(encoderValue);
      }
      Detenerse();
      ShA = Sharp(IA);
    }
  }
}
void SeguirDerecha()
{
  bool Pd = ParedDer();
  bool Pi = ParedIzq();
  bool Pe = ParedEnf();
  if (Pd == false)
  {
    GiroDer90();
    delay(100);
    Acomodo();


  }
  else if (Pd == true && Pe == false)
  {
    Adelante30();
    delay(100);
    Acomodo();
  }
  else if (Pd == true && Pe == true)
  {
    GiroIzq90();
    delay(100);
    Acomodo();
  }
  else if (Pi == true)
  {
    GiroIzq90();
    delay(100);
    Acomodo();

  }

}

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
  Serial.println("----Datos----");
  bool bDir[4] = {false, false, false, false};
  Paso++;
  pasoActual = Paso;
  Serial.print("Paso: ");
  Serial.println(Paso);
  Pos[bZ][bX][bY] = Paso;
  Serial.print("bZ: ");
  Serial.println(bZ);
  Serial.print("bX: ");
  Serial.println(bX);
  Serial.print("bY: ");
  Serial.println(bY);
  Serial.print("Pos[bZ][bX][bY]: ");
  Serial.println(Pos[bZ][bX][bY]);
  Possibility[Paso] = getPossibility(bDir);
  Serial.print("Possibility: ");
  Serial.println(Possibility[Paso]);
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
  //Serial.println("----Datos----");
  //Serial.print("bZ: ");
  //Serial.println(bZ);
  //Serial.print("bX: ");
  //Serial.println(bX);
  //Serial.print("bY: ");
  //Serial.println(bY);
  //Serial.print("Pos: ");
  //Serial.println(Pos[bZ][bX][bY]);
  //Serial.print("Possibility: ");
  //Serial.println(Possibility[Paso]);
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
  delay(200);
  bool bReturn = false;
  int iMPUP = MPUP();
  if (iMPUP > 2000 || iMPUP < -3000)
  {
    bReturn = true;
    oled.Yazaki();
  }
  return bReturn;
}

//Cruza la rampa, ACTUALIZAR bZ aqui!!
void MovRampa()
{
  Serial.println("----Rampa----");
  Serial.print("brRampa, paso: ");
  Serial.println(brRampa);
  Serial.print("biI, run: ");
  Serial.println(biI);
  Serial.println("Run[brRampa][biI]");
  Serial.print("De: ");
  Serial.println(Run[brRampa][biI]);
  //Serial.print("bZ: ");
  //Serial.println(bZ);
  int iMPUP = MPUP();
  //Si detecta que hay que subir la rampa
  if (iMPUP > 4000)
  {
    if (true == Explore)
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
    }
    //AcomodoRampa();
    RampaS();
    Detenerse();
    Acomodo();
    bZ += 1;
  }
  //Si esta bajando la rampa
  else if (iMPUP < -2000)
  {
    if (true == Explore)
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
    }
    AcomodoRampa();
    RampaB();
    Detenerse();
    bZ -= 1;
  }
  Serial.print("a: ");
  Serial.println(Run[brRampa][biI]);
}

//Se mueve de la coordenada actuar(iCoordAc) a la coordenada deseada(icCoord)
void Move(int iCoordAc, int icCoord)
{
  Serial.println("---Move---");
  Serial.print("Coordenada Actual: ");
  Serial.println(iCoordAc);
  Serial.print("Coordenada deseada: ");
  Serial.println(icCoord);
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
  Detenerse();
  //delay(100);
  //si esa nueva coordenada es un hoyo negro regresa y busca a donde moverse
  if (HoyoNegro())
  {
    listaX[subNegro] = CopcCoord;
    subNegro += 1;
    //Serial.println("----Entra HoyoNegro----");
    bool bListo = false;
    Atras30();
    Acomodo();
    bX = bCX;
    bY = bCY;
    byte iCPaso = Pos[bZ][bX][bY];
    //Serial.print("bZ: ");
    //Serial.println(bZ);
    //Serial.print("bX: ");
    //Serial.println(bX);
    //Serial.print("bY: ");
    //Serial.println(bY);
    //Serial.print("Pos[bZ][bX][bY]= ");
    //Serial.println(Pos[bZ][bX][bY]);
    for (byte iI = 0; iI < Possibility[iCPaso]; iI++)
    {
      //Serial.print("icCoord: ");
      //Serial.println(icCoord);
      int iThis = getCoord(Run[iCPaso][iI], iCPaso);
      //Serial.print("iThis #");
      //Serial.print(iI);
      //Serial.print(": ");
      //Serial.println(getCoord(Run[iCPaso][iI], iCPaso));
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
    //Serial.print("iCPaso: ");
    //Serial.println(iCPaso);
    //Serial.print("Run 1: ");
    //Serial.println(Run[iCPaso][0]);
    //Serial.print("Run 2: ");
    //Serial.println(Run[iCPaso][1]);
    //Serial.print("Run 3: ");
    //Serial.println(Run[iCPaso][2]);
    //Serial.print("Run 4: ");
    //Serial.println(Run[iCPaso][3]);
    //Serial.print("Possbility: ");
    //Serial.println(Possibility[iCPaso]);
    SearchRouteAndMove();
  }
  //Si esa nueva coordenada es la rampa, la cruza y actualiza bZ
  else if (Rampa())
  {
    MovRampa();
    bARampa = true;
  }
  Acomodo();
  //lcd.clear();
  //lcd.setCursor(0, 0);
  //lcd.print(bZ);
  //lcd.print(" ");
  //lcd.print(bX);
  //lcd.print(" ");
  //lcd.print(bY);
}


//Se mueve hasta el paso que se tiene como parametro
void moveUntil(byte bHere)
{
  Serial.println("---MoveUntil---");
  Serial.print("Ir hasta el paso #");
  Serial.println(bHere);
  byte counter = 0;
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
    counter++;
    if (counter > 0)
    {
      bARampa = false;
    }
  }
}

//Regresa el punto de inicio
void extractionPoint()
{
  moveUntil(1);
  Detenerse();
  //lcd.clear();
  //lcd.write("Fin");
  oled.Meta();
  delay(30000);
}

//Se mueve a la coordenada desconocida
void exploreNewWorlds(byte bHere)
{
  Explore = true;
  Serial.println("----exploreNewWorlds----");
  Serial.print("Se encuentra en Paso: ");
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
      //lcd.setCursor(7, 1);
      //lcd.print("c ");
      //lcd.print(getCoord(Run[bHere][iI], bHere));
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
      //Serial.print("Coordenada Actual: ");
      //Serial.println(iCoordAc);
      //Serial.print("Coordenada Deseada: ");
      //Serial.println(icCoord);
      Move(iCoordAc, icCoord);
      iCounter++;
    }
  }
  Explore = false;
}

//Busca el paso al cual llegar para despues moverse a la coordenada desconocida
byte WhereToGo()
{
  Serial.println("---WhereToGo---");
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
          Serial.print("No ha estado en un Run del paso #");
          Serial.println(iI);
          Serial.print("Numero de Run: ");
          Serial.println(iJ);
          Serial.print("Coordenada: ");
          Serial.println(getCoord(Run[iI][iJ], iI));
        }
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
  Serial.println("---SearchAndMove---");
  byte bData = WhereToGo();
  //lcd.setCursor(0, 1);
  //lcd.print("ir #");
  //lcd.print(bData);
  moveUntil(bData);
  exploreNewWorlds(bData);
}

//Funcion a llamar para completar el laberinto
void Laberinto()
{
  Serial.println("------Laberinto-------");
  GetDatos();
  SearchRouteAndMove();
}

bool HoyoNegro()
{
  bool bNegro = false;
  int color = pulseIn(out, LOW);
  if (color > 1500)
  {
    bNegro = true;
  }
  return bNegro;
}

void Atras30()
{
  encoderValue = 0;
  Atras();
  while (encoderValue < const30)
  {
    Serial.println(encoderValue);
  }
  Detenerse();
}

//Sube la rampa detectando si hay victima en ella
//void RampaS() {} //Usar MPU, MLX

//Baja la rampa detectando si hay victima en ella
void RampaB()
{
  int MPU;
  Adelante();
  delay(200);
  Detenerse();
  unsigned long Time3;
  unsigned long Time2;
  unsigned long Time = millis();
  do {
    MPU = MPUP();
    do {
      Adelante();
      MPU = MPUP();
    } while (MPU < -500);
    Time2 = millis();
    Time3 = Time2 - Time;
    if (Time3 < 800)
    {
      MPU = -1000;
    }
    else
    {
      Detenerse();
      delay(300);
      MPU = MPUP();
    }
  } while (MPU < -500);
  Detenerse();
  Adelante();
  delay(500);
  Detenerse();
  oled.DefinityFirst();
} //Usar MPU, MLX

//Acomodoarse en la rampa para bajarla
void AcomodoRampa()
{
  Twerk();
}

int MPUP()
{
  int ax, ay, az;         // accelerometer values
  int arr[100];
  for (byte iI = 0; iI < 100; iI++)
  {
    CurieIMU.readAccelerometer(ax, ay, az);
    arr[iI] = ay;
  }
  BubbleSort(arr, 100);
  Serial.println(arr[50]);
  return arr[50];
}

void RampaS()
{
  int MPU;
  byte DE = 15;
  byte DA = 15;
  byte IE = 15;
  byte IA = 15;
  do
  {
    CSM();
    //AdelanteRampa(DE, DA, IE, IA);
    do {
      /*
        if (ParedIzq())
        {
        byte Enf = Sharp(IE);
        byte Atr = Sharp(IA);
        Serial.print("ENF: ");
        Serial.print(Enf);
        Serial.print("\t");
        Serial.print("ATR: ");
        Serial.println(Atr);
        if (Enf < 13 || Atr < 13)
        {
          DA = 40;
          IE = 40;
        }
        else if (Enf > 13 || Atr > 13)
        {
          DE = 40;
          IA = 40;
        }
        else
        {
          IE = 15;
          IA = 15;
          DE = 15;
          DA = 15;
        }
        }
        AdelanteRampa(DE, DA, IE, IA);
      */
      MPU = MPUP();
    } while (MPU > 1000);
    Detenerse();
    delay(500);
    MPU = MPUP();
  } while (MPU > 2000);
  Detenerse();
  Adelante();
  delay(200);
  Detenerse();
  Adelante();
  delay(500);
  Detenerse();
  oled.DefinityFirst();
}

void Evadir()
{
  byte u1 = Ult(UltIzq);
  if (u1 > 25)
    u1 = 0;
  delay(10);
  byte u2 = Ult(UltDer);
  if (u2 > 25)
    u2 = 0;
  delay(10);
  Serial.print(u1);
  Serial.print("\t");
  Serial.println(u2);
  if (u2 != 0 && u1 == 0)
  {
    while (u2 != 0 && u1 == 0)
    {

      IzquierdaMFrac();
      u1 = Ult(UltIzq);
      if (u1 > 25)
        u1 = 0;
      delay(10);
      u2 = Ult(UltDer);
      if (u2 > 25)
        u2 = 0;

    }

  }
  else if (u1 != 0 && u2 == 0)
  {
    while (u1 != 0 && u2 == 0)
    {
      DerechaMFrac();
      u1 = Ult(UltIzq);
      if (u1 > 25)
        u1 = 0;
      delay(10);
      u2 = Ult(UltDer);
      if (u2 > 25)
        u2 = 0;
    }
  }
}

void AcejarseEnfrente()
{
  byte u1 = Ult(UltIzq);
  if (u1 > 25)
    u1 = 0;
  delay(1);
  byte u2 = Ult(UltDer);
  if (u2 > 25)
    u2 = 0;
  delay(1);
  if (u1 > 15 || u2 > 15)
  {
    Adelante();
    delay(10);
  }
  Detenerse();
}

void Detectar()
{
  /*
    if (bVictimaDetectada == false)
    {
    Serial.print("CalibCalor: ");
    Serial.print("\t");
    Serial.print(CalibCalor);
    Serial.print("\t");
    Serial.println(CalibCalor2);
    servo.write(70);
    int calor1 = 0;
    int calor2 = 0;
    therm1.read();
    calor1 = therm1.object();
    therm2.read();
    calor2 = therm2.object();
    Serial.print(calor1);
    Serial.print("\t");
    Serial.println(calor2);
    if (calor1 > CalibCalor && calor1 < 100)
    {
      Detenerse();
      bVictimaDetectada = true;
      Serial.println("1 IF");
      servo.write(75);
      oled.Kit();
      delay(1000);
      oled.KitNegro();
      servo.write(4);
      delay(1000);
      oled.Kit();
      delay(1000);
      oled.KitNegro();
      delay(1000);
      oled.Kit();
      delay(1000);
      oled.Mexico();
      Adelante();
    }
    if (calor2 > CalibCalor2 && calor2 < 100)
    {
      Detenerse();
      bVictimaDetectada = true;
      Serial.println("2 IF");
      servo.write(65);
      oled.Kit();
      delay(1000);
      oled.KitNegro();
      servo.write(145);
      delay(1000);
      oled.Kit();
      delay(1000);
      oled.KitNegro();
      delay(1000);
      oled.Kit();
      delay(1000);
      oled.Mexico();
      Adelante();
    }
    }
  */
} //Usar MLX

void DiagonalDerecha2()
{
  MDE->run(FORWARD);
  MDA->run(FORWARD);
  MIE->run(FORWARD);
  MIA->run(FORWARD);

  MDE->setSpeed(velMDE);
  MDA->setSpeed(velMDA + (velMDA * 60 / 100));
  MIE->setSpeed(velMIE + (velMIE * 60 / 100));
  MIA->setSpeed(velMIA);
}

void DiagonalIzquierda2()
{
  MDE->run(FORWARD);
  MDA->run(FORWARD);
  MIE->run(FORWARD);
  MIA->run(FORWARD);

  MDE->setSpeed(velMDE  + (velMDE * 60 / 100));
  MDA->setSpeed(velMDA);
  MIE->setSpeed(velMIE);
  MIA->setSpeed(velMIA + (velMIA * 60 / 100));
}

void CalorInicio()
{
  byte calor1 = 0;
  byte calor2 = 0;
  therm1.read();
  calor1 = therm1.object();
  therm2.read();
  calor2 = therm2.object();
  CalibCalor = calor1 + 2;
  CalibCalor2 = calor2 + 2;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
}

void loop() {
  Laberinto();
  /*
  Serial.print(digitalRead(5));
  Serial.print("\t");
  Serial.println(digitalRead(13));
  /*
  Adelante30();
  delay(500);
  /*
  servo.write(4);
  delay(1000);
  servo.write(70);
  delay(1000);
  servo.write(160);
  delay(1000);
  servo.write(70);
  delay(1000);
  */
}
//Laberinto();
//Detecte victima, dispense kit y marque en la oled en mov, giros tmb
//Hoyos Negros, que los reconozca y los mapee
//Detecte Rampa, RampaS y RampaB}
