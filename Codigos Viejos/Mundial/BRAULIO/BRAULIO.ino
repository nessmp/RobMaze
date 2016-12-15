#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <Wire.h> //Libreria para I2C
#include <SparkFunMLX90614.h> //libreria de los MLX

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

const int const90 = 935;
const int const30 = 1300;

int encoderValue = 0;

//void count(void); // code for counting the increasing values of encoder ticks void setup()

//SHARPS
const byte IE = A2;
const byte IA = A0;
const byte DE = A3;
const byte DA = A1;

//MLX
int PROGMEM I2C_Address_MLX1 = 0x5A;
IRTherm therm1; //Primer MLX
int PROGMEM I2C_Address_MLX2 = 0x4C;
IRTherm therm2; //Segundo MLX
//COLOR
const byte out = 11;

//ULTRASONICO
const char UltDer = '>';
const char UltIzq = '<';

void setup() {
  Serial.begin(9600);
  Serial.println("Zucaritas");

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

  encoderValue = 0;
  Serial.println("Nesquik");
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

//Funcion que detiene los motores
void Detenerse()
{
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


void Adelante30()
{
  encoderValue = 0;
  Adelante();
  while (encoderValue < const30)
  {
    Serial.println(encoderValue);
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
  IzquierdaM();

  while (encoderValue < 10)
  {
    Serial.println(encoderValue);

  }
  Detenerse();
  // Dif = 0;
}

void DerechaMFrac()
{
  encoderValue = 0;
  //Dif = -10;
  DerechaM();

  while (encoderValue < 10)
  {
    Serial.println(encoderValue);

  }
  Detenerse();
  // Dif = 0;
}
//GIROS
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
  encoderValue = 0;

  Derecha();
  while (encoderValue < const90  )
  {
    Serial.println(encoderValue);
  }

  Detenerse();
}

void GiroIzq90()
{
  encoderValue = 0;

  Izquierda();
  while (encoderValue < const90  )
  {
    Serial.println(encoderValue);
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
  bool pared = false;
  int dist = Sharp(DE);
  if (dist < 40)
  {
    pared = true;
  }
  return pared;
}

bool ParedIzq()
{
  bool pared = false;
  int dist = Sharp(IE);
  if (dist < 40)
  {
    pared = true;
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
  bool pared = false;
  int UD, UI;
  UD = Ult(UltDer);
  delay(5);
  UI = Ult(UltIzq);
  if ((UD < 20) && (UI < 20) )
  {
    pared = true;
  }
  return pared;

}

//ACOMODOS
void Acomodo()
{
  Acejarse();
  delay(250);
  Faltante();
  delay(250);
  Twerk();
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
    else if (Dist == 12)
      break;
    Detenerse();
  }
}

void AcejarseIzquierda()
{
  int Dist2;

  ////Serial.println("entro 1 if");
  Dist2 = Sharp(IE);

  while (Dist2 != 12) {
    if (Dist2 < 12)
    {

      while (Dist2 < 12)
      {
        DerechaMFrac();
        //  delay(90);

        Dist2 =  Sharp(IE);

        //delay(10);

      }
    }
    else if (Dist2 > 12)
    {

      while (Dist2 > 12)
      {
        IzquierdaMFrac();
        //   delay(90);

        Dist2 =  Sharp(IE);

        // delay(10);

      }
    }
    else if (Dist2 == 12)
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

  while ((ShA - ShE) != 0)
  {
    if ((ShA - ShE) < 0)
    {
      encoderValue = 0;
      Dif = -20;
      Izquierda();
      while (encoderValue > ((const90 / 90) ))
      {
        Serial.println(encoderValue);
      }
      Detenerse();
      Dif = 0;
      //  delay(90);
    }

    else if ((ShA - ShE) > 0)
    {
      encoderValue = 0;

      Dif = -20;
      Derecha();
      while (encoderValue > (const90 / 90))
      {
        Serial.println(encoderValue);
      }
      Detenerse();
      Dif = 0;
      // delay(90);
    }


    ShE = Sharp(IE);
    delay(5);
    ShA = Sharp(IA);
    delay(5);
  }
}

void TwerkDer()
{

  int ShE, ShA;

  ShE = Sharp(DE);
  delay(5);
  ShA = Sharp(DA);
  delay(5);

  while ((ShE - ShA) != 0)
  {
    if ((ShE - ShA) < 0)
    {
      encoderValue = 0;

      Dif = -20;
      Izquierda();
      while (encoderValue > ((const90 / 90) ))
      {
        Serial.println(encoderValue);
      }
      Detenerse();
      Dif = 0;
      //    delay(90);
    }
    else if ((ShE - ShA) > 0)
    {
      encoderValue = 0;

      Dif = -20;
      Derecha();
      while (encoderValue < (const90 / 90))
      {
        Serial.println(encoderValue);
      }
      Detenerse();
      Dif = 0;
      //    delay(90);
    }


    ShE = Sharp(DE);
    delay(5);
    ShA = Sharp(DA);
    delay(5);
  }
}
void Faltante()
{
  if (ParedIzq())
  {
    FaltanteIzq();
  }
  if (ParedDer())
  {
    FaltanteDer();
  }
}

void FaltanteDer()
{
  int ShE, ShA;
  int Max = 30; // Aun no se le da valor!!!!
  int const1 = const30 / 6;
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
  int const1 = const30 / 6;

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
void loop() {
  SeguirDerecha();
  //delay(50);
  /*Serial.print(Sharp(A0));
    Serial.print("\t");
    Serial.print(Sharp(A1));
    Serial.print("\t");
    Serial.print(Sharp(A2));
    Serial.print("\t");
    Serial.println(Sharp(A3));*/
  /*Serial.print(Ult('<'));
    Serial.print("   ");
    delay(1);
    Serial.print(Ult('>'));
    Serial.print("   ");
    Serial.println(ParedEnf());
    delay(250);*/

  //GiroDer90();
  //Adelante30();
  //Acomodo();
  //delay(2000);
  // SeguirDerecha();
  //delay(1000);
  //Adelante();
}
