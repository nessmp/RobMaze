#include <NewPing.h>
#define MAX_DISTANCE 200

#include <Encoder.h>

#include <SharpIR.h>

#define model 20150

byte bTriggerE1 = 8;
byte bEchoE1 = 9;

NewPing sonarE1(bTriggerE1, bEchoE1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE1.ping_cm();

byte bTriggerE2 = 10;
byte bEchoE2 = 11;

NewPing sonarE2(bTriggerE2, bEchoE2, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE1.ping_cm();

byte bTriggerD1 = 8;
byte bEchoD1 = 9;

NewPing sonarD1(bTriggerD1, bEchoD1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE1.ping_cm();

byte bTriggerD2 = 10;
byte bEchoD2 = 11;

NewPing sonarD2(bTriggerD2, bEchoD2, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE1.ping_cm();

byte bTriggerA1 = 8;
byte bEchoA1 = 9;

NewPing sonarA1(bTriggerA1, bEchoA1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE1.ping_cm();

byte bTriggerA2 = 10;
byte bEchoA2 = 11;

NewPing sonarA2(bTriggerA2, bEchoA2, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE1.ping_cm();

byte bTriggerI1 = 8;
byte bEchoI1 = 9;

NewPing sonarI1(bTriggerI1, bEchoI1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE1.ping_cm();

byte bTriggerI2 = 10;
byte bEchoI2 = 11;

NewPing sonarI2(bTriggerI2, bEchoI2, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE1.ping_cm();

byte motDerE1 = 19;
byte motDerE2 = 20;

byte motDerA1 = 21;
byte motDerA2 = 22;

byte motIzqE1 = 23;
byte motIzqE2 = 24;

byte motIzqA1 = 25;
byte motIzqA2 = 26;

////////////
//Encoders//
////////////

Encoder EncDerEnf(2, 4);
Encoder EncDerAtr(5, 6);
Encoder EncIzqEnf(7, 8);
Encoder EncIzqAtr(9, 10);
long oldPosition  = -999;

//////////
//Sharps//
//////////

int ir_sensor0 = A0;
int ir_sensor1 = A1;
int ir_sensor2 = A2;

/////////
//Sharp//
/////////

SharpIR sharp(A0, 25, 93, model);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  ////////////////////////
  //Setup de los motores//
  ////////////////////////
  
  pinMode(motDerE1, OUTPUT);
  pinMode(motDerE2, OUTPUT);
  
  pinMode(motDerA1, OUTPUT);
  pinMode(motDerA2, OUTPUT);

  pinMode(motIzqE1, OUTPUT);
  pinMode(motIzqE2, OUTPUT);

  pinMode(motIzqA1, OUTPUT);
  pinMode(motIzqA2, OUTPUT);
  
  /////////
  //Sharp//
  /////////

  pinMode (A0, INPUT);
}

///////////////////////////
//funciones de movimiento//
///////////////////////////

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

  analogWrite(motDerA1, 255);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 255);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 255);
  analogWrite(motIzqA2, 0);

}

//funcion para moverse hacia izquierda
void Izquierda()
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

int Encoders()
{
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    //Serial.println(newPosition);
  }
  return newPosition;
}

int Sharps()
{
  int lectura, cm;
 
  lectura = analogRead(ir_sensor0); // lectura del sensor 0
  cm = pow(3027.4 / lectura, 1.2134); // conversión a centímetros
  //Serial.print("Sensor 0: ");
  //Serial.println(cm); // lectura del sensor 0
  return cm;
}

void Enf30()
{
  long Enc = EncDerEnf;
  long Enc30 = Enc + 30;
  
  long Ultrasonico = sonarE1.ping_cm();
  long Ultrasonico30 = ultrasonico - 30;

  long Sharp = sharp.distance();
  long Sharp30 = Sharp - 30;

  do{
    Adelante();
    
    Enc = EncDerEnf;
    Ultrasonico = sonarE1.ping_cm();
    Sharp = sharp.distance();
  }while(Enc < Enc 30 && Ultrasonico > Ultrasonico 30 && Sharp > Sharp30)

  Detenerse();
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
