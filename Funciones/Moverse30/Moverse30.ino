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

/////////
//Sharp//
/////////

SharpIR sharpEnf(A0, 25, 93, model);
SharpIR sharpDer(A1, 25, 93, model);
SharpIR sharpIzq(A2, 25, 93, model);

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
  pinMode (A1, INPUT);
  pinMode (A2, INPUT);
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

void Der30()
{
  long EncDE = EncDerEnf.read();
  long EncDE30 = EncDE + 30;

  long EncDA = EncDerAtr.read();
  long EncDA30 = EncDA + 30;

  long EncIE = EncIzqEnf.read();
  long EncIE30 = EncIE + 30;

  int EncIA = EncIzqAtr.read();
  long EncIA30 = EncIA + 30;
  
  long Ultrasonico1 = sonarD1.ping_cm();
  long Ultrasonico130 = Ultrasonico1 - 30;

  long Ultrasonico2 = sonarD2.ping_cm();
  long Ultrasonico230 = Ultrasonico2 - 30;

  long Sharp = sharpDer.distance();
  int Sharp30 = Sharp - 30;

  do{
    Adelante();
    
    EncDE = EncDerEnf.read();
    EncDA = EncDerAtr.read();
    EncIE = EncIzqEnf.read();
    EncIA = EncIzqAtr.read();
    
    Ultrasonico1 = sonarD1.ping_cm();
    Ultrasonico2 = sonarD2.ping_cm();
    
    Sharp = sharpDer.distance();
  }while(EncDE <= EncDE30 && EncDA <= EncDA30 && EncIE <= EncIE30 && EncIA <= EncIA30 && Ultrasonico1 >= Ultrasonico130 && Ultrasonico2 >= Ultrasonico230 && Sharp >= Sharp30);

  Detenerse();
}

void Enf30()
{
  long EncDE = EncDerEnf.read();
  long EncDE30 = EncDE + 30;

  long EncDA = EncDerAtr.read();
  long EncDA30 = EncDA + 30;

  long EncIE = EncIzqEnf.read();
  long EncIE30 = EncIE + 30;

  int EncIA = EncIzqAtr.read();
  long EncIA30 = EncIA + 30;
  
  long Ultrasonico1 = sonarE1.ping_cm();
  long Ultrasonico130 = Ultrasonico1 - 30;

  long Ultrasonico2 = sonarE2.ping_cm();
  long Ultrasonico230 = Ultrasonico2 - 30;

  long Sharp = sharpEnf.distance();
  int Sharp30 = Sharp - 30;

  bool Ramps = false;

  do{
    Adelante();
    
    EncDE = EncDerEnf.read();
    EncDA = EncDerAtr.read();
    EncIE = EncIzqEnf.read();
    EncIA = EncIzqAtr.read();
    
    Ultrasonico1 = sonarE1.ping_cm();
    Ultrasonico2 = sonarE2.ping_cm();
    
    Sharp = sharpEnf.distance();

    Rampa = acelerometro();

    if(Rampa == true)
    {
      do{
        Adelante(); 
        Rampa = acelerometro();
      }while(Rampa == true);
      
    }
  }while(EncDE <= EncDE30 && EncDA <= EncDA30 && EncIE <= EncIE30 && EncIA <= EncIA30 && Ultrasonico1 >= Ultrasonico130 && Ultrasonico2 >= Ultrasonico230 && Sharp >= Sharp30);

  Detenerse();
}

void Izq30()
{
  long EncDE = EncDerEnf.read();
  long EncDE30 = EncDE + 30;

  long EncDA = EncDerAtr.read();
  long EncDA30 = EncDA + 30;

  long EncIE = EncIzqEnf.read();
  long EncIE30 = EncIE + 30;

  int EncIA = EncIzqAtr.read();
  long EncIA30 = EncIA + 30;
  
  long Ultrasonico1 = sonarI1.ping_cm();
  long Ultrasonico130 = Ultrasonico1 - 30;

  long Ultrasonico2 = sonarI2.ping_cm();
  long Ultrasonico230 = Ultrasonico2 - 30;

  long Sharp = sharpIzq.distance();
  int Sharp30 = Sharp - 30;

  do{
    Izquierda();
    
    EncDE = EncDerEnf.read();
    EncDA = EncDerAtr.read();
    EncIE = EncIzqEnf.read();
    EncIA = EncIzqAtr.read();
    
    Ultrasonico1 = sonarI1.ping_cm();
    Ultrasonico2 = sonarI2.ping_cm();
    
    Sharp = sharpIzq.distance();
  }while(EncDE <= EncDE30 && EncDA <= EncDA30 && EncIE <= EncIE30 && EncIA <= EncIA30 && Ultrasonico1 >= Ultrasonico130 && Ultrasonico2 >= Ultrasonico230 && Sharp >= Sharp30);

  Detenerse();
}

void Atr30()
{
  long EncDE = EncDerEnf.read();
  long EncDE30 = EncDE + 30;

  long EncDA = EncDerAtr.read();
  long EncDA30 = EncDA + 30;

  long EncIE = EncIzqEnf.read();
  long EncIE30 = EncIE + 30;

  int EncIA = EncIzqAtr.read();
  long EncIA30 = EncIA + 30;
  
  long Ultrasonico1 = sonarA1.ping_cm();
  long Ultrasonico130 = Ultrasonico1 - 30;

  long Ultrasonico2 = sonarA2.ping_cm();
  long Ultrasonico230 = Ultrasonico2 - 30;

  bool Rampa = false;

  do{
    Atras();
    
    EncDE = EncDerEnf.read();
    EncDA = EncDerAtr.read();
    EncIE = EncIzqEnf.read();
    EncIA = EncIzqAtr.read();
    
    Ultrasonico1 = sonarA1.ping_cm();
    Ultrasonico2 = sonarA2.ping_cm();

    Rampa = acelerometro();

    if(Rampa == true)
    {
      do{
        Atras(); 
        Rampa = acelerometro();
      }while(Rampa == true);
      
    }
    
  }while(EncDE <= EncDE30 && EncDA <= EncDA30 && EncIE <= EncIE30 && EncIA <= EncIA30 && Ultrasonico1 >= Ultrasonico130 && Ultrasonico2 >= Ultrasonico230);

  Detenerse();
}

void loop() {
  // put your main code here, to run repeatedly:

}
