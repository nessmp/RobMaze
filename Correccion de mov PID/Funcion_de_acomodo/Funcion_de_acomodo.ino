#include <NewPing.h>
#include <PID_v1.h>

#define MAX_DISTANCE 200

#define PIN_OUTPUT 11

//ultrasonicos de enfrente
byte bTriggerE1 = 8;
byte bEchoE1 = 9;

NewPing sonarE1(bTriggerE1, bEchoE1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE1.ping_cm();


//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
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

  Input = sonarE1.ping_cm();
  Setpoint = 10;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);


}

void AcomodarseLejos()
{
  Setpoint = 10
  Input = sonarE1.ping_cm();
  myPID.Compute();
  //Movimiento hacia la izquierda
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, Output);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, Output);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, Output);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, Output);
}

void AcomodarseCerca()
{
  Setpoint = 10
  Input = sonarE1.ping_cm();
  myPID.Compute();

  //Movimiento hacia la derecha
  analogWrite(motDerE1, Output);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, Output);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, Output);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, Output);
  analogWrite(motIzqA2, 0);
}


void loop() {
  // put your main code here, to run repeatedly:

}
