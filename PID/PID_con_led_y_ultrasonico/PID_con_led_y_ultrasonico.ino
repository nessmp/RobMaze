#include <NewPing.h>
#include <PID_v1.h>

#define MAX_DISTANCE 200

#define PIN_OUTPUT 11

//ultrasonicos de enfrente
byte bTriggerE1 = 8;
byte bEchoE1 = 9;

NewPing sonarE1(bTriggerE1, bEchoE1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE1.ping_cm();

double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Input = sonarE1.ping_cm();
  Setpoint = 10;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Ultrasonico: ");
  Serial.println(sonarE1.ping_cm());
  Input = sonarE1.ping_cm();
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
  Serial.println(Output);
}
