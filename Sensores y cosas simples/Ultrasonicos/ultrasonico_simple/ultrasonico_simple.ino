#include <NewPing.h>
#define MAX_DISTANCE 200
//ultrasonicos de enfrente
byte bTriggerE1 = 9;
byte bEchoE1 = 8;

NewPing sonarE1(bTriggerE1, bEchoE1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE1.ping_cm();

byte bTriggerE2 = 10;
byte bEchoE2 = 11;

NewPing sonarE2(bTriggerE2, bEchoE2, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE1.ping_cm();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  delay(250);
  // put your main code here, to run repeatedly:
  Serial.print("Ultrasonico Derecha: ");
  Serial.println(sonarE1.ping_cm());
  delay(250);
  Serial.print("Ultrasonico Izquierda: ");
  Serial.println(sonarE2.ping_cm());
  
}
