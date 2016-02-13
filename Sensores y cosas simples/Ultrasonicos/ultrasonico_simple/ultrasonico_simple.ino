#include <NewPing.h>
#define MAX_DISTANCE 200
//ultrasonicos de enfrente
byte bTriggerE1 = 13;
byte bEchoE1 = 10;

NewPing sonarE1(bTriggerE1, bEchoE1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE1.ping_cm();

byte bTriggerE2 = 0;
byte bEchoE2 = 1;

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
  
}
