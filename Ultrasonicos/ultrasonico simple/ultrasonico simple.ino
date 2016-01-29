#include <NewPing.h>
#define MAX_DISTANCE 200
//ultrasonicos de enfrente
byte bTriggerE1 = 6;
byte bEchoE1 = 7;

NewPing sonarE1(bTriggerE1, bEchoE1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE1.ping_cm();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Ultrasonico: ");
  Serial.println(sonarE1.ping_cm());
  delay(500);
}
