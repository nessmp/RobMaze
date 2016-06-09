#include <Ultrasonico.h>

Ultrasonico ult(7, 8); //Crea el objeto (pin Trigger, pin Echo)

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println(ult.distance());
}
