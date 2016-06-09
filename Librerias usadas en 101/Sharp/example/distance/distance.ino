#include <Sharp.h>

Sharp sharp(0, 30); //Crea el objeto, (pin, distancia maxima del sensor)

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println(sharp.distance());
}
