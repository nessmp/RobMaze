#include <Servo.h>

Servo Dispensador;  
const int pinservo = 3; 
const int PulsoMinimo = 650; 
const int PulsoMaximo = 2550; 

void setup() {
  Dispensador.attach(pinservo, PulsoMinimo, PulsoMaximo);
}
void loop() {
  Dispensador.write(110
  ); 
  delay(1000); 
  Dispensador.write(75); 
  delay(1000);
}
