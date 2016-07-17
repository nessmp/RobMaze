#include <Sharp.h>
#include <Wire.h>
#include <ZX_Sensor.h>
#include <SparkFunMLX90614.h> // SparkFunMLX90614 Arduino library

IRTherm therm; // Create an IRTherm object to interact with throughout

Sharp sharp0(0, 30); //Crea el objeto, (pin, distancia maxima del sensor)
Sharp sharp1(1, 30); //Crea el objeto, (pin, distancia maxima del sensor)
Sharp sharp2(2, 30); //Crea el objeto, (pin, distancia maxima del sensor)
Sharp sharp3(3, 30); //Crea el objeto, (pin, distancia maxima del sensor)

int trigger2 = 8;
int echo2 = 7;

int trigger1 = 10;
int echo1 = 9;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  pinMode(11, INPUT);
  pinMode(trigger1, OUTPUT); /*activación del pin 9 como salida: para el pulso ultrasónico*/
  pinMode(echo1, INPUT); /*activación del pin 8 como entrada: tiempo del rebote del ultrasonido*/
  pinMode(trigger2, OUTPUT); /*activación del pin 9 como salida: para el pulso ultrasónico*/
  pinMode(echo2, INPUT); /*activación del pin 8 como entrada: tiempo del rebote del ultrasonido*/
  therm.begin(0x5A); // Initialize thermal IR sensor
  therm.setUnit(TEMP_C); // Set the library's units to Farenheit
}

bool ParedEnf()
{
  bool bPared = false;
  digitalWrite(10, LOW);
  delayMicroseconds(5);
  digitalWrite(10, HIGH);
  delayMicroseconds(10);
  int tiempo = pulseIn(9, HIGH);
  int distancia1 = int(0.017 * tiempo);

  digitalWrite(8, LOW);
  delayMicroseconds(5);
  digitalWrite(8, HIGH);
  delayMicroseconds(10);
  tiempo = pulseIn(7, HIGH);

  int distancia2 = int(0.017 * tiempo);

  Serial.print(distancia1);
  Serial.print("\t");
  Serial.print(distancia2);
  Serial.print("\t");

  if(distancia1 < 20 && distancia2 < 20)
  {
    bPared = true;
  }
  return bPared;
}

void loop() {
  //Serial.println(ParedEnf());
  //therm.read();
  digitalWrite(trigger1, LOW);
  delayMicroseconds(5);
  digitalWrite(trigger1, HIGH);
  delayMicroseconds(10);
  int tiempo = pulseIn(echo1, HIGH);
  int distancia1 = int(0.017 * tiempo);

  digitalWrite(trigger2, LOW);
  delayMicroseconds(5);
  digitalWrite(trigger2, HIGH);
  delayMicroseconds(10);
  tiempo = pulseIn(echo2, HIGH);

  int distancia2 = int(0.017 * tiempo);
  Serial.print(sharp2.distance());

  Serial.print("\t");
  Serial.print(sharp3.distance());
  Serial.print("\t");
  Serial.print(pulseIn(11, LOW));
  
  Serial.print("\t");
  Serial.print(distancia1);
  Serial.print("\t");
  Serial.print(distancia2);
  Serial.print("\t");
  Serial.print(sharp0.distance());
  Serial.print("\t");
  Serial.print(sharp1.distance());
  Serial.print("\t");
  Serial.println(therm.object());
  Serial.println("-------------");
  delay(100);
}
