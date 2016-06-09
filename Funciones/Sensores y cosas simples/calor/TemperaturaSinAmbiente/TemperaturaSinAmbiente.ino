#include <Wire.h> // I2C library, required for MLX90614
#include <SparkFunMLX90614.h> // SparkFunMLX90614 Arduino library

IRTherm calor1; // Create an IRTherm object to interact with throughout

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Initialize Serial to log output
  calor1.begin(0x5A); // Initialize thermal IR sensor
}

void loop() {
  calor1.read(); 
    // Use the object() and ambient() functions to grab the object and ambient
  // temperatures.
  // They'll be floats, calculated out to the unit you set with setUnit().
    Serial.print("Object 1: " + String(calor1.object()));
    Serial.write('°'); // Degree Symbol
    Serial.println("C");

    delay(100);
}
