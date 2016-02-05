/////////////////////////////
////Conctarlo a 3.3 VOLTS////
/////////////////////////////

#include <Wire.h> // I2C library, required for MLX90614
#include <SparkFunMLX90614.h> // SparkFunMLX90614 Arduino library

IRTherm calor1; // Create an IRTherm object to interact with throughout

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Initialize Serial to log output
  calor1.begin(0x1B); // Initialize thermal IR sensor
  calor1.setUnit(TEMP_C); 
}

void loop() {
  if (calor1.read()) // On success, read() will return 1, on fail 0.
  {
    // Use the object() and ambient() functions to grab the object and ambient
  // temperatures.
  // They'll be floats, calculated out to the unit you set with setUnit().
    Serial.print("Object: " + String(calor1.object(), 2));
    Serial.write('°'); // Degree Symbol
    Serial.println("C");
  }
}
