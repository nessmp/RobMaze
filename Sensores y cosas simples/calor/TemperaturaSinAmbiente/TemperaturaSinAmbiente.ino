#include <Wire.h> // I2C library, required for MLX90614
#include <SparkFunMLX90614.h> // SparkFunMLX90614 Arduino library

IRTherm calor1; // Create an IRTherm object to interact with throughout
IRTherm calor2; // Create an IRTherm object to interact with throughout
IRTherm calor3; // Create an IRTherm object to interact with throughout
IRTherm calor4; // Create an IRTherm object to interact with throughout

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Initialize Serial to log output
  calor1.begin(0x4C); // Initialize thermal IR sensor
  /*
  calor1.setUnit(TEMP_C); 
  calor2.begin(0x2C); // Initialize thermal IR sensor
  calor2.setUnit(TEMP_C); 
  calor3.begin(0x3C); // Initialize thermal IR sensor
  calor3.setUnit(TEMP_C); 
  calor4.begin(0x4C); // Initialize thermal IR sensor
  calor4.setUnit(TEMP_C); 
  */
}

void loop() {
    // Use the object() and ambient() functions to grab the object and ambient
  // temperatures.
  // They'll be floats, calculated out to the unit you set with setUnit().
    Serial.print("Object 1: " + String(calor1.object(), 2));
    Serial.write('°'); // Degree Symbol
    Serial.println("C");
    /*
    delay(100);
    
    Serial.print("Object 2: " + String(calor2.object(), 2));
    Serial.write('°'); // Degree Symbol
    Serial.println("C");

    delay(100);

    Serial.print("Object 3: " + String(calor3.object(), 2));
    Serial.write('°'); // Degree Symbol
    Serial.println("C");

    delay(100);

    Serial.print("Object 4: " + String(calor4.object(), 2));
    Serial.write('°'); // Degree Symbol
    Serial.println("C");
    */

    delay(100);
}
