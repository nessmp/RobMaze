#include "CurieIMU.h"
#include <CurieTime.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);
  CurieIMU.begin();
  CurieIMU.setGyroRange(250);
  CurieIMU.autoCalibrateGyroOffset();
}

void loop() {
  int gxRaw, gyRaw, gzRaw;         // raw gyro values
  float gx, gy, gz;  //Valores para almacenar velocidad angular
  float avgGZ = 0;
  float Degrees = 0;
  int iCounter = 0;
  //setTime(0);
  int Time = second(); //Tiempor en seg
  if (Time > 50)
  {
    setTime(0);
    while (Time != 0)
    {
      Time = second();
    }
  }
  Serial.print("Time: ");
  Serial.println(Time);
  int Time2 = Time + 3;
  Serial.print("Time2: ");
  Serial.println(Time2);
  if (Time2 > 59)
  {
    Time2 = 0;
  }

  while (Time < Time2)
  {
    CurieIMU.readGyro(gxRaw, gyRaw, gzRaw);
    //gz = (gzRaw * 250.0) / 32768.0;
    //avgGZ += gz;
    avgGZ += gzRaw;
    iCounter++;
    Time = second();
  }
  avgGZ = avgGZ / iCounter;
  avgGZ = (avgGZ * 250.0) / 32768.0;
  Degrees = avgGZ / 3;
  int Grados = Degrees * 10 - 2;
  Serial.print("Grados: ");
  Serial.println(Grados);
  //delay(1000);


}
