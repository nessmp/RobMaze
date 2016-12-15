#include "CurieIMU.h"
#include <CurieTime.h>
#include <MadgwickAHRS.h>

Madgwick madG;

#define M_PI   3.14159265358979323846264338327950288

int ax, ay, az;
int gx, gy, gz;

int factor = 200;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  CurieIMU.begin();
  CurieIMU.setGyroRange(250);
  CurieIMU.autoCalibrateGyroOffset();
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
}

void loop() {
  int setPoint = 0;
  int actualPoint = 0;
  int velDer = 0;
  int velIzq = 0;

  CurieIMU.readAccelerometer(ax, ay, az);
  float g = (ax / 32768.0) * CurieIMU.getAccelerometerRange();
  Serial.println(g);
  if (g > .38)
  {
    ax = CurieIMU.getAccelerationX();
    ay = CurieIMU.getAccelerationY();
    az = CurieIMU.getAccelerationZ();
    gx = CurieIMU.getRotationX();
    gy = CurieIMU.getRotationY();
    gz = CurieIMU.getRotationZ();
    madG.updateIMU(gx / factor, gy / factor, gz / factor, ax, ay, az);

    setPoint = madG.getYaw() * 180.0 / M_PI;
    do {
      ax = CurieIMU.getAccelerationX();
      ay = CurieIMU.getAccelerationY();
      az = CurieIMU.getAccelerationZ();
      gx = CurieIMU.getRotationX();
      gy = CurieIMU.getRotationY();
      gz = CurieIMU.getRotationZ();
      madG.updateIMU(gx / factor, gy / factor, gz / factor, ax, ay, az);
      actualPoint = madG.getYaw() * 180.0 / M_PI;
      Serial.println(actualPoint);
      if (actualPoint > setPoint + 12)
      {
        velDer++;
        velIzq = 0;
      }
      else if (actualPoint < setPoint - 12)
      {
        velIzq++;
        velDer = 0;
      }
      else
      {
        velDer = 0;
        velIzq = 0;
      }
      analogWrite(3, velDer);
      analogWrite(5, velIzq);
      CurieIMU.readAccelerometer(ax, ay, az);
      g = (ax / 32768.0) * CurieIMU.getAccelerometerRange();
    } while (g > .38);
  }
  else
  {
    analogWrite(3, 0);
    analogWrite(5, 0);
  }
}
