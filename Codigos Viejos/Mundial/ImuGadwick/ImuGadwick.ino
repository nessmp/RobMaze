#include "CurieIMU.h"
#include <CurieTime.h>
#include <MadgwickAHRS.h>

Madgwick madG;

#define M_PI   3.14159265358979323846264338327950288

int ax, ay, az;         // accelerometer values
int gx, gy, gz;         // gyrometer values
float yaw, pitch, roll;

int factor = 200;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  CurieIMU.begin();
  CurieIMU.setGyroRange(250);
  CurieIMU.autoCalibrateGyroOffset();
}

void loop() {
  ax = CurieIMU.getAccelerationX();
  ay = CurieIMU.getAccelerationY();
  az = CurieIMU.getAccelerationZ();
  gx = CurieIMU.getRotationX();
  gy = CurieIMU.getRotationY();
  gz = CurieIMU.getRotationZ();

  madG.updateIMU(gx / factor, gy / factor, gz / factor, ax, ay, az);

  yaw = madG.getYaw();
  roll = madG.getRoll();
  pitch = madG.getPitch();

  Serial.print("yaw:\t");
  Serial.print(yaw * 180.0 / M_PI);
  Serial.print("\t roll:\t");
  Serial.print(roll * 180.0 / M_PI);
  Serial.print("\t pitch:\t");
  Serial.println(pitch * 180.0 / M_PI);
}
