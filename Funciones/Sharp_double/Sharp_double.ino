#include <SharpDouble.h>
#define ir A1
#define model 1080


SharpDouble sharp(ir, 25, 93, model);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode (ir, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  double dis=sharp.distance();  // this returns the distance to the object you're measuring

  Serial.print("Mean distance: ");  // returns it to the serial monitor
  Serial.println(dis);
  delay(800);
}
