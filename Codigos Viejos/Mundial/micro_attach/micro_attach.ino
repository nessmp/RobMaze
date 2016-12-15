#include <SparkFunMLX90614.h> //libreria de los MLX

int I2C_Address_MLX1 = 0x4C;
IRTherm therm1; //Primer MLX
int I2C_Address_MLX2 = 0x1C;
IRTherm therm2; //Primer MLX

int AmbientTemp = 201;
int Calor1,Calor2;
byte counter = 0;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  therm1.begin(I2C_Address_MLX1); //Primer MLX
  therm1.setUnit(TEMP_C);
    therm2.begin(I2C_Address_MLX2); //Primer MLX
  therm2.setUnit(TEMP_C);
  while (AmbientTemp > 200)
  {
    therm1.read();
    AmbientTemp = therm1.ambient() + 2;
  }
  Serial.print("--->");
  Serial.println(AmbientTemp);

  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  therm1.read();
  therm2.read();
  Serial.print("--->");
  Serial.print(AmbientTemp);
  Serial.print("\t");
  Calor1 = therm1.object();
  Calor2= therm2.object();
  Serial.print(Calor1);
   Serial.print("\t");
    Serial.println(Calor2);
    
   digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  if (Calor1 >= AmbientTemp && counter == 0)
  {
    digitalWrite(9, LOW);
    counter++;
   
  }
  if (Calor2 >= AmbientTemp && counter == 0)
  {
    digitalWrite(8, LOW);
    counter++;
   
  }
  else
  {
    counter = 0;
  }
  
  digitalWrite(9, HIGH);
   digitalWrite(8, HIGH);

}
