#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "CurieIMU.h"

int ax, ay, az;         // accelerometer values
int gx, gy, gz;         // gyrometer values

const int ledPin = 13;      // activity LED pin
boolean blinkState = false; // state of the LED

int calibrateOffsets = 1; // int to determine whether calibration takes place or not


Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(2);


byte irPin = A0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(9, OUTPUT); /*activación del pin 9 como salida: para el pulso ultrasónico*/
  pinMode(8, INPUT); /*activación del pin 8 como entrada: tiempo del rebote del ultrasonido*/
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A0, INPUT);
  AFMS.begin();
    myMotor->setSpeed(255);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
  CurieIMU.begin();
  if (calibrateOffsets == 1) {
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -235
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 168
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
    CurieIMU.autoCalibrateGyroOffset();
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
  }
}

int distUlt()
{
  CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);
  int Trigger = 9;
  int Echo = 8;
  long distancia;
  long tiempo;
  // put your main code here, to run repeatedly:
  digitalWrite(Trigger,LOW); /* Por cuestión de estabilización del sensor*/
  delayMicroseconds(5);
  digitalWrite(Trigger, HIGH); /* envío del pulso ultrasónico*/
  delayMicroseconds(10);
  tiempo=pulseIn(Echo, HIGH); /* Función para medir la longitud del pulso entrante. Mide el tiempo que transcurrido entre el envío
  del pulso ultrasónico y cuando el sensor recibe el rebote, es decir: desde que el pin 12 empieza a recibir el rebote, HIGH, hasta que
  deja de hacerlo, LOW, la longitud del pulso entrante*/
  distancia= int(0.017*tiempo); /*fórmula para calcular la distancia obteniendo un valor entero*/
  /*Monitorización en centímetros por el monitor serial*/
  return distancia;
}

int cm()
{
    int raw = analogRead(irPin);
    float voltFromRaw = map(raw, 0, 1023, 0, 3300); //Cambiar 5000 por 3300
    
    int puntualDistance;
    
    puntualDistance = 27.728 * pow(voltFromRaw / 1000, -1.2045);  
    
     /*
     * float volts = analogRead(IRpin)*0.0048828125;   // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
     *float distance = 65*pow(volts, -1.10);          // worked out from graph 65 = theretical distance / (1/Volts)S - luckylarry.co.uk
     *Serial.println(distance);                       // print the distance
     *delay(100);                                     // arbitary wait time.
     */

    return puntualDistance;
}

int Sharp()
{   
    int _p = 0;
    int _sum = 0;
    int _avg = 25;
    int _tol = 93 / 100;
    int _previousDistance = 0;
    
    
    for (int i=0; i<_avg; i++)
    {    
        int foo= cm();
       
        if (foo>=(_tol*_previousDistance))
        {
            _previousDistance=foo;
            _sum=_sum+foo;
            _p++;        
        }  
    }
    
    int accurateDistance=_sum/_p;
    
    return accurateDistance;
}

void loop() {
  CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);
  int DistUlt = distUlt();
  int distanceSharp = Sharp();
  if(ay < 10000)
  {
    myMotor->run(FORWARD);
    Serial.print("Ultrasonico: ");
    Serial.println(DistUlt);
    Serial.print("Sharp: ");
    Serial.println(distanceSharp);
    Serial.println();
  }
  else if(ay > 10000)
  {
    myMotor->run(RELEASE);
  }
  delay(30);
}
