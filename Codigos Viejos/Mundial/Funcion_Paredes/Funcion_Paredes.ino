#include <Ultrasonico.h> //Libreria para los Ultrasonicos
#include <Sharp.h> //libreria de los Sharps, duh
#include <Wire.h> //Libreria para I2C
#include <ZX_Sensor.h> //Libreria para sensores ZX

//Sensores ZX
ZX_Sensor ZXDer(0x10); //ZX de la Der con I2C address 0x10
ZX_Sensor ZXIzq(0x11); //ZX de la Izq con I2C address 0x11

//Sharps
Sharp SharpDerEnf(0, 30); //Sharp de la Derecha Enf en el pin 0 con dist 30
Sharp SharpDerAtr(1, 30); //Sharp de la Derecha Atr en el pin 1 con dist 30
Sharp SharpIzqEnf(2, 30); //Sharp de la Izquierda Enf en el pin 2 con dist 30
Sharp SharpIzqAtr(3, 30); //Sharp de la Izquierda Atr en el pin 3 con dist 30

//Ultrasonico
Ultrasonico UltIzq(2, 3); //Ult de la Izq, trigger en pin 2 y echo en pin 3
Ultrasonico UltDer(4, 5); //Ult de la Der, trigger en pin 4 y echo en pin 5

void setup() {
  Serial.begin(9600);
  ZXDer.init(); //Inicializa sensores ZX
  ZXIzq.init();
}

//Regresa verdadero si detecta pared en la derecha
bool ParedDer()
{
  bool bPared = false; //Variable a regresar
  byte bX = ZXDer.readX(); //Lectura del eje X
  byte bZ = ZXDer.readZ(); //Lectura del eje Y
  if(bZ < 30 && (bX > 118 && bX < 122)) //Si Z es pequeño y X esta entre 119 y 121 hay pared
  {
    bPared = true;
  }
  return bPared;
}

//Regresa verdadero si detecta pared en la izquierda
bool ParedIzq()
{
  bool bPared = false; //Variable a regresar
  byte bX = ZXIzq.readX(); //Lectura del eje X
  byte bZ = ZXIzq.readZ(); //Lectura del eje Y
  if(bZ < 30 && (bX > 118 && bX < 122)) //Si Z es pequeño y X esta entre 119 y 121 hay pared
  {
    bPared = true;
  }
  return bPared;
}

//Regresa verdadero si detecta pared enfrente
bool ParedEnf()
{
  bool bPared = false; //Variable a regresar
  byte U1 = UltIzq.distance(); //Lectura del ultrasonico de la Izquierda
  byte U2 = UltDer.distance(); //Lectura del ultrasonico de la Derecha
  if(U1 < 20 && U2 < 20) //Si ambos ultrasonicos detectan una corta distancia, hay pared
  {
    bPared = true;
  }
  return bPared;
}

void loop() {

}
