#include "Sharp.h"
#include <Arduino.h>

Sharp::Sharp()
{
  maxDistance = 0;
  irPin = 0;
}

Sharp::Shapr(int irPin, int maxDistance)
{
  this -> irPin = irPin;
  this -> maxDistance = maxDistance;
}

Sharp::~Sharp()
{
  //dtor
}

int Sharp::distance()
{
  int _p = 0;
  int _sum = 0;
  int _avg = 25;
  int _tol = 93 / 100;
  int _previousDistance = 0;


  for (int i = 0; i < _avg; i++)
  {
    int foo = cm();

    if (foo >= (_tol * _previousDistance))
    {
      _previousDistance = foo;
      _sum = _sum + foo;
      _p++;
    }
  }

  int accurateDistance = _sum / _p;

  return accurateDistance;
}

int Sharp::cm()
{
  if (30 == maxDistance)
  {
    int raw = analogRead(irPin);
    float voltFromRaw = map(raw, 0, 1023, 0, 3300); //Cambiar 5000 por 3300

    int puntualDistance;

    puntualDistance = 27.728 * pow(voltFromRaw / 1000, -1.2045);
  }

  else if (80 == maxDistance)
  {

    int raw = analogRead(irPin);
    float voltFromRaw = map(raw, 0, 1023, 0, 3300); //Cambiar 5000 por 3300

    int puntualDistance;

    puntualDistance = 11.83 * pow(voltFromRaw / 1000, -1.2045);
  }

  return puntualDistance;
}
