#include "Arduino.h"
#include "Sharp.h"

Sharp::Sharp()
{
  maxDistance = 0;
  irPin = 0;
}

Sharp::Sharp(int irPin, int maxDistance)
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
  int puntualDistance;
  float voltFromRaw;
  int raw = analogRead(irPin);
  if (30 == maxDistance)
  {
    voltFromRaw = map(raw, 0, 1023, 0, 3300);

    puntualDistance = 27.728 * pow(voltFromRaw / 1000, -1.2045);
  }

  else if (80 == maxDistance)
  {
    voltFromRaw = map(raw, 0, 1023, 0, 3300);

    puntualDistance = 11.83 * pow(voltFromRaw / 1000, -1.2045);
  }
  return puntualDistance;
}
