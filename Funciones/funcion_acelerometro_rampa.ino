

bool acelerometro()
{

  int y;
  int estable = 3280; //delta = 200
  byte piny = 1;
  bool rampa;

  y = map(analogRead(piny), 0, 1023, 0, 10000);

  if (y < estable - 200 || y > estable + 200)
  {
    rampa = true;
  }
  else
  {
    rampa = false;
  }
  return rampa;


}



void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
