byte irPin = A0;

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A0, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int distanceSharp = Sharp();
  Serial.println(distanceSharp);
  delay(250);
}
