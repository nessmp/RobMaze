///////////////////////
///////////////////////
#define ENCODER_A_PIN 3
#define ENCODER_B_PIN 8

long position;

void setup()
{
   Serial.begin(9600);
   Serial.println("Started");
   
   pinMode(ENCODER_A_PIN, INPUT);
   pinMode(ENCODER_B_PIN, INPUT);
   
   attachInterrupt(0, read_quadrature, CHANGE);
}

void loop()
{
   Serial.print("Position: ");
   Serial.println(position, DEC);
}

void read_quadrature()
{  
  // found a low-to-high on channel A
  if (digitalRead(ENCODER_A_PIN) == HIGH)
  {   
    // check channel B to see which way
    if (digitalRead(ENCODER_B_PIN) == LOW)
        position++;
    else
        position--;
  }
  // found a high-to-low on channel A
  else                                        
  {
    // check channel B to see which way
    if (digitalRead(ENCODER_B_PIN) == LOW)
        position--;
    else
        position++;
  }
}
