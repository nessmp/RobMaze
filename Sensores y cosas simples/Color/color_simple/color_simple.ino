const int s0 = 18;  
const int s1 = 17;  
const int s2 = 20;  
const int s3 = 19;  
const int out = 21;   

// Variables  
int red = 0;  
int green = 0;  
int blue = 0;  
String colon = "";
    
void setup()   
{  
  Serial.begin(9600); 
  pinMode(s0, OUTPUT);  
  pinMode(s1, OUTPUT);  
  pinMode(s2, OUTPUT);  
  pinMode(s3, OUTPUT);  

  //digitalWrite(s0, HIGH);  
  //digitalWrite(s1, HIGH);  
}  

String color()  
{    
  digitalWrite(s2, LOW);  
  digitalWrite(s3, LOW);  
  //count OUT, pRed, RED  
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
  digitalWrite(s3, HIGH);  
  //count OUT, pBLUE, BLUE  
  blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
  digitalWrite(s2, HIGH);  
  //count OUT, pGreen, GREEN  
  green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  

  Serial.println("red: ");
  Serial.println(red);
  Serial.println("green: ");
  Serial.println(green);
  Serial.println("blue: ");
  Serial.println(blue);
  

   if ((green >= 3 && green <= 15) && (blue >= 3 && blue <= 15) && (red >= 3 &&  red <= 15))
  {
    colon = "white";
    
  }
  else if ((green > 25 && green < 32) && (blue > 25 && blue < 32) && (red > 45 &&  red < 62))
  {
    colon = "green";
  }
  else if ((green > 55 && green < 72) && (blue > 40 && blue < 52) && (red > 70 &&  red < 85))
  {
    colon = "black";
  }
  else if ((green > 25 && green < 36) && (blue > 18 && blue < 27) && (red > 8 &&  red < 15))
  {
    colon = "red";
  }
  return colon;
}

void ColorNuevo()
{
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  Serial.println(red);
}
    
void loop() 
{
  color();
} 

