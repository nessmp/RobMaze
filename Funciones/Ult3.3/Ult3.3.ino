unsigned int pingUlt()
{
   int MAX_SENSOR_DELAY = 5800;
   int max_cm_distance = 30;
   int MAX_SENSOR_DISTANCE = 500;
   int US_ROUNDTRIP_CM = 57;
   int PING_OVERHEAD = 5;
   
   int _maxEchoTime = min(max_cm_distance + 1, (unsigned int) MAX_SENSOR_DISTANCE + 1) * US_ROUNDTRIP_CM;
   int _max_time = micros() + _maxEchoTime + MAX_SENSOR_DELAY;
   return (micros() - (_max_time - _maxEchoTime) - PING_OVERHEAD); // Calculate ping time, include overhead.
}


unsigned long ping_cmUlt() {
  int US_ROUNDTRIP_CM = 57;
  unsigned long echoTime = pingUlt();         // Calls the ping method and returns with the ping echo distance in uS.
//#if ROUNDING_ENABLED == false
  return (echoTime / US_ROUNDTRIP_CM);  // Call the ping method and returns the distance in centimeters (no rounding).
  /*
#else
  return NewPingConvert(echoTime, US_ROUNDTRIP_CM); // Convert uS to centimeters.
#endif
*/
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int distanceUlt = ping_cmUlt();
  Serial.println(distanceUlt);
  delay(100);
}
