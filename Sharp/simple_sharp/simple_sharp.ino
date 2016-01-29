///
/// Utilización del sensor de distancia Sharp 2Y0A21
/// by RafaG
///
 
// Pines de lectura
int ir_sensor0 = A0;

 
void setup()
{
  // inicia comunicaciones serie a 9600 bps
  Serial.begin(9600);
}
 
void loop()
{
  int lectura, cm;
 
  lectura = analogRead(ir_sensor0); // lectura del sensor 0
  cm = pow(3027.4 / lectura, 1.2134); // conversión a centímetros
  Serial.print("Sensor 0: ");
  Serial.println(cm); // lectura del sensor 0
  delay(500); // tiempo de espera
}
