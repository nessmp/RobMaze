#include <Encoder.h>
#include "CurieIMU.h"
#include <CurieTime.h>

Encoder Enc(6, 8);

int const Enc90 = 3000;

void setup() {
  Serial.begin(9600);
  CurieIMU.begin();
  CurieIMU.setGyroRange(250);
}

//analogWrite de los motores para girar a la derecha
void Derecha() {}
//analogWrite de los motores para girar a la izquierda
void Izquierda() {}
//analogWrite de los motores para detenerse
void Detenerse() {}

//Funcion para girar a la izquierda 90 grados apoyandose de los encoders y del IMU
void giroIzq90()
{
  Enc.write(0); //iniciar cueta de los encoders desde 0
  int iCuentas = Enc.read(); //Realiza una lectura
  int iCounter = 0; //Contador para el avg del IMU
  int gxRaw; //almacena el dato en el eje X del gyro, no nos sirve
  int gyRaw; //almacena el dato en el eje Y del gyro, no nos sirve
  int gzRaw; //almacena el dato en el eje Z del gyro
  int avgGzRaw = 0; //Almacena el avg de los Raw del eje Z del gyro
  int Degrees = 0; //Almacena los grados que nos hemos movido
  byte bTime1 = second(); //Segundo en el que se empezo el movimiento
  if (bTime1 > 50) //En caso de ser mayor de 50 se regresa a 0, por seguridad
  {
    setTime(0); //inicializa los segundos en 0
    while (bTime1 != 0) //La funcion de setTime se tarda en surtir efecto, por lo tanto esperamos a que funcione
    {
      bTime1 = second();
    }
  }
  Izquierda(); //Iniciamos el moviemiento a la izquierda
  while (iCuentas < Enc90) //hasta que las cuentas de encoders den la constante de 90
  {
    CurieIMU.readGyro(gxRaw, gyRaw, gzRaw); //lee los datos del gyro
    avgGzRaw += gzRaw; //sumamos los datos del eje Z para despues hacer un avg
    iCounter++; //ayuda para el avg
    iCuentas = Enc.read(); //Actualizamos las cuentas del encoder para llegar a salir del while
  }
  Detenerse(); //Termina el movimiento
  byte bTime2 = second(); //Segundo en el que se acabo el movimiento
  byte bTime = bTime2 - bTime1; //Tiempo que se tardo en efectuar el movimiento
  avgGzRaw = avgGzRaw / iCounter; // avg de los datos del eje Z del gyro
  Degrees = (avgGzRaw * 250.0) / 32768.0; //convertimos el avg a velocidad angular
  Degrees = (Degrees / bTime) * 10 - 2; //Sacamos los grados con la velocidad angular y se le hace una pequeña corrección, observada experimentalmente
  while (Degrees < 78 || Degrees > 112) //Correcciones si el IMU no detecto el mov aproximado de 90 grados
  {
    while (Degrees < 78) //en caso de haberse movido menos de 90 grados
    {
      avgGzRaw = 0; //volvemos a iniciar todos los valores necesarios a 0
      iCounter = 0;
      Enc.write(0);
      bTime1 = second(); //segundo antes del mov
      if (bTime1 > 50)
      {
        setTime(0);
        while (bTime1 != 0)
        {
          bTime1 = second();
        }
      }
      Izquierda(); //inicia mov
      while (iCuentas < (Enc90 / 9)) //nos movemos 10 grados en vez de 90
      {
        CurieIMU.readGyro(gxRaw, gyRaw, gzRaw);
        avgGzRaw += gzRaw;
        iCounter++;
        iCuentas = Enc.read();
      }
      Detenerse(); //finaliza mov
      bTime2 = second(); //segundo despues del mov
      bTime = bTime2 - bTime1; //timepo en que se tardo realizar el mov
      avgGzRaw = avgGzRaw / iCounter; //avg del eje Z
      int Degrees2 = (avgGzRaw * 250.0) / 32768.0; //grados del mov de correccion
      Degrees2 = (Degrees2 / bTime) * 10 - 2;
      Degrees += Degrees2; //se suma estos grados a los grados que se movieron inicialmente
    }
    while (Degrees > 112) //En caso de haberse movido más de 90 grados
    {
      avgGzRaw = 0; //volvemos a iniciar todos los valores necesarios a 0
      iCounter = 0;
      Enc.write(0);
      bTime1 = second(); //segundo antes del mov
      if (bTime1 > 50)
      {
        setTime(0);
        while (bTime1 != 0)
        {
          bTime1 = second();
        }
      }
      Derecha(); //inicia mov
      while (iCuentas < (Enc90 / 9)) //nos movemos 10 grados en vez de 90
      {
        CurieIMU.readGyro(gxRaw, gyRaw, gzRaw);
        avgGzRaw += gzRaw;
        iCounter++;
        iCuentas = Enc.read();
      }
      Detenerse(); //finaliza mov
      bTime2 = second(); //segundo despues del mov
      bTime = bTime2 - bTime1; //timepo en que se tardo realizar el mov
      avgGzRaw = avgGzRaw / iCounter; //avg del eje Z
      int Degrees2 = (avgGzRaw * 250.0) / 32768.0; //grados del mov de correccion
      Degrees2 = (Degrees2 / bTime) * 10 - 2;
      Degrees += Degrees2; //se suma estos grados a los grados que se movieron inicialmente
    }
  }
}

//Funcion para girar a la derecha 90 grados apoyandose de los encoders y del IMU
void giroDer90()
{
  Enc.write(0); //iniciar cueta de los encoders desde 0
  int iCuentas = Enc.read(); //Realiza una lectura
  int iCounter = 0; //Contador para el avg del IMU
  int gxRaw; //almacena el dato en el eje X del gyro, no nos sirve
  int gyRaw; //almacena el dato en el eje Y del gyro, no nos sirve
  int gzRaw; //almacena el dato en el eje Z del gyro
  int avgGzRaw = 0; //Almacena el avg de los Raw del eje Z del gyro
  int Degrees = 0; //Almacena los grados que nos hemos movido
  byte bTime1 = second(); //Segundo en el que se empezo el movimiento
  if (bTime1 > 50) //En caso de ser mayor de 50 se regresa a 0, por seguridad
  {
    setTime(0); //inicializa los segundos en 0
    while (bTime1 != 0) //La funcion de setTime se tarda en surtir efecto, por lo tanto esperamos a que funcione
    {
      bTime1 = second();
    }
  }
  Derecha(); //Iniciamos el moviemiento a la izquierda
  while (iCuentas < Enc90) //hasta que las cuentas de encoders den la constante de 90
  {
    CurieIMU.readGyro(gxRaw, gyRaw, gzRaw); //lee los datos del gyro
    avgGzRaw += gzRaw; //sumamos los datos del eje Z para despues hacer un avg
    iCounter++; //ayuda para el avg
    iCuentas = Enc.read(); //Actualizamos las cuentas del encoder para llegar a salir del while
  }
  Detenerse(); //Termina el movimiento
  byte bTime2 = second(); //Segundo en el que se acabo el movimiento
  byte bTime = bTime2 - bTime1; //Tiempo que se tardo en efectuar el movimiento
  avgGzRaw = avgGzRaw / iCounter; // avg de los datos del eje Z del gyro
  Degrees = (avgGzRaw * 250.0) / 32768.0; //convertimos el avg a velocidad angular
  Degrees = (Degrees / bTime) * 10 - 2; //Sacamos los grados con la velocidad angular y se le hace una pequeña corrección, observada experimentalmente
  while (Degrees < -78 || Degrees > -112) //Correcciones si el IMU no detecto el mov aproximado de 90 grados
  {
    while (Degrees < -78) //en caso de haberse movido menos de 90 grados
    {
      avgGzRaw = 0; //volvemos a iniciar todos los valores necesarios a 0
      iCounter = 0;
      Enc.write(0);
      bTime1 = second(); //segundo antes del mov
      if (bTime1 > 50)
      {
        setTime(0);
        while (bTime1 != 0)
        {
          bTime1 = second();
        }
      }
      Derecha(); //inicia mov
      while (iCuentas < (Enc90 / 9)) //nos movemos 10 grados en vez de 90
      {
        CurieIMU.readGyro(gxRaw, gyRaw, gzRaw);
        avgGzRaw += gzRaw;
        iCounter++;
        iCuentas = Enc.read();
      }
      Detenerse(); //finaliza mov
      bTime2 = second(); //segundo despues del mov
      bTime = bTime2 - bTime1; //timepo en que se tardo realizar el mov
      avgGzRaw = avgGzRaw / iCounter; //avg del eje Z
      int Degrees2 = (avgGzRaw * 250.0) / 32768.0; //grados del mov de correccion
      Degrees2 = (Degrees2 / bTime) * 10 - 2;
      Degrees += Degrees2; //se suma estos grados a los grados que se movieron inicialmente
    }
    while (Degrees > -112) //En caso de haberse movido más de 90 grados
    {
      avgGzRaw = 0; //volvemos a iniciar todos los valores necesarios a 0
      iCounter = 0;
      Enc.write(0);
      bTime1 = second(); //segundo antes del mov
      if (bTime1 > 50)
      {
        setTime(0);
        while (bTime1 != 0)
        {
          bTime1 = second();
        }
      }
      Izquierda(); //inicia mov
      while (iCuentas < (Enc90 / 9)) //nos movemos 10 grados en vez de 90
      {
        CurieIMU.readGyro(gxRaw, gyRaw, gzRaw);
        avgGzRaw += gzRaw;
        iCounter++;
        iCuentas = Enc.read();
      }
      Detenerse(); //finaliza mov
      bTime2 = second(); //segundo despues del mov
      bTime = bTime2 - bTime1; //timepo en que se tardo realizar el mov
      avgGzRaw = avgGzRaw / iCounter; //avg del eje Z
      int Degrees2 = (avgGzRaw * 250.0) / 32768.0; //grados del mov de correccion
      Degrees2 = (Degrees2 / bTime) * 10 - 2;
      Degrees += Degrees2; //se suma estos grados a los grados que se movieron inicialmente
    }
  }
}

void loop() {
  Serial.println("EMPIEZA");
  giroIzq90();
  Serial.println("SALIO");
  Serial.println("......................");
}
