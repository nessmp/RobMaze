#include "Ultrasonico.h"
#include "Arduino.h"

Ultrasonico::Ultrasonico()
{
    //ctor
}

Ultrasonico::Ultrasonico(int iTrigger, int iEcho)
{
    this -> iTrigger = iTrigger;
    this -> iEcho = iEcho;
}

Ultrasonico::~Ultrasonico()
{
    //dtor
}

int Ultrasonico::distance()
{
    long distancia;
    long tiempo;
    // put your main code here, to run repeatedly:
    digitalWrite(iTrigger, LOW); /* Por cuestión de estabilización del sensor*/
    delayMicroseconds(5);
    digitalWrite(iTrigger, HIGH); /* envío del pulso ultrasónico*/
    delayMicroseconds(10);
    tiempo = pulseIn(iEcho, HIGH); /* Función para medir la longitud del pulso entrante. Mide el tiempo que transcurrido entre el envío
    del pulso ultrasónico y cuando el sensor recibe el rebote, es decir: desde que el pin 12 empieza a recibir el rebote, HIGH, hasta que
    deja de hacerlo, LOW, la longitud del pulso entrante*/
    distancia = int(0.017 * tiempo); /*fórmula para calcular la distancia obteniendo un valor entero*/
    /*Monitorización en centímetros por el monitor serial*/
    return distancia;
}
