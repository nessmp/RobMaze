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
    digitalWrite(iTrigger, LOW); /* Por cuesti�n de estabilizaci�n del sensor*/
    delayMicroseconds(5);
    digitalWrite(iTrigger, HIGH); /* env�o del pulso ultras�nico*/
    delayMicroseconds(10);
    tiempo = pulseIn(iEcho, HIGH); /* Funci�n para medir la longitud del pulso entrante. Mide el tiempo que transcurrido entre el env�o
    del pulso ultras�nico y cuando el sensor recibe el rebote, es decir: desde que el pin 12 empieza a recibir el rebote, HIGH, hasta que
    deja de hacerlo, LOW, la longitud del pulso entrante*/
    distancia = int(0.017 * tiempo); /*f�rmula para calcular la distancia obteniendo un valor entero*/
    /*Monitorizaci�n en cent�metros por el monitor serial*/
    return distancia;
}
