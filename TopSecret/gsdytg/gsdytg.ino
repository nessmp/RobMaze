#include <NewPing.h>
#define MAX_DISTANCE 200

////////////////////////////
//Ultrasonicos de enfrente//
////////////////////////////

//ultrasonicos de enfrente
byte bTriggerE1 = 3;
byte bEchoE1 = 4;

NewPing sonarE1(bTriggerE1, bEchoE1, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE1.ping_cm();

byte bTriggerE2 = 5;
byte bEchoE2 = 6;

NewPing sonarE2(bTriggerE2, bEchoE2, MAX_DISTANCE);  //llamar a la funcion para saber la distancia con sonarE2.ping_cm();

//////////////////////////////
//Ultrasonicos de la derecha//
//////////////////////////////

//ultraonicos de la derecha
byte bTriggerD1 = 7;
byte bEchoD1 = 8;

NewPing sonarD1(bTriggerD1, bEchoD1, MAX_DISTANCE); //llamar a la funcion para saber la distancia con sonarD1.ping_cm();

byte bTriggerD2 = 9;
byte bEchoD2 = 10;

NewPing sonarD2(bTriggerD2, bEchoD2, MAX_DISTANCE); //llamar a la funcion para saber la distancia con sonarD2.ping_cm();

/////////////////////////
//Ultrasonicos de atras//
/////////////////////////

//ultraonicos de atras
byte bTriggerA1 = 11;
byte bEchoA1 = 12;

NewPing sonarA1(bTriggerA1, bEchoA1, MAX_DISTANCE); //llamar a la funcion para saber la distancia con sonarA1.ping_cm();

byte bTriggerA2 = 13;
byte bEchoA2 = 14;

NewPing sonarA2(bTriggerA2, bEchoA2, MAX_DISTANCE); //llamar a la funcion para saber la distancia con sonarA2.ping_cm();

////////////////////////////////
//Ultrasonicos de la izquierda//
////////////////////////////////

//ultraonicos de la Izquierda
byte bTriggerI1 = 15;
byte bEchoI1 = 16;

NewPing sonarI1(bTriggerI1, bEchoI1, MAX_DISTANCE); //llamar a la funcion para saber la distancia con sonarI1.ping_cm();

byte bTriggerI2 = 17;
byte bEchoI2 = 18;

NewPing sonarI2(bTriggerI2, bEchoI2, MAX_DISTANCE); //llamar a la funcion para saber la distancia con sonarI2.ping_cm();

/////////////
//variables//
/////////////

const byte iMAX = 50 //definir un meximo en los array, intentar que el numero sea el mas pequeño posible
byte iPass[400]; //numero de paso en el que te encuentras
byte iPos[400]; //coordenada en la que te encuentras en dicho paso [paso]
byte iPossibility[4]; //posibilidades de moviemiento que existen en ese [paso]
byte iRun [400][4]; // [paso][posibilidad] da la coordenada de la posibilidad a la que te puedes mover por prioridades
byte iX = 2000; //Coordenada
byte iY = 20; //Coordenada
byte iWay = 0; 
byte bCount = 0;
bool bOldPosition = false;
int iI = -1;
bool Direccion [4] = {false, false, false, false};

void setup() {
  ////////////////////////
  //Setup de los motores//
  ////////////////////////
  
  pinMode(motDerE1, OUTPUT);
  pinMode(motDerE2, OUTPUT);
  
  pinMode(motDerA1, OUTPUT);
  pinMode(motDerA2, OUTPUT);

  pinMode(motIzqE1, OUTPUT);
  pinMode(motIzqE2, OUTPUT);

  pinMode(motIzqA1, OUTPUT);
  pinMode(motIzqA2, OUTPUT);

}

///////////////////////////
//funciones de movimiento//
///////////////////////////

//Funcion para apagar motores
void Detenerse()
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 0);
}

//funcion para moverse hacia adelante
void Adelante(int &iX; int &iY)
{
  analogWrite(motDerE1, 255);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 255);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 255);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 255);
  analogWrite(motIzqA2, 0);

  iY += 1; //mueve el valor de la coordenada para iPos
}

//funcion para moverse hacia atras
void Atras(int &iX; int &iY)
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 255);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 255);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 255);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 255);
  iY -= 1; //mueve el valor de la coordenada para iPos
}

//funcion para moverse hacia adelante
void Derecha(int &iX; int &iY)
{
  analogWrite(motDerE1, 255);
  analogWrite(motDerE2, 0);

  analogWrite(motDerA1, 255);
  analogWrite(motDerA2, 0);

  analogWrite(motIzqE1, 255);
  analogWrite(motIzqE2, 0);

  analogWrite(motIzqA1, 255);
  analogWrite(motIzqA2, 0);
  iX += 100; //mueve el valor de la coordenada para iPos
}

//funcion para moverse hacia izquierda
void Izquierda(int &iX; int &iY)
{
  analogWrite(motDerE1, 0);
  analogWrite(motDerE2, 255);

  analogWrite(motDerA1, 0);
  analogWrite(motDerA2, 255);

  analogWrite(motIzqE1, 0);
  analogWrite(motIzqE2, 255);

  analogWrite(motIzqA1, 0);
  analogWrite(motIzqA2, 255);
  iX-= 100; //mueve el valor de la coordenada para iPos
}

///////////////////////////////////////
//////////////////////////////////////

//da el numero de posibles direcciones a las que te puedes mover
int Posib(bool &Direccion[])
{
  int iPosWays = 0;

     //Revisa ultrasonicos de la derecha
     int Dist1 = (sonarD1.ping() / US_ROUNDTRIP_CM);
     int Dist2 = (sonarD2.ping() / US_ROUNDTRIP_CM);
     if(Dist1 >= 30 && Dist2 >= 30)
     {
      iPosWays++;
      Direccion [0] = true;
     }

     //Revisa ultrasonicos de enfrente
     Dist1 = (sonarE1.ping() / US_ROUNDTRIP_CM);
     Dist2 = (sonarE2.ping() / US_ROUNDTRIP_CM);
     if(Dist1 >= 30 && Dist2 >= 30)
     {
      iPosWays++;
      Direccion [1] = true;
     }

     //Revisa ultrasonicos de la izquierda
     Dist1 = (sonarI1.ping() / US_ROUNDTRIP_CM);
     Dist2 = (sonarI2.ping() / US_ROUNDTRIP_CM);
     if(Dist1 >= 30 && Dist2 >= 30)
     {
      iPosWays++;
      Direccion [2] = true;
     }

     //Revisa ultrasonicos de atras
     Dist1 = (sonarA1.ping() / US_ROUNDTRIP_CM);
     Dist2 = (sonarA2.ping() / US_ROUNDTRIP_CM);
     if(Dist1 >= 30 && Dist2 >= 30)
     {
      iPosWays++;
      Direccion [3] = true;
     }
     
     return iPosWays;
}

//funcion para conseguir valor de variables
void GetVariables(int &iPass[]; int &iPos[]; int &iPossibility[]; int iRun[][])
{
    //suma uno al paso en el que vamos
    iI += 1;

    //pone todo Direccion en falso
    for(int I = 0; I < 4; I++)
    {
       Direccion [I] = false;
    }
    
    iPass[iI] = iI;
    iPos[iI] = iX + iY;
    iPossibility [iI] = Posib(Direccion);
    
    for(int iJ = 0; iJ < iPossibility [iI]; iJ++)
    {
      int iA = iX
      int iB = iY

      if(Direccion[0] == true)
      {
        iRun[iI][iJ] = (iA + 100) +iB;
        Direccion[0] = false;
      }
      else if(Direccion[1] == true)
      {
        iRun[iI][iJ] = iA + (iB +1);
        Direccion[1] = false;
      }
      else if(Direccion[2] == true)
      {
        iRun[iI][iJ] = (iA - 100) + iB;
        Direccion[2] = false;
      }
      else if(Direccion[3] == true)
      {
        iRun[iI][iJ] = iA + (iB - 1);
        Direccion[3] = false;
      }
    }
  
}

//regresa bOldPosition como verdadera en caso de que ya se haya estado en esa coordenada
void CheckPosition(int &iPass[]; int &iPos[]; int &iPossibility[]; int iRun[][]; bool &bOldPosition)
{
  bOldPosition = false;

    do{
       for(int iK = 0; iK < 400; iK++)
       {
         for(int iJ = 0; iJ < 4; iJ++)
         {
           if(iRun[iK][iJ] == iPos[iK])
           {
             bOldPosition = true; 
           }
         }
       }
       if(bOldPosition == false)
       {
        break;
       }
    }while(bOldPosition = false)
}

void RobotOfTime(int &iPass[]; int &iPos[]; int &iPossibility[]; int iRun[][])
{
  for(int iK = 400; iK >= 0; iK--)
  {
    
  }
}

//funcion para cruzar el laberinto
void Laberinto()
{
  GetVariables(iPass; iPos; iPossibility; iRun)
  CheckPosition(iPass; iPos; iPossibility; iRun; bOldPosition)
  if(bOldPosition == true)
  {
     RobotOfTime(iPass; iPos; iPossibility; iRun)
  }
}

void loop() {
  
}
