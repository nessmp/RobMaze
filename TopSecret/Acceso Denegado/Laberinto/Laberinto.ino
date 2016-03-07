//Paso
int iPass[400];
//Posicion
int iPos[400];
//posibles movimientos
int iPossibility[400];
//coordenadas posibles
int iRun[400][4];
//counter de los pasos
int iCounter = -1;
//coordenada
int iCoord = 2020;
//direcciones
bool bDireccion[4] = {false, false, false, false};

//Da el numero de posibilidades
int GetPossibility()
{
  //todo el arreglo a falso.
  bDireccion[0] = false;
  bDireccion[1] = false;
  bDireccion[2] = false;
  bDireccion[3] = false;
  
  //variable a regresar
  int iReturn = 0;

  //comprueba la Der
  int DistDerA = UltDA.ping_cm();
  int DistDerB = UltDB.ping_cm();
  if(DistDerA > 30 || DistDerB > 30)
  {
    bDireccion[0] = true;
    iReturn++;
  }

  //comprueba Enf
  int iDistEA = UltEA.ping_cm();
  int iDistEB = UltEB.ping_cm();
  if(iDistEA > 30 || iDistEB > 30)
  {
    bDireccion[1] = true;
    iReturn++;
  }

  //comprueba la Izq
  int iDistIzqA = UltIA.ping_cm();
  int iDistIzqB = UltIB.ping_cm();
  if(iDistIzqB > 30 || iDistIzqB > 30)
  {
    bDireccion[2] = true;
    iRetrun++; 
  }

  //comprueba atras
  int iDistAA = UltAA.ping_cm();
  int iDistAB = UltAB.ping_cm();
  if(iDistAA > 30 || iDistAB > 30)
  {
    bDireccion[3] = true;
    iReturn++;
  }
  return iReturn;
}

//da los datos de la posicion actual
void GetData()
{
  iCounter++;
  //llena iPass
  iPass[iCounter] = iCounter;
  //llena iPos
  iPos[iCounter] = iCoord;
  //llena iPossibility
  iPossibility[iCounter] = GetPossibility();
  //llena iRun
  for(int iI = 0; iI < iPossibility[iCounter]; iI++)
  {
    if(bDireccion[0] == true)
    {
      iRun[iCounter][iI] = iCoord + 100;
      bDireccion[0] = false;
    }
    
    else if(bDireccion[1] == true)
    {
      iRun[iCounter][iI] = iCoord + 1;
      bDireccion[1] = false;
    }

    else if(bDireccion[2] == true)
    {
      iRun[iCounter][iI] = iCoord - 100;
      bDireccion[2] = false;
    }

    else if(bDireccion[3] == true)
    {
      iRun[iCounter][iI] = iCoord - 1;
      bDireccion[3] = false;
    }
  }
}

//comprueva si ya se ha estado en la posicion mandada como paramtetro
bool BeenHere(int Pos)
{
  bool bReturn = false;
  for(int iI = iCounter; iI >= 0; iI--)
  {
    if(Pos == iPos[iI])
    {
      bReturn = true;
    }
  }
  return bReturn;
}

//se mueve al irun preferencial o regresa un true si es necesario buscar alguno que no este adyecente
bool Moverse()
{
  bool Next = false;
  bool Go = false;
  int GoHere = 0;
  for(int iI = 0; iI < 4; iI++)
  {
    Go = BeenHere(iRun[iCounter][iI]);
    if(Go == false)
    {
       GoHere = iRun[iCounter][iI];
       break;
    }
  }
  if(GoHere == 0)
  {
    Next = true;
  }
  else
  {
    if(iPos[iCounter] + 100 == GoHere) 
    {
      Derecha30();
      iCoord += 100;
    }
    else if(iPos[iCounter] + 1 == GoHere) 
    {
      Adelante30();
      iCoord += 1;
    }
    else if(iPos[iCounter] - 100 == GoHere) 
    {
      Izquierda30();
      iCoord -= 100;
    }
    else if(iPos[iCounter] - 1 == GoHere) 
    {
      Atras30();
      iCoord -= 1;
    }
  }
  return Next;
}

//busca y regresa el paso al cual quieres ir para moverte a una localizacion desconocida
int SearchWhereToGo()
{
   int iReturn = 999999;
   int iPassToGo;
   for(int iI = iCounter; iI >= 0; iI--)
   {
    for(int iJ = 0; iJ < iPossibility[iI]; iJ++)
    {
      int iHelper = 0;
      for(int iK = iI; iK >= 0; iK--)
      {
        if(iRun[iI][iJ] == iPos[iK])
        {
          iHelper++  
        }
      }
      if(iHelper = 0)
      {
        iReturn = iRun[iI][iJ];
        iPassToGo = iI;
        break;
      }
    }
    if(iReturn != 999999)
    {
      break;
    }
   }
   return iPassToGo;
   // return iReturn;
}

//consigue el iPass de el iRun que le des como parametro
int GetPass(int iRun)
{
  int iThis;
  int iOfThis = iRun;
  int iCopyPass = iPass;
  
  while(iRun != iPass)
  {
    iRun = iCopyPass;
    iCopyPass--;
  }
  
  iThis = iCopyPass;
  return iThis;
}

/*
void Moverse(int iHere, int iHelper)
{
  int iActual = iPos[iHelper]
  int iProxima = iPos[iHere]

  if(iActual == iProxima + 100)
  {
    Derecha30();
    iCoord += 100;
  }
  
  else if(iActual == iProxima + 1)
  {
    Enfrente30();
    iCoord += 1;
  }

  else if(iActual == iProxima - 100)
  {
    Izquierda30();
    iCoord -= 100;
  }

  else if(iActual == iProxima - 1)
  {
    Atras30();
    iCoord -= 1;
  }
}
*/

//Ir a la posicion que necesitas ir
void Go()
{
  int CopyCounter = iCounter;
  int iYouAreHere = iPass[iCopyCounter];
  int iGetHere = SearchWhereToGo();
  int iMinor[4];
  int iProx;
  int iHelper;

  while(YouAreHere != iGetHere)
  {
    for(int iI = 0; iI < iPossibility[iCopyCounter]; iI++)
    {
      iMinor[iI] = GetPass(iRun[iCounter][iI]);
    }
    for(int iI = 0; iI < iPossibility[iCopyCounter] - 1; iI++)
    {
      if(iMinor[iI] < iMinor[iI + 1])
      {
        /*
        if(iMinor[iI] >= iGetHere)
        {
          iProx = iMinor[iI];
        }
        */
        //contiene el ipass menor al cual se debe de mover
        iProx = iMinor[iI];
        iHelper = iI;
      }
    }
    Moverse(iProx,iHelper);
    YouAreHere = iPos[iProx];
  }
}

void Laberinto()
{
  GetData();
  bool Next = Moverse();
  if(Next == true)
  {
    SearchWhereToGo();
    Go();
  }
}

