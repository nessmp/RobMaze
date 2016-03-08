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
//Direccion
int iDireccion = 1;
int iMovimiento = 0;

//Actualiza el valor de coord
void setCoord(int iDirecc, int iMov)
{
  if (iDirecc == 1)
  {
    if (iMov == 1)
    {
      iCoord += 100;
    }
    else if (iMov == 2)
    {
      iCoord += 1;
    }
    else if (iMov == 3)
    {
      iCoord -= 100;
    }
    else if (iMov == 4)
    {
      iCoord -= 1;
    }
  }

  else if (iDirecc == 2)
  {
    if (iMov == 1)
    {
      iCoord += 1;
    }
    else if (iMov == 2)
    {
      iCoord -= 100;
    }
    else if (iMov == 3)
    {
      iCoord -= 1;
    }
    else if (iMov == 4)
    {
      iCoord += 100;
    }
  }

  else if (iDirecc == 3)
  {
    if (iMov == 1)
    {
      iCoord -= 100;
    }

    else if (iMov == 2)
    {
      iCoord -= 1;
    }

    else if (iMov == 3)
    {
      iCoord += 100;
    }

    else if (iMov == 4)
    {
      iCoord += 1;
    }
  }

  else if (iDirecc == 4)
  {
    if (iMov == 1)
    {
      iCoord -= 1;
    }

    else if (iMov == 2)
    {
      iCoord += 100;
    }

    else if (iMov == 3)
    {
      iCoord += 1;
    }

    else if (iMov == 4)
    {
      iCoord -= 100;
    }
  }
}

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
  if (DistDerA > 30 || DistDerB > 30)
  {
    bDireccion[0] = true;
    iReturn++;
  }

  //comprueba Enf
  int iDistEA = UltEA.ping_cm();
  int iDistEB = UltEB.ping_cm();
  if (iDistEA > 30 || iDistEB > 30)
  {
    bDireccion[1] = true;
    iReturn++;
  }

  //comprueba la Izq
  int iDistIzqA = UltIA.ping_cm();
  int iDistIzqB = UltIB.ping_cm();
  if (iDistIzqB > 30 || iDistIzqB > 30)
  {
    bDireccion[2] = true;
    iRetrun++;
  }

  //comprueba atras
  int iDistAA = UltAA.ping_cm();
  int iDistAB = UltAB.ping_cm();
  if (iDistAA > 30 || iDistAB > 30)
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
  for (int iI = 0; iI < iPossibility[iCounter]; iI++)
  {
    if (bDireccion[0] == true)
    {
      iRun[iCounter][iI] = iCoord + 100;
      bDireccion[0] = false;
    }

    else if (bDireccion[1] == true)
    {
      iRun[iCounter][iI] = iCoord + 1;
      bDireccion[1] = false;
    }

    else if (bDireccion[2] == true)
    {
      iRun[iCounter][iI] = iCoord - 100;
      bDireccion[2] = false;
    }

    else if (bDireccion[3] == true)
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
  for (int iI = iCounter; iI >= 0; iI--)
  {
    if (Pos == iPos[iI])
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
  for (int iI = 0; iI < 4; iI++)
  {
    Go = BeenHere(iRun[iCounter][iI]);
    if (Go == false)
    {
      GoHere = iRun[iCounter][iI];
      break;
    }
  }
  if (GoHere == 0)
  {
    Next = true;
  }
  else
  {
    if (iPos[iCounter] + 100 == GoHere)
    {
      GiroDer90();
      delay(500);
      Adelante30();
      delay(500);
      iMovimiento = 1;
      iDireccion -= 1;
      if (iDireccion == 0)
      {
        iDireccion = 4;
      }
      setCoord(iDireccion, iMovimiento);
    }

    else if (iPos[iCounter] + 1 == GoHere)
    {
      Adelante30();
      iMovimiento = 2;
      setCoord(iDireccion, iMovimiento);
    }

    else if (iPos[iCounter] - 100 == GoHere)
    {
      GiroIzq90();
      delay(500);
      Adelante30();
      delay(500);
      iMovimiento = 3;
      iDireccion += 1;
      if (iDireccion == 5)
      {
        iDireccion = 0;
      }
      setCoord(iDireccion, iMovimiento);
    }

    else if (iPos[iCounter] - 1 == GoHere)
    {
      GiroDer90();
      delay(500);
      GiroDer90();
      delay(500;
            Adelante30();
            delay(500);
            iMovimiento = 4;
            iDireccion += 2;
            if (iDireccion == 5)
    {
      iDireccion = 1;
    }
    else if (iDireccion == 6)
    {
      iDireccion = 2;
    }
    setCoord(iDireccion, iMovimiento);
  }
}
return Next;
}

//busca y regresa el paso al cual quieres ir para moverte a una localizacion desconocida
int SearchWhereToGo(int &iReturn)
{
  int iPassToGo;
  for (int iI = iCounter; iI >= 0; iI--)
  {
    for (int iJ = 0; iJ < iPossibility[iI]; iJ++)
    {
      int iHelper = 0;
      for (int iK = iI; iK >= 0; iK--)
      {
        if (iRun[iI][iJ] == iPos[iK])
        {
          iHelper++
        }
      }
      if (iHelper = 0)
      {
        iReturn = iRun[iI][iJ];
        iPassToGo = iI;
        break;
      }
    }
    if (iReturn != 999999)
    {
      break;
    }
  }
  return iPassToGo;
}

//consigue el iPass de el iRun que le des como parametro
int GetPass(int iHere)
{
  int iReturn = 0;
  int iCopyCounter = iCounter;
  while (iHere != iPos[iCopyCounter])
  {
    iCopyCounter--;
    if (iHere == iPos[iCopyCounter])
    {
      iReturn = iPass[iCopyCounter];
    }
  }
  return iReturn;
}

int GetCoord(int iThesePass)
{
  return iPos[iThesePass];
}

MoverseShido(int iActual, int iDestination)
{
  if (iActual + 1 == iDestination)
  {
    Adelante30();
    iMovimiento = 2;
    setCoord(iDireccion, iMovimiento);
  }

  else if  (iActual - 1 == iDestination)
  {
    GiroDer90();
    delay(500);
    GiroDer90();
    delay(500;
          Adelante30();
          delay(500);
          iMovimiento = 4;
          iDireccion += 2;
          if (iDireccion == 5)
  {
    iDireccion = 1;
  }
  else if (iDireccion == 6)
  {
    iDireccion = 2;
  }
  setCoord(iDireccion, iMovimiento);
}

else if (iActual + 100 == iDestination)
  {
    GiroDer90();
    delay(500);
    Adelante30();
    delay(500);
    iMovimiento = 1;
    iDireccion -= 1;
    if (iDireccion == 0)
    {
      iDireccion = 4;
    }
    setCoord(iDireccion, iMovimiento);
  }

  else if (iActual - 100 == iDestination)
  {
    GiroIzq90();
    delay(500);
    Adelante30();
    delay(500);
    iMovimiento = 3;
    iDireccion += 1;
    if (iDireccion == 5)
    {
      iDireccion = 0;
    }
    setCoord(iDireccion, iMovimiento);
  }
}

//Ir a la posicion que necesitas ir
int Go(int &iParamter)
{
  int CopyCounter = iCounter;
  int iYouAreHere = iPass[iCopyCounter];
  int iGetHere = SearchWhereToGo(iParameter);
  int iMinor[4];
  int iProx;
  int iHelper;
  int iTemporal = 9999;

  do
  {
    for (int iI = 0; iI < iPossibility[iCopyCounter]; iI++)
    {
      iMinor[iI] = GetPass(iRun[iCounter][iI]);
      if (iMinor[iI] < iTemporal)
      {
        iTemporal = iMinor[iI];
        iHelper = iI;
      }
    }
    //contiene el ipass menor al cual se debe de mover
    iProx = iTemporal;//iMinor[iI];
    int i1 = getCoord(iYouAreHere);
    int i2 = getCoord(iProx);
    MoverseShido(i1, i2);
    YouAreHere = iProx;
    iCopyCounter = iProx;
  } while (iYouAreHere != iGetHere);
  return iYouAreHere;
}

void Laberinto()
{
  GetData();
  bool Next = Moverse();
  if (Next == true)
  {
    int iReturn = 999999;
    int iDestination = Go(iReturn);
    MoverseShido(iPos[iDestination], iReturn);
  }
}
