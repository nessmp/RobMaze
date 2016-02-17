int iPass[200];
int iPos[200];
int iPossibility[200];
int iRun[200][4];
int iRunHelper[4];
int iCoord = 2020;
int iHelper = 0;
int iCopyHelper = iHelper;

//regresa numero de posibles mov
int GetPosib(int iRunHelper[])
{
  int iPosib = 0;

  //Para la derecha
  int iDist = UltDA.ping_cm();
  int iDist2 = UltDB.ping_cm();
  if(iDist > 30 || iDIst 2 > 30)
  {
    iRunHelper[iPosib] = iCoord + 100;
    iPosib++
  }
  
  //Para Enfrente
  iDist = UltEA.ping_cm();
  iDist2 = UltEB.ping_cm();
  if(iDist > 30 || iDIst 2 > 30)
  {
    iRunHelper[iPosib] = iCoord + 1;
    iPosib++
  }

  //Para Izqu
  iDist = UltIA.ping_cm();
  iDist2 = UltIB.ping_cm();
  if(iDist > 30 || iDIst 2 > 30)
  {
    iRunHelper[iPosib] = iCoord - 100;
    iPosib++
  }

  //Para Atras
  iDist = UltAA.ping_cm();
  iDist2 = UltAB.ping_cm();
  if(iDist > 30 || iDIst 2 > 30)
  {
    iRunHelper[iPosib] = iCoord - 1;
    iPosib++
  }

  return iPosib;
}

void GetData()
{
  iPass[iHelper] = iHelper;
  iPos[iHelper] = iCoord;
  iPossibility [iHelper] = GetPosib(iRunHelper);
  for(int iI = 0; iI < iPossibility[iHelper]; iI++)
  {
   iRun[iHelper][iI] =  iRunHelper[iI]
  }
}

bool DoneThat(int iX)
{
  bool BeenHere = false;
  for(int iI = iHelper; iI >= 0; iI--)
  {
    if(iX == iPos[iI])
    {
      BeenHere = true;
    }
  }
  return BeenHere;
}

int Where()
{
  iCopyHelper = iHelper;
  int iHere = 99999;
  while(iHere = 99999)
  {
    for(int iI = 0; iI < iPossibility[iCopyHelper]; iI++)
    {
      int iX = iRun[iCopyHelper][iI]
      bool BeenHere = DoneThat(iX);
      if(BeenHere == false)
      {
        iHere = iPass[iCopyHelper];
        break;
      }
    }
    if(iHere == 99999)
    {
      IrARampa();
      break;
    }
  }
  return iHere;
}

int GetIPass(int iX)
{
  int iThisPass;
  for(int iI = iHelper; iI >= 0; iI--)
  {
    if(iX == iPos[iI])
    {
      iThisPass = iPass[iI];
    }
  }
  return iThisPass;
}

int GetThere()
{
  iCopyHelper = iHelper;
  int iPosition = iPos[iCopyHelper];
  int iObjective = iWhere();
  while(iPosition != iObjective)
  {
    int iX = iRun[iPos][0]
    GetIPass(iX);
    if(iRun[
  }
}

