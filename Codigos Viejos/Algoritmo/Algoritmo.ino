//Max de arreglo de x
byte const PROGMEM XX = 40;
//Maximo de arreglo de y
byte const PROGMEM YY = 40;
//Maximo de arreglos de z
byte const PROGMEM ZZ = 3;
//Maximo de pasos posibles
byte const PROGMEM maxPasos = 100;
//Maximo de cuadros negros
byte const PROGMEM maxNegros = 20;
//Almacenara el numero de paso en la cordenada dictada por los corchetes
byte Pos[ZZ][XX][YY];
//Almacenara las posibilidades de movimiento en el paso del corchete
byte Possibility[maxPasos];
/*Almacenara las posibles coordenadas a las que se puede mover
  estara cifrado, y = zz+xx+(yy + 1); x = zz+(xx + 1)+yy;
  o = zz+(xx - 1)+yy; p = zz+xx+(yy - 1);
  r = (zz - 1) + (xx - 1) + yy; b = (zz + 1) + (xx + 1) + yy;
  s = (zz - 1) + (xx + 1) + yy; c = (zz + 1) + (xx - 1) + yy;
  t = (zz - 1) + xx + (yy + 1); d = (zz + 1) + xx + (yy + 1);
  u = (zz - 1) + xx + (yy - 1); e = (zz + 1) + xx + (yy - 1);
*/
char Run[maxPasos][4];
//Ubicación de cuadros negros
int listaX[maxNegros];
//Subindice para el areglos de listaX
byte subNegro = 0;
/*Dirección a la cual esta viendo el robot, donde 1 siempre es
  la dirección en donde empezo observando.
                1
                ^
                |
             3<- ->2
                |
                v
                4
*/
byte Direcc = 1;
//Paso en el que se quedo
byte Paso = 0;
//Paso en el que se esta
byte pasoActual = 0;
//Almacenara la coordenada actual de X
byte bX = XX / 2;
//Almacenara la coordenada actual de Y
byte bY = YY / 2;
//Almacenara la coordenada actual de Z
byte bZ = 1;
//true si en el mov anterios se cruzo una rampa
bool bARampa = false;
//Paso antes de mov de rampa
byte bPassRampa = 255;
//Paso de Rampa
byte brRampa = 255;
//Posibilidad de Run de Rampa
byte biI = 255;
//Char que va a cambiar el run antes de rampa
char cRampa = 'a';
//Char que va a cambiar el run despues de la rampa
char cRa = 'a';
//Para La rampa solo cambie datos cuando se esta explorando nuevo terreno
bool Explore = false;

bool bVictimaDetectada = false;
bool bInicio = true;

void setup()
{
  Serial.begin(9600);
  for (byte iI = 0; iI < ZZ; iI++)
  {
    for (byte iJ = 0; iJ < XX; iJ++)
    {
      for (byte iK = 0; iK < YY; iK++)
      {
        Pos[iI][iJ][iK] = 255;
      }
    }
  }

  for (byte iI = 0; iI < maxPasos; iI++)
  {
    for (byte iJ = 0; iJ < 4; iJ++)
    {
      Run[iI][iJ] = 'a';
    }
    Possibility[iI] = 255;
  }
}

//Calibrar la constante de calor
void CalibrarCalor() {} //Usar MLX

//Funciones utilizadas en el algoritmo externas a el
//Regresa true si hay una pared a la Derecha
bool ParedDer() {} //Usar Sharp y ZX
//Regresa true si hay una pared a la Izquierda
bool ParedIzq() {} //Usar Sharp y ZX
//Regresa true si hay una pared Enfrente
bool ParedEnf() {} //Usar ZX y Ultrasonico
//Regresa true si se esta sobre un cuadro negro
bool HoyoNegro() {} //Usar Sensor de Color
//Regresa el valor de Pitch que detecta el MPU
int MPUP() {} //Usar MPU
//Se mueve 30 cm hacia adelante
void Adelante30() {} //Usar Encoders, Sharps, ZX, Ultrasonicos, MPU
//Se mueve 30 cm hacia atrás
void Atras30() {} //Usar Encoders, Sharps, ZX, Ultrasonicos, MPU
//Gira 90 grados a la derecha
void GiroDer90() {} //Usar Encoders, Sharps, ZX, Ultrasonicos, MPU
//Gira 90 grados a la Izquierda
void GiroIzq90() {} //Usar Encoders, Sharps, ZX, Ultrasonicos, MPU
//Se detiene... easy game easy life
void Detenerse() {} //EZ
//Se acomoda en el cuadro en el que esta
void Acomodo() {} //Usar Encoders, Sharps, ZX, Ultrasonicos, MPU
//Detecta victimas y deja el kit
void Detectar() {} //Usar MLX
//Sube la rampa detectando si hay victima en ella
void RampaS() {} //Usar MPU, MLX
//Baja la rampa detectando si hay victima en ella
void RampaB() {} //Usar MPU, MLX
//Acomodoarse en la rampa para bajarla
void AcomodoRampa() {} 

//Regresa cuantas posibilidades tiene de mov en numero y como booleano
byte getPossibility(bool bDir[])
{
  byte bReturn = 1;
  bDir[3] = true;

  if (!ParedDer())
  {
    bReturn++;
    bDir[0] = true;
  }
  if (!ParedEnf())
  {
    bReturn++;
    bDir[1] = true;
  }
  if (!ParedIzq())
  {
    bReturn++;
    bDir[2] = true;
  }
  return bReturn;
}

//LLena los datos de la posicion donde se encuentra
void GetDatos()
{
  //Serial.println("----Datos----");
  bool bDir[4] = {false, false, false, false};
  Paso++;
  pasoActual = Paso;
  //Serial.print("Paso: ");
  //Serial.println(Paso);
  Pos[bZ][bX][bY] = Paso;
  //Serial.print("bZ: ");
  //Serial.println(bZ);
  //Serial.print("bX: ");
  //Serial.println(bX);
  //Serial.print("bY: ");
  //Serial.println(bY);
  //Serial.print("Pos[bZ][bX][bY]: ");
  //Serial.println(Pos[bZ][bX][bY]);
  Possibility[Paso] = getPossibility(bDir);
  //Serial.print("Possibility: ");
  //Serial.println(Possibility[Paso]);
  for (byte iI = 0; iI < Possibility[Paso]; iI++)
  {
    if (true == bDir[0])
    {
      if (1 == Direcc)
      {
        Run[Paso][iI] = 'x';
      }
      else if (2 == Direcc)
      {
        Run[Paso][iI] = 'p';
      }
      else if (3 == Direcc)
      {
        Run[Paso][iI] = 'y';
      }
      else if (4 == Direcc)
      {
        Run[Paso][iI] = 'o';
      }
      bDir[0] = false;
    }
    else if (true == bDir[1])
    {
      if (1 == Direcc)
      {
        Run[Paso][iI] = 'y';
      }
      else if (2 == Direcc)
      {
        Run[Paso][iI] = 'x';
      }
      else if (3 == Direcc)
      {
        Run[Paso][iI] = 'o';
      }
      else if (4 == Direcc)
      {
        Run[Paso][iI] = 'p';
      }
      bDir[1] = false;
    }
    else if (true == bDir[2])
    {
      if (1 == Direcc)
      {
        Run[Paso][iI] = 'o';
      }
      else if (2 == Direcc)
      {
        Run[Paso][iI] = 'y';
      }
      else if (3 == Direcc)
      {
        Run[Paso][iI] = 'p';
      }
      else if (4 == Direcc)
      {
        Run[Paso][iI] = 'x';
      }
      bDir[2] = false;
    }
    else if (true == bDir[3])
    {
      if (1 == Direcc)
      {
        Run[Paso][iI] = 'p';
      }
      else if (2 == Direcc)
      {
        Run[Paso][iI] = 'o';
      }
      else if (3 == Direcc)
      {
        Run[Paso][iI] = 'x';
      }
      else if (4 == Direcc)
      {
        Run[Paso][iI] = 'y';
      }
      bDir[3] = false;
    }
  }
  if (true == bARampa)
  {
    if ('a' != Run[Paso][3])
    {
      Run[Paso][3] = cRa;
    }
    else if ('a' != Run[Paso][2])
    {
      Run[Paso][2] = cRa;
    }
    else if ('a' != Run[Paso][1])
    {
      Run[Paso][1] = cRa;
    }
    else if ('a' != Run[Paso][0])
    {
      Run[Paso][0] = cRa;
    }
    bARampa = false;
  }
  for (byte iI = 0; iI < maxNegros; iI++)
  {
    for (byte iJ = 0; iJ < 4; iJ++)
    {
      int iCoord = getCoord(Run[Paso][iJ], Paso);
      if (iCoord == listaX[iI])
      {
        for (byte iK = iJ; iK < 4; iK++)
        {
          Run[Paso][iK] = Run[Paso][1 + iK];
        }
        Run[Paso][3] = 'a';
      }
    }
  }
  ////Serial.println("----Datos----");
  ////Serial.print("bZ: ");
  ////Serial.println(bZ);
  ////Serial.print("bX: ");
  ////Serial.println(bX);
  ////Serial.print("bY: ");
  ////Serial.println(bY);
  ////Serial.print("Pos: ");
  ////Serial.println(Pos[bZ][bX][bY]);
  ////Serial.print("Possibility: ");
  ////Serial.println(Possibility[Paso]);
  //Serial.print("Run 1: ");
  //Serial.println(Run[Paso][0]);
  //Serial.print("Run 2: ");
  //Serial.println(Run[Paso][1]);
  //Serial.print("Run 3: ");
  //Serial.println(Run[Paso][2]);
  //Serial.print("Run 4: ");
  //Serial.println(Run[Paso][3]);
}

/*Regresa true si ya se ha estado antes en el Run que se le manda
  como parametro, cRun, y false si no
*/
bool BeenHere(char cRun, byte iPaso)
{
  //Variables que almacenan lo que se va a regresar, X, Y y Z del Paso que se investiga
  bool bReturn = false;
  byte iX = 255;
  byte iY = 255;
  byte iZ = 255;

  //Busca en todos los pasos el iPaso para conseguir X, Y y Z
  for (byte iI = 0; iI < ZZ; iI++)
  {
    for (byte iJ = 0; iJ < XX; iJ++)
    {
      for (byte iK = 0; iK < YY; iK++)
      {
        if (iPaso == Pos[iI][iJ][iK])
        {
          iZ = iI;
          iX = iJ;
          iY = iK;
        }
        if (255 != iX)
          break;
      }
      if (255 != iX)
        break;
    }
    if (255 != iX)
      break;
  }
  //Modifica X o Y dependiendo del cRun
  if ('x' == cRun)
  {
    iX += 1;
  }
  else if ('y' == cRun)
  {
    iY += 1;
  }
  else if ('o' == cRun)
  {
    iX -= 1;
  }
  else if ('p' == cRun)
  {
    iY -= 1;
  }
  else if ('r' == cRun)
  {
    iZ -= 1;
    iX -= 1;
  }
  else if ('s' == cRun)
  {
    iZ -= 1;
    iX += 1;
  }
  else if ('t' == cRun)
  {
    iZ -= 1;
    iY += 1;
  }
  else if ('u' == cRun)
  {
    iZ -= 1;
    iY -= 1;
  }
  else if ('e' == cRun)
  {
    iZ += 1;
    iY -= 1;
  }
  else if ('b' == cRun)
  {
    iZ += 1;
    iX += 1;
  }
  else if ('d' == cRun)
  {
    iZ += 1;
    iY += 1;
  }
  else if ('c' == cRun)
  {
    iZ += 1;
    iX -= 1;
  }
  //Busca si ya se ha estado en esa coordenada
  if (255 != Pos[iZ][iX][iY])
  {
    bReturn = true;
  }
  return bReturn;
}

//Consgiue el numero de paso del Run que se da como parametro
byte getPass(char cRun, byte iPaso)
{
  byte bReturn = 255;
  byte iX = 255;
  byte iY = 255;
  byte iZ = 255;
  //Busca en todos los pasos el iPaso para conseguir X, Y y Z
  for (byte iI = 0; iI < ZZ; iI++)
  {
    for (byte iJ = 0; iJ < XX; iJ++)
    {
      for (byte iK = 0; iK < YY; iK++)
      {
        if (iPaso == Pos[iI][iJ][iK])
        {
          iZ = iI;
          iX = iJ;
          iY = iK;
        }
        if (255 != iX)
          break;
      }
      if (255 != iX)
        break;
    }
    if (255 != iX)
      break;
  }
  //Modifica X o Y dependiendo del cRun
  if ('x' == cRun)
  {
    iX += 1;
  }
  else if ('y' == cRun)
  {
    iY += 1;
  }
  else if ('o' == cRun)
  {
    iX -= 1;
  }
  else if ('p' == cRun)
  {
    iY -= 1;
  }
  else if ('r' == cRun)
  {
    iZ -= 1;
    iX -= 1;
  }
  else if ('s' == cRun)
  {
    iZ -= 1;
    iX += 1;
  }
  else if ('t' == cRun)
  {
    iZ -= 1;
    iY += 1;
  }
  else if ('u' == cRun)
  {
    iZ -= 1;
    iY -= 1;
  }
  else if ('e' == cRun)
  {
    iZ += 1;
    iY -= 1;
  }
  else if ('b' == cRun)
  {
    iZ += 1;
    iX += 1;
  }
  else if ('d' == cRun)
  {
    iZ += 1;
    iY += 1;
  }
  else if ('c' == cRun)
  {
    iZ += 1;
    iX -= 1;
  }
  bReturn = Pos[iZ][iX][iY];
  return bReturn;
}

int getCoord(char cRun, byte iPaso)
{
  int iReturn = 255;
  byte iX = 255;
  byte iY = 255;
  byte iZ = 255;
  //Busca en todos los pasos el iPaso para conseguir X, Y y Z
  for (byte iI = 0; iI < ZZ; iI++)
  {
    for (byte iJ = 0; iJ < XX; iJ++)
    {
      for (byte iK = 0; iK < YY; iK++)
      {
        if (iPaso == Pos[iI][iJ][iK])
        {
          iZ = iI;
          iX = iJ;
          iY = iK;
        }
        if (255 != iX)
          break;
      }
      if (255 != iX)
        break;
    }
    if (255 != iX)
      break;
  }
  //Modifica X o Y dependiendo del cRun
  if ('x' == cRun)
  {
    iX += 1;
  }
  else if ('y' == cRun)
  {
    iY += 1;
  }
  else if ('o' == cRun)
  {
    iX -= 1;
  }
  else if ('p' == cRun)
  {
    iY -= 1;
  }
  else if ('r' == cRun)
  {
    iZ -= 1;
    iX -= 1;
  }
  else if ('s' == cRun)
  {
    iZ -= 1;
    iX += 1;
  }
  else if ('t' == cRun)
  {
    iZ -= 1;
    iY += 1;
  }
  else if ('u' == cRun)
  {
    iZ -= 1;
    iY -= 1;
  }
  else if ('e' == cRun)
  {
    iZ += 1;
    iY -= 1;
  }
  else if ('b' == cRun)
  {
    iZ += 1;
    iX += 1;
  }
  else if ('d' == cRun)
  {
    iZ += 1;
    iY += 1;
  }
  else if ('c' == cRun)
  {
    iZ += 1;
    iX -= 1;
  }
  iReturn = (iZ * 10000) + (iX * 100) + iY;
  return iReturn;
}

//Regresa true si se esta en una rampa
bool Rampa()
{
  bool bReturn = false;
  int iMPUP = MPUP();
  if (iMPUP > 4000 || iMPUP < -2500)
  {
    bReturn = true;
  }
  return bReturn;
}

//Cruza la rampa, ACTUALIZAR bZ aqui!!
void MovRampa()
{
  //Serial.println("----Rampa----");
  //Serial.print("brRampa, paso: ");
  //Serial.println(brRampa);
  //Serial.print("biI, run: ");
  //Serial.println(biI);
  //Serial.println("Run[brRampa][biI]");
  //Serial.print("De: ");
  //Serial.println(Run[brRampa][biI]);
  ////Serial.print("bZ: ");
  ////Serial.println(bZ);
  int iMPUP = MPUP();
  //Si detecta que hay que subir la rampa
  if (iMPUP > 4000)
  {
    if (true == Explore)
    {
      if ('x' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 'b';
        cRa = 'r';
      }
      else if ('y' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 'd';
        cRa = 'u';
      }
      else if ('o' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 'c';
        cRa = 's';
      }
      else if ('p' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 'e';
        cRa = 't';
      }
    }
    Atras30();
    Acomodo();
    GiroDer90();
    Acomodo();
    GiroDer90();
    Acomodo();
    Atras30();
    Atras30();
    //AcomodoRampa();
    RampaS();
    Detenerse();
    GiroDer90();
    Acomodo();
    GiroDer90();
    Acomodo();
    bZ += 1;
  }
  //Si esta bajando la rampa
  else if (iMPUP < -2000)
  {
    if (true == Explore)
    {
      if ('x' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 's';
        cRa = 'c';
      }
      else if ('y' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 't';
        cRa = 'e';
      }
      else if ('o' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 'r';
        cRa = 'b';
      }
      else if ('p' == Run[brRampa][biI])
      {
        Run[brRampa][biI] = 'u';
        cRa = 'd';
      }
    }
    AcomodoRampa();
    RampaB();
    Detenerse();
    bZ -= 1;
  }
  //Serial.print("a: ");
  //Serial.println(Run[brRampa][biI]);
}

//Se mueve de la coordenada actuar(iCoordAc) a la coordenada deseada(icCoord)
void Move(int iCoordAc, int icCoord)
{
  //Serial.println("---Move---");
  //Serial.print("Coordenada Actual: ");
  //Serial.println(iCoordAc);
  //Serial.print("Coordenada deseada: ");
  //Serial.println(icCoord);
  //Copia de iCoordAc para la rampa
  int CopCoordAc = iCoordAc;
  //Copia de icCoord para los hoyos negros
  int CopcCoord = icCoord;

  //Se quita el dato del piso de la coordenada actual y a la que se quiere llegar
  iCoordAc = iCoordAc % 10000;
  icCoord = icCoord % 10000;

  //Copias de X, Y y Z
  byte bCX = bX;
  byte bCY = bY;
  byte bCZ = bZ;

  //Busca a que dirección moverse actualiza la variable correcta de X oY y Direcc
  if (iCoordAc == icCoord - 100)
  {
    if (Direcc == 1)
    {
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 2)
    {
      Adelante30();
    }
    else if (Direcc == 3)
    {
      GiroDer90();
      Acomodo();
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 4)
    {
      GiroIzq90();
      Acomodo();
      Adelante30();
    }
    bX += 1;
    Direcc = 2;
  }
  else if (iCoordAc == icCoord - 1)
  {
    if (Direcc == 1)
    {
      Adelante30();
    }
    else if ( Direcc == 2)
    {
      GiroIzq90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 3)
    {
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 4)
    {
      GiroDer90();
      Acomodo();
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    bY += 1;
    Direcc = 1;
  }
  else if (iCoordAc == icCoord + 100)
  {
    if (Direcc == 1)
    {
      GiroIzq90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 2)
    {
      GiroDer90();
      Acomodo();
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 3)
    {
      Adelante30();
    }
    else if (Direcc == 4)
    {
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    bX -= 1;
    Direcc = 3;
  }
  else if (iCoordAc == icCoord + 1)
  {
    if (Direcc == 1)
    {
      GiroDer90();
      Acomodo();
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 2)
    {
      GiroDer90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 3)
    {
      GiroIzq90();
      Acomodo();
      Adelante30();
    }
    else if (Direcc == 4)
    {
      Adelante30();
    }
    bY -= 1;
    Direcc = 4;
  }
  Detenerse();
  //delay(100);
  //si esa nueva coordenada es un hoyo negro regresa y busca a donde moverse
  if (HoyoNegro())
  {
    listaX[subNegro] = CopcCoord;
    subNegro += 1;
    ////Serial.println("----Entra HoyoNegro----");
    bool bListo = false;
    Atras30();
    Acomodo();
    bX = bCX;
    bY = bCY;
    byte iCPaso = Pos[bZ][bX][bY];
    ////Serial.print("bZ: ");
    ////Serial.println(bZ);
    ////Serial.print("bX: ");
    ////Serial.println(bX);
    ////Serial.print("bY: ");
    ////Serial.println(bY);
    ////Serial.print("Pos[bZ][bX][bY]= ");
    ////Serial.println(Pos[bZ][bX][bY]);
    for (byte iI = 0; iI < Possibility[iCPaso]; iI++)
    {
      ////Serial.print("icCoord: ");
      ////Serial.println(icCoord);
      int iThis = getCoord(Run[iCPaso][iI], iCPaso);
      ////Serial.print("iThis #");
      ////Serial.print(iI);
      ////Serial.print(": ");
      ////Serial.println(getCoord(Run[iCPaso][iI], iCPaso));
      if (iThis == CopcCoord)
      {
        for (byte iJ = iI; iJ < 4; iJ++)
        {
          Run[iCPaso][iJ] = Run[iCPaso][1 + iJ];
          bListo = true;
        }
      }
      if (bListo == true)
      {
        break;
      }
    }
    Run[iCPaso][3] = 'a';
    Possibility[iCPaso] -= 1;
    pasoActual = iCPaso;
    ////Serial.print("iCPaso: ");
    ////Serial.println(iCPaso);
    ////Serial.print("Run 1: ");
    ////Serial.println(Run[iCPaso][0]);
    ////Serial.print("Run 2: ");
    ////Serial.println(Run[iCPaso][1]);
    ////Serial.print("Run 3: ");
    ////Serial.println(Run[iCPaso][2]);
    ////Serial.print("Run 4: ");
    ////Serial.println(Run[iCPaso][3]);
    ////Serial.print("Possbility: ");
    ////Serial.println(Possibility[iCPaso]);
    SearchRouteAndMove();
  }
  //Si esa nueva coordenada es la rampa, la cruza y actualiza bZ
  else if (Rampa())
  {
    MovRampa();
    bARampa = true;
  }
  Acomodo();
}


//Se mueve hasta el paso que se tiene como parametro
void moveUntil(byte bHere)
{
  //Serial.println("---MoveUntil---");
  //Serial.print("Ir hasta el paso #");
  //Serial.println(bHere);
  byte counter = 0;
  //copia del paso actual
  byte bCPaso = pasoActual;
  //Hasta no encontrarse en el paso deseado se sigue moviendo
  while (bCPaso != bHere)
  {
    /*variables, paso más cercano, posible paso más cercano,
      Paso antiguo, Paso, Coordenada Actual, Coordenada deseada, posible
      coordenada deseada
    */
    byte bClose = 255;
    byte bPosib = 255;
    byte bPassAntiguo = 255;
    int iPass = 255;
    int iCoordAc = 255;
    int icCoord = 255;
    /*Primero revisa los Pasos del los Run de donde te encuentras
      y escoge el Paso más cercano al bHere
    */
    for (byte iI = 0; iI < Possibility[bCPaso]; iI++)
    {
      bPosib = getPass(Run[bCPaso][iI], bCPaso);
      iPass = bPosib - bHere;
      if (iPass < 0)
      {
        iPass *= -1;
      }
      if (iPass < bPassAntiguo)
      {
        bPassAntiguo = iPass;
        bClose = bPosib;
        icCoord = getCoord(Run[bCPaso][iI], bCPaso);
      }
    }
    iCoordAc = (bZ * 10000) + (bX * 100) + bY;
    Move(iCoordAc, icCoord);
    bCPaso = bClose;
    counter++;
    if (counter > 0)
    {
      bARampa = false;
    }
  }
}

//Regresa el punto de inicio
void extractionPoint()
{
  moveUntil(1);
  Detenerse();
  delay(30000);
}

//Se mueve a la coordenada desconocida
void exploreNewWorlds(byte bHere)
{
  Explore = true;
  //Serial.println("----exploreNewWorlds----");
  //Serial.print("Se encuentra en Paso: ");
  //Serial.println(bHere);
  byte iCounter = 0;
  for (byte iI = 0; iI < Possibility[bHere]; iI++)
  {
    int iCoordAc = 9999;
    int icCoord = 9999;
    bool bCheck = BeenHere(Run[bHere][iI], bHere);
    if (bCheck == false && iCounter == 0)
    {
      //Serial.print("No ha estado en: ");
      //Serial.println(getCoord(Run[bHere][iI], bHere));
      //Serial.print("Del Paso: ");
      //Serial.println(bHere);
      //Serial.print("Posibilidad #");
      //Serial.println(iI);
      icCoord = getCoord(Run[bHere][iI], bHere);
      brRampa = bHere;
      biI = iI;
      for (byte iI = 0; iI < ZZ; iI++)
      {
        for (byte iJ = 0; iJ < XX; iJ++)
        {
          for (byte iK = 0; iK < YY; iK++)
          {
            if (Pos[iI][iJ][iK] == bHere)
            {
              iCoordAc = (iI * 10000) + (iJ * 100) + iK;
            }
            if (iCoordAc != 9999)
            {
              break;
            }
          }
          if (iCoordAc != 9999)
          {
            break;
          }
        }
        if (iCoordAc != 9999)
        {
          break;
        }
      }
      ////Serial.print("Coordenada Actual: ");
      ////Serial.println(iCoordAc);
      ////Serial.print("Coordenada Deseada: ");
      ////Serial.println(icCoord);
      Move(iCoordAc, icCoord);
      iCounter++;
    }
  }
  Explore = false;
}

//Busca el paso al cual llegar para despues moverse a la coordenada desconocida
byte WhereToGo()
{
  //Serial.println("---WhereToGo---");
  //copia del paso actual, de x y de y
  byte bCPaso = pasoActual;
  byte bCX = bX;
  byte bCY = bY;
  //paso al cual llegar
  byte bHere = 255;

  //busca la coordendada desconocida por todos los Run concidos
  for (byte iI = bCPaso; iI > 0; iI--)
  {
    for (byte iJ = 0; iJ < Possibility[iI]; iJ++)
    {
      if (!BeenHere(Run[iI][iJ], iI))
      {
        bool Revision = false;
        for (byte iK = 0; iK < maxNegros; iK++)
        {
          int coordenada = getCoord(Run[iI][iJ], iI);
          if (listaX[iK] == coordenada)
          {
            Revision = true;
          }
        }
        if (Revision == false)
        {

          bHere = iI;
          //Serial.print("No ha estado en un Run del paso #");
          //Serial.println(iI);
          //Serial.print("Numero de Run: ");
          //Serial.println(iJ);
          //Serial.print("Coordenada: ");
          //Serial.println(getCoord(Run[iI][iJ], iI));
        }
      }
      if (255 != bHere)
        break;
    }
    if (255 != bHere)
      break;
  }
  //Significa que se ha acabado de recorrer el laberinto
  if (255 == bHere)
  {
    extractionPoint();
  }
  return bHere;
}


//Busca la ruta y se mueve hasta la posicion desconocida
void SearchRouteAndMove()
{
  //Serial.println("---SearchAndMove---");
  byte bData = WhereToGo();
  moveUntil(bData);
  exploreNewWorlds(bData);
}

//Funcion a llamar para completar el laberinto
void Laberinto()
{
  //Serial.println("------Laberinto-------");
  GetDatos();
  SearchRouteAndMove();
}
