int iCoord[40][40];

for(int iI = 0; iI < 40; iI++)
{
	for(int iJ = 0; iJ < 40; iJ++)
	{
		iCoord[iI][iJ] = 99999;
	}
}

int iPoss[40];

for(int iJ = 0; iJ < 40; iJ++)
{
	iPoss[iJ] = 99999;
}

int iRun[40][4];

for(int iI = 0; iI < 40; iI++)
{
	for(int iJ = 0; iJ < 4; iJ++)
	{
		iRun[iI][iJ] = 99999;
	}
}

int iX = 20;
int iY = 20;

int iPaso = -1

int iDireccion = 2;

void Derecha90()
{
	iDireccion -= 1;
	if(iDireccion == 0)
	{
		iDireccion = 4;
	}
}

void Izquierda90()
{
	iDireccion += 1;
	if(iDireccion == 5)
	{
		iDireccion = 1;
	}
}

void Adelante30()
{
	if(iDireccion == 1)
	{
		iX += 1;
	}
	else if(iDireccion == 2)
	{
		iY += 1;
	}
	else if(iDireccion == 3)
	{
		iX -= 1;
	}
	else if(iDireccion == 4)
	{
		iY -= 1;
	}
}

int getPoss(bool Direcc[])
{
	int iReturn = 0;

	if(ParedDer())
	{
		iReturn++;
		Direcc[0] = true;
	}

	if(ParedEnf())
	{
		iReturn++;
		Direcc[1] = true;
	}

	if(ParedIzq())
	{
		iReturn++;
		Direcc[2] = true;
	}

	if(ParedAtr())
	{
		iReturn++;
		Direcc[3] = true;
	}

	return iReturn;
}

int setRun(int Direcc)
{
	int iX2 = 0;
	int iY2 = 0;

	if(iDireccion == 1)
	{
		if(Direcc == 0)
		{
			iX2 = iX;
			iY2 = iY - 1;
		}
		else if(Direcc == 1)
		{
			iX2 = iX + 1;
			iY2 = iY;
		}
		else if(Direcc == 2)
		{
			iX2 = iX;
			iY2 = iY + 1;
		}
		else if(Direcc == 3)
		{
			iX2 = iX - 1;
			iY2 = iY;
		}
	}

	else if(iDireccion == 2)
	{
		if(Direcc == 0)
		{
			iX2 = iX + 1;
			iY2 = iY;
		}
		else if(Direcc == 1)
		{
			iX2 = iX;
			iY2 = iY + 1;
		}
		else if(Direcc == 2)
		{
			iX2 = iX - 1;
			iY2 = iY;
		}
		else if(Direcc == 3)
		{
			iX2 = iX;
			iY2 = iY - 1;
		}
	}

	else if(iDireccion == 3)
	{
		if(Direcc == 0)
		{
			iX2 = iX;
			iY2 = iY + 1;
		}
		else if(Direcc == 1)
		{
			iX2 = iX - 1;
			iY2 = iY;
		}
		else if(Direcc == 2)
		{
			iX2 = iX;
			iY2 = iY - 1;
		}
		else if(Direcc == 3)
		{
			iX2 = iX + 1;
			iY2 = iY;
		}
	}

	else if(iDireccion == 4)
	{
		if(Direcc == 0)
		{
			iX2 = iX - 1;
			iY2 = iY;
		}
		else if(Direcc == 1)
		{
			iX2 = iX;
			iY2 = iY - 1;
		}
		else if(Direcc == 2)
		{
			iX2 = iX + 1;
			iY2 = iY;
		}
		else if(Direcc == 3)
		{
			iX2 = iX;
			iY2 = iY + 1;
		}
	}
	int iReturn = (iX2 * 100) + iY2;

	return iReturn;
}

void GetData()
{
	iPaso++;
	bool Direcc[4] = {false, false, false, false}
	iCoord [iX][iY] = iPaso
	iPoss[iPaso] = getPoss(Direcc);
	for(int iI = 0; iI < iPoss[iPaso]; iI++)
	{
		if(Direcc[0] == true)
		{
			iRun[iPaso][iI] == setRun(0);
			Driecc[0] = false;
		}
		else if(Direcc[1] == true)
		{
			iRun[iPaso][iI] == setRun(1);
			Driecc[1] = false;
		}
		else if(Direcc[2] == true)
		{
			iRun[iPaso][iI] == setRun(2);
			Driecc[2] = false;
		}
		else if(Direcc[3] == true)
		{
			iRun[iPaso][iI] == setRun(3);
			Driecc[3] = false;
		}
	}
}

void GetPass()
{
	int iX2 = 0;
	int iY2 = 0;
	int a,b,c,d,e;
	int iPaso2 = iPaso;
	int iReturn = 0;
	while(iReturn == 0)
	{
		for(int iI = 0; iI < iPoss[iPaso2]; iI++)
		{  
			b = (iRun[iPaso2][iI]% 10000) / 1000;
			c = (iRun[iPaso2][iI] % 10000) % 1000 / 100;
			d = ((iRun[iPaso2][iI] % 10000) % 1000) % 100 / 10;
			e = (((iRun[iPaso2][iI] % 10000) % 1000) % 100) % 10;
			iX2 = (b*10) + c;
			iY2 = (d*10) + e;

			if(iCoord[iX2][iY2] == 99999)
			{
				iReturn = iPaso2;
				break;
			}
			iPaso2--;
		}
	}
	return iReturn;
}

void MoversePasoCercano(int iX2, int iY2, int iDestinaiton)
{
	bool Listo = flase;
	int Paso = iCoord[iX2][iX1]
	while(Listo == false)
	{
		int iMenor
		for(int iI = 0; iI < Paso; iI++)
		{
			iRun[Paso][iI]
		}
	}
}

void GetToPass()
{
	int iDestination = GetPass();
	int iPosition = (iX * 100) + iY
	int iX2 = iX;
	int iY2 = iY;
	while(iDestination != iPosition)
	{
		MoversePasoCercano(iX2, iY2m int iDestination);
	}
}

void Laberinto()
{

}