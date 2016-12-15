// PRUEBAS DE REVOLUCION LABERINTO

//<-----------Libraries----------------------------->
#include <NewPing.h>
#include <math.h>

//<-------------------------Encoders--------------------->
#define encoderI 2

volatile int count;

byte colorSensor = A5;
bool bForward = true;
bool bCorrection = false;
//const short lenBetUltrasonic = 10.0; // cm
const int giroDerecha = 1500; // el valor que los encoders necesitan para girar 90° a la derecha
const int giroIzquierda = 1500; // el valor que los encoders necesitan para girar 90° a la izquierda
short voltage[]={90,102,113,123,136,148,169,190,215,255,315,410,525};
short distance[]={140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 20};

float diffLeft = 0, diffRight = 0;
double tempIR = 0;

int arrInclination[32], arrIncMean[64]; // Acelerometro
int inclinationY = 0;
#define maxDistance 15
#define maxInclinationDistance 1

//<----------------------------Pins------------------------>
//declaring ultrasonic pins 
// LEFT BACK
#define trigPin1 13
#define echoPin1 12

// LEFT FRONT
#define trigPin2 A2
#define echoPin2 A3

// FRONT LEFT
#define trigPin3 A0
#define echoPin3 A1

// FRONT RIGHT
#define trigPin4 A11
#define echoPin4 A10

// RIGHT FRONT
#define trigPin5 51
#define echoPin5 53

// RIGHT BACK
#define trigPin6 31
#define echoPin6 33

/*		   FRONT
 	 ________________	
	|  U3		U4  |
	|U2		  U5|	
	|		    |
	|U1		  U6|
	|_________________  |

		  BACK 
**/

// FALTAN LOS SHARP



//<----------------------------Global Variables------------------------>

//storing distance of every ultrasonic sensor
double arrUltra1[]= {0, 0, 0};
double arrUltra2[]= {0, 0, 0};
double arrUltra3[]= {0, 0, 0};
double arrUltra4[]= {0, 0, 0};
double arrUltra5[]= {0, 0, 0};
double arrUltra6[]= {0, 0, 0};
short arrIrFront[]= {0, 0, 0, 0, 0, 0, 0, 0};
short arrIrBack[]= {0, 0, 0, 0, 0, 0, 0, 0};

// EL ARREGLO arrDist[] guarda de la siguiente manera las distancias {U1, U2, U3, U4, U5, U6}
double arrDist[] = {0,0,0,0,0,0};

// El arreglo guarda las distancias de los IR
short arrIR[] = {0,0};
// IR variables
bool initialIR =true;

//<--------------------------Motors------------------------------->
//declaring motors pins motorRightFrontForward


const int BackLeftFor = 11;
const int BackLeftBack = 10;

const int BackRightFor = 5;
const int BackRightBack = 4;


const int FrontLeftBack = 8;
const int FrontLeftFor = 9;

const int FrontRightBack = 7;
const int FrontRightFor = 6;

long int t; // tiempo para delays

//<---------------Interruption---------------------->
void handleEncoder(){
  if(digitalRead(encoderI) == HIGH)
           count++;
 }

//<----------------------------Motors------------------------>

void rampUp(){
	

    count = 0;
    while(count < 3000){ 
  
    analogWrite(FrontLeftFor, 255);
    analogWrite(BackLeftFor, 255);
    analogWrite(BackRightFor, 255);
    analogWrite(FrontRightFor, 255); 
    }
    readDistFront();
  while(arrDist[2] == 0 && arrDist[3] == 0){
        readDistFront();
      analogWrite(FrontLeftFor, 255);
      analogWrite(BackLeftFor, 255);
      analogWrite(BackRightFor, 255);
      analogWrite(FrontRightFor, 255); 

  
  }
	/*while(true){
    readInclination();
    
    if(inclinationY>335){
		  analogWrite(FrontLeftFor, 255);
    	analogWrite(BackLeftFor, 255);
    	analogWrite(BackRightFor, 255);
    	analogWrite(FrontRightFor, 255); 
    }
    else
    break;
	}**/

  stopMotors();
  turnRight();

}

void rampDown(){

  count = 0;
    while(count < 3000){ 
  
      analogWrite(FrontLeftFor, 175);
      analogWrite(BackLeftFor, 175);
      analogWrite(BackRightFor, 175);
      analogWrite(FrontRightFor, 175); 
    }
	readDistFront();

	while(arrDist[2] == 0 || arrDist[3] == 0) {
		analogWrite(FrontLeftFor, 175);
    	analogWrite(BackLeftFor, 175);
    	analogWrite(BackRightFor, 175);
    	analogWrite(FrontRightFor, 175); 

    	readDistFront();
	}

  stopMotors();
  turnLeft();
}

void forward(){
 
 	count = 0;
  	while(count < 1550){ 
  
    analogWrite(FrontLeftFor, 255);
    analogWrite(BackLeftFor, 255);
    analogWrite(BackRightFor, 245);
    analogWrite(FrontRightFor, 245); 
  	}
 
 	stopMotors();

  movementCorrection();
  delay(10);
  
 	readInclination();
 	if(inclinationY > 365)
 		rampUp();
 	else if(inclinationY < 330)
 		rampDown();

   
 	}

  /*void forwardCorr(){
    readInclination();
    int blackTile, tempInc = inclinationY;
  	readDistance();
  	int tempCount = 0;
  	if(arrDist[0] != 0 && arrDist[1] != 0){
  		while(count < 1550){ 
  
	    analogWrite(FrontLeftFor, 255);
	    analogWrite(BackLeftFor, 255);
	    analogWrite(BackRightFor, 245);
	    analogWrite(FrontRightFor, 245); 

	    tempCount = count;
	    readDistLeft();
	    movementCorrectionLeft();

	    count = tempCount;
  		}

  		stopMotors();
 		readInclination();
 	
 		if(inclinationY - tempInc > 30)
 			rampUp();
 		else if(tempInc - inclinationY > 30)
 			rampDown();

   		movementCorrection();

  	}
  	else if(arrDist[4] != 0 && arrDist[5] != 0){
  		while(count < 1550){ 
  
	    analogWrite(FrontLeftFor, 255);
	    analogWrite(BackLeftFor, 255);
	    analogWrite(BackRightFor, 245);
	    analogWrite(FrontRightFor, 245); 

	    tempCount = count;
	    readDistRight();
	    movementCorrectionRight();

	    count = tempCount;
  		}

  		stopMotors();
	 	readInclination();
	 	
	 	if(inclinationY - tempInc > 30)
	 		rampUp();
	 	else if(tempInc - inclinationY > 30)
	 		rampDown();

	   	movementCorrection();

  	}
  	else
  		forward();
  	
  }
**/

void back(){
  count = 0; 
  while(count < 1550){
     analogWrite(FrontLeftBack, 210);
     analogWrite(BackLeftBack, 220);
     analogWrite(FrontRightBack, 200);
     analogWrite(BackRightBack, 200);
   }
  
  stopMotors();

  turnRight();
}

void slideRight(){
	count = 0;
        stopMotors();
  	while(count < 250){
    	analogWrite(FrontLeftFor, 255);
    	analogWrite(BackLeftBack, 255);
    	analogWrite(BackRightFor, 255); 
    	analogWrite(FrontRightBack, 255); 
  	}
 
 	stopMotors();
       delay(50);
}

void slideLeft(){
	count = 0;
        stopMotors();
  	while(count < 250){
    
    	analogWrite(FrontLeftBack, 255);
    	analogWrite(BackLeftFor, 255);
    	analogWrite(BackRightBack, 255);
    	analogWrite(FrontRightFor, 255);
    
  	}
 
 	stopMotors();
       delay(25);
}

void turnLeft(int iRot){
	count=0;
        stopMotors();
 	while(count < iRot){  // recorrido necesario para la vuelta
	 analogWrite(FrontRightFor, 220);
   analogWrite(BackRightFor, 220);
    analogWrite(BackLeftBack, 220);
    analogWrite(FrontLeftBack, 220);
 	}
 	
 	stopMotors();
	delay(25);
}

void turnRight(int iRot){
	count=0;
        stopMotors();
 	while(count < iRot){  // recorrido necesario para la vuelta
    analogWrite(FrontRightBack, 220);
    analogWrite(BackRightBack, 220);
    analogWrite(BackLeftFor, 220);
    analogWrite(FrontLeftFor, 220);
 	}
 	
 	stopMotors();
	delay(25);
}

void back(byte temp){
  count = 0; 
  while(count < temp){
     analogWrite(FrontLeftBack, 210);
     analogWrite(BackLeftBack, 220);
     analogWrite(FrontRightBack, 200);
     analogWrite(BackRightBack, 200);
   }
  stopMotors();
  delay(25);
}
void turnLeft(){
   	count = 0;
 	while(count < 1215){  // recorrido necesario para la vuelta
	  analogWrite(FrontRightFor, 220);
	  analogWrite(BackRightFor, 220);
	  analogWrite(BackLeftBack, 220);
	  analogWrite(FrontLeftBack, 220);
 	}
  	stopMotors();
  	delay(50);
}

void turnRight(){
  count=0;
  while(count<1210){
  analogWrite(FrontRightBack, 220);
  analogWrite(BackRightBack, 220);
  analogWrite(BackLeftFor, 220);
  analogWrite(FrontLeftFor, 220);
   }
 
  
  stopMotors();
  delay(50);
    
}
void stopMotors(){

	analogWrite(FrontLeftFor, 0);
	analogWrite(BackLeftFor, 0);
	analogWrite(FrontRightFor, 0);
	analogWrite(BackRightFor, 0);

	analogWrite(FrontLeftBack, 0);
	analogWrite(BackLeftBack, 0);
	analogWrite(FrontRightBack, 0);
	analogWrite(BackRightBack, 0);

    delayMicroseconds(20);
}

//<----------------------------Distance------------------------>

// ULTRASONIC SENSORS

NewPing ultra1(trigPin1, echoPin1, maxDistance);

NewPing ultra2(trigPin2, echoPin2, maxDistance);

NewPing ultra3(trigPin3, echoPin3, maxDistance);

NewPing ultra4(trigPin4, echoPin4, maxDistance);

NewPing ultra5(trigPin5, echoPin5, maxDistance);

NewPing ultra6(trigPin6, echoPin6, maxDistance);

void readDistLeft(){
  arrDist[0] = 0;
  arrDist[1] = 0;
   
   
  
   for(byte iA = 0; iA < 3; iA++){
     
   arrUltra1[iA] = ultra1.ping();
   arrUltra1[iA] /= US_ROUNDTRIP_CM;
   delay(5);
   arrUltra2[iA] = ultra2.ping();
   arrUltra2[iA] /= US_ROUNDTRIP_CM;
   delay(5);
   }
	for(int iA = 0; iA < 3; iA++){	// CICLO PARA SUMATORIA DE LAS DISTANCIAS
		arrDist[0] += arrUltra1[iA];
		arrDist[1] += arrUltra2[iA];

	}

   	// SE HACEN TRES CORRIMIENTOS A LA DERECHA QUE ES EQUIVALENTE A DIVIDIR ENTRE 2^2=4
   	arrDist[0] /= 3;
   	arrDist[1] /= 3;

   	Serial.print("   mean U1: ");	Serial.print(arrDist[0]);	Serial.print("	U1: ");		Serial.println(arrUltra1[2]);
	
	Serial.print("   mean U2: ");	Serial.print(arrDist[1]);	Serial.print("	U2: ");		Serial.println(arrUltra2[2]);

	Serial.println("");

        Serial3.print("   mean U1: ");	Serial3.print(arrDist[0]);	Serial3.print("	U1: ");		Serial3.println(arrUltra1[2]);
	
	Serial3.print("   mean U2: ");	Serial3.print(arrDist[1]);	Serial3.print("	U2: ");		Serial3.println(arrUltra2[2]);

	Serial3.println("");
   	


}
void readDistRight(){
  
        arrDist[5] = 0;
        arrDist[4] = 0;
	for(int iA = 0; iA < 3; iA++){
          arrUltra5[iA] = ultra5.ping();
          arrUltra5[iA] /= US_ROUNDTRIP_CM;
	  delay(5);
	  arrUltra6[iA] = ultra6.ping();
          arrUltra6[iA] /= US_ROUNDTRIP_CM;
          delay(5);

	}
	for(int iA = 0; iA < 3; iA++){	// CICLO PARA SUMATORIA DE LAS DISTANCIAS
		arrDist[4] += arrUltra5[iA];
		arrDist[5] += arrUltra6[iA];
	}

   	// SE HACEN TRES CORRIMIENTOS A LA DERECHA QUE ES EQUIVALENTE A DIVIDIR ENTRE 2^2=4
   	arrDist[4] = arrDist[4] / 3;
   	arrDist[5] = (arrDist[5] / 3) - 0.7;

     if(arrDist[5] < 0)
        arrDist[5] = 0;
     

   	Serial.print("   mean U5: ");	Serial.print(arrDist[4]);	Serial.print("	U5: ");		Serial.println(arrUltra5[2]);
	
	Serial.print("   mean U6: ");	Serial.print(arrDist[5]);	Serial.print("	U6: ");		Serial.println(arrUltra6[2]);

	Serial.println("");
}

void readDistFront(){
        arrDist[2] = 0;
        arrDist[3] = 0;
	for(int iA = 0; iA < 3; iA++){    
          arrUltra3[iA] = ultra3.ping();
          arrUltra3[iA] /= US_ROUNDTRIP_CM;
     delay(10);
          arrUltra4[iA] = ultra4.ping();
          arrUltra4[iA] /= US_ROUNDTRIP_CM; 
	  delay(10);
	}

	for(int iA = 0; iA < 3; iA++){	// CICLO PARA SUMATORIA DE LAS DISTANCIAS
		arrDist[2] += arrUltra3[iA];
		arrDist[3] += arrUltra4[iA];

	}

   	// SE HACEN TRES CORRIMIENTOS A LA DERECHA QUE ES EQUIVALENTE A DIVIDIR ENTRE 2^2=4
   	arrDist[2] = arrDist[2] / 3;
   	arrDist[3] = arrDist[3] / 3;

   	Serial.print("   mean U3: ");	Serial.print(arrDist[2]);	Serial.print("	U3: ");		Serial.println(arrUltra3[2]);
	
	Serial.print("   mean U4: ");	Serial.print(arrDist[3]);	Serial.print("	U4: ");		Serial.println(arrUltra4[2]);

	Serial.println("");
}



void readDistIR(){

  double b = 0, m = 0;
    
  arrIR[0] = 0;
  arrIR[1] = 0;

  // Valores iniciales
  if(initialIR){
    for(){int iA=0;iA<8;iA++)
    arrIrFront[iA]=analogRead(A7);
    arrIrBack[iA]=analogRead(A5);
    }
    initial=false;
  }


    for(int iA = 0; iA < 7; iA++){ // Este ciclo es para correr todos los datos y dejar la última casilla libre
        arrIrFront[iA] = arrIrFront[iA + 1];    
        arrIrBack[iA] = arrIrBack[iA + 1];  
    }
    
  arrIrFront[7] = analogRead(A4);
  arrIrBack[7] = analogRead(A5);

    for(int iA = 0; iA < 8; iA++){  // CICLO PARA SUMATORIA DE LAS DISTANCIAS
      arrIR[0] += arrIrFront[iA];
      arrIR[1] += arrIrBack[iA];

    }

    arrIR[0] = arrIR[0] >> 3; 
    arrIR[1] = arrIR[1] >> 3; 
   
  for(byte iA = 0; iA < 13; iA++){
          if(arrIR[0] >= voltage[iA] && arrIR[0] <= voltage[iA+1]){
      m = (voltage[iA + 1] - voltage[iA]) / (distance[iA + 1] - distance[iA]);
      b = voltage[iA + 1] - (m * distance[iA + 1]);
      //distance = (voltage - b) / m
            arrIR[0] = (arrIR[0] - b) / m; //-----------------> DISTANCIA AQUI
    }
  }

  for(byte iA = 0; iA < 13; iA++){
          if(arrIR[1] >= voltage[iA] && arrIR[1] <= voltage[iA+1]){
      m = (voltage[iA + 1] - voltage[iA]) / (distance[iA + 1] - distance[iA]);
      b = voltage[iA + 1] - (m * distance[iA + 1]);
      //distance = (voltage - b) / m
            arrIR[1] = (arrIR[1] - b) / m;
    }
  }

  Serial.print("   mean IR front: "); Serial.print(arrIR[0]); Serial.println(" cm");    
    
    Serial.print("   mean IR back: ");  Serial.print(arrIR[1]); Serial.println(" cm");    

    Serial.println("");  
}

void CorrectionIR(){
  byte tempIR1, tempIR2;
  bool available1= false, available2=false;

  readDistIR();
  if(arrIR[0]<100){
    available1=true;
    tempIR1=arrIR[0];
  }
  else if(arrIR[1]<100){
    available2=true;
    tempIR2=arrIR[1];
  }
  // toda rutina de forward
  while(count < 1550 ){ 
    analogWrite(FrontLeftFor, 230);
    analogWrite(BackLeftFor, 230);
    analogWrite(BackRightFor, 220);
    analogWrite(FrontRightFor, 220); 
    }
  // fin de rutina de forward

  readDistIR();
  if (available1)
  {
    if(tempIR1- arrIR[0] < 15){
      while(!(tempIR1- arrIR[0] < 15)){
        analogWrite(FrontLeftFor, 230);
          analogWrite(BackLeftFor, 230);
          analogWrite(BackRightFor, 220);
          analogWrite(FrontRightFor, 220)
          readDistIR();
    }
  }
  else if (available2)
  {
    
  }

}

void readDistance(){
  readDistLeft();
  readDistFront();
  readDistRight();
  readDistIR();
}

void readInclination(){

  
  for(int iCont = 0; iCont < 64; iCont++){
    inclinationY = 0;
    
    for(int iA = 0; iA < 32; iA++){
  	  arrInclination[iA] = analogRead(A15);
  	  inclinationY += arrInclination[iA];
	  }
    
	  inclinationY = inclinationY >> 5;

   arrIncMean[iCont] = inclinationY;
  }

  inclinationY = 0;

  for(int iA = 0; iA < 64; iA++){
    inclinationY += arrIncMean[iA];
  }

  inclinationY = inclinationY >> 6;
 


  Serial.print("Inclination: ");      Serial.println(inclinationY);
}

long unsigned int readColor()
{
  long unsigned int colorValue = pulseIn(A3, LOW);
  return colorValue;
}

void printDistance()
{
	
	Serial3.print("   mean U1: ");	Serial3.print(arrDist[0]);	Serial3.print("	U1: ");		Serial3.println(arrUltra1[3]);
	
	Serial3.print("   mean U2: ");	Serial3.print(arrDist[1]);	Serial3.print("	U2: ");		Serial3.println(arrUltra2[3]);
	
	Serial3.print("   mean U3: ");	Serial3.print(arrDist[2]);	Serial3.print("	U3: ");		Serial3.println(arrUltra3[1]);
	
	Serial3.print("   mean U4: ");	Serial3.print(arrDist[3]);	Serial3.print("	U4: ");		Serial3.println(arrUltra4[1]);
	
	Serial3.print("   mean U5: ");	Serial3.print(arrDist[4]);	Serial3.print("	U5: ");		Serial3.println(arrUltra5[1]);
	
	Serial3.print("   mean U6: ");	Serial3.print(arrDist[5]);	Serial3.print("	U6: ");		Serial3.println(arrUltra6[1]);
	
    Serial3.print("   mean IR: ");	Serial3.print(arrDist[6]);	Serial3.print("	IR: ");		Serial3.println(arrIrFront[7]);
    
    Serial3.println("");
/*
    

    Serial.print("   mean U1: ");	Serial.print(arrDist[0]);	Serial.print("	U1: ");		Serial.println(arrUltra1[1]);
	
	Serial.print("   mean U2: ");	Serial.print(arrDist[1]);	Serial.print("	U2: ");		Serial.println(arrUltra2[1]);
	
	Serial.print("   mean U3: ");	Serial.print(arrDist[2]);	Serial.print("	U3: ");		Serial.println(arrUltra3[1]);
	
	Serial.print("   mean U4: ");	Serial.print(arrDist[3]);	Serial.print("	U4: ");		Serial.println(arrUltra4[1]);
	
	Serial.print("   mean U5: ");	Serial.print(arrDist[4]);	Serial.print("	U5: ");		Serial.println(arrUltra5[1]);
	
	Serial.print("   mean U6: ");	Serial.print(arrDist[5]);	Serial.print("	U6: ");		Serial.println(arrUltra6[1]);
	
    Serial.print("   mean IR: ");	Serial.print(arrDist[6]);	Serial.print("	IR: ");		Serial.println(arrIrFront[1]);
    
    Serial.println("");            
    **/
}



void calculateDiffLeft(){
    diffLeft = arrDist[0] - arrDist[1]; // U1 - U2
    // SE PONE EN VALOR ABSOLUTO LAS DIFERENCIAS
    diffLeft = abs(diffLeft);
  }

void calculateDiffRight(){
	diffRight = arrDist[4] - arrDist[5]; // U5 - U6
	diffRight = abs(diffRight);
}

void calculateDiff(){
  calculateDiffRight();
  calculateDiffLeft();
}

void movementCorrectionLeft(){ // Utiliza U1 y U2
       	
	
        readDistLeft();
        if((arrDist[0] <= 3 || arrDist[1] <= 3) && (arrDist[0] != 0 && arrDist[1] != 0)){
             slideRight();  
        }
        if((arrDist[0] >= 12 || arrDist[1] >= 12) && (arrDist[0] != 0 && arrDist[1] != 0)){
             slideLeft();
        } 
        
        readDistLeft();
	calculateDiffLeft();    
	if(diffLeft >= maxInclinationDistance && arrDist[0] != 0 && arrDist[1] != 0){
               
		double U1 = arrDist[0];
		double U2 = arrDist[1];
  
		if(U1 > U2){
			while(U1 - U2 > maxInclinationDistance){
				turnRight(40);
                                readDistLeft();
				calculateDiffLeft();

				U1 = arrDist[0];
				U2 = arrDist[1];
			}      
		}
		else{
			while(U2 - U1 > maxInclinationDistance){
				turnLeft(60);
				readDistLeft();
				calculateDiffLeft();
				U1 = arrDist[0];
				U2 = arrDist[1];
			}
		}
	}
              
}

void movementCorrectionRight(){ // Utiliza U5 y U6

        
        
        //Esta corrección es para tratar de centrarse en el cuadro
        readDistRight();
        if((arrDist[4] <= 3 || arrDist[5] <= 3) && (arrDist[4] != 0 && arrDist[5] != 0)){
             slideLeft();
        }
        if((arrDist[4] >= 12 || arrDist[5] >= 12) && (arrDist[4] != 0 && arrDist[5] != 0)){
             slideRight();
        }
        
        // Esta corrección endereza al Robot
    	readDistRight();
	calculateDiffRight();
	if(diffRight >= maxInclinationDistance && (arrDist[4] != 0 && arrDist[5] != 0)){
		double U5 = arrDist[4];
		double U6 = arrDist[5];
		if(U6 > U5){
		  while(U6 - U5 > maxInclinationDistance){
		    turnLeft(25);
		    readDistRight();
		    calculateDiffRight();

		    U5 = arrDist[4];
		    U6 = arrDist[5];
		  }
		}
		else{
      while(U5 - U6 > maxInclinationDistance){
		    turnRight(25);
		    readDistRight();
		    calculateDiffRight();
              
                    U5 = arrDist[4];
	            U6 = arrDist[5];
		  }
		}
	}      
}

void movementCorrectionFront(){
	readDistFront();
	if(arrDist[2] != 0 || arrDist[3] != 0){
		if(arrDist[2] <= 3 || arrDist[3] <= 3){
			while((arrDist[2] <= 3 || arrDist[3] <= 3) && (arrDist[2] != 0 && arrDist[3] != 0)){
				back(25);
				readDistFront();
			}
		}
	}
}

void movementCorrection(){
 
 movementCorrectionLeft();
 movementCorrectionRight(); 
 movementCorrectionFront();
}




//---------------->Variables Generales
char Maze[40][40];


			//	  0 1 2 3 4 5 6 7 8 9 

char direct = 'U';
/* R = right
   L = left
   U = up
   D = down
*/

 byte  row = 20, column = 20 ; // 'center' of matrix 


 //-----------------> Funciones <---------------//
 

 void mapNavegation(){
 bool bWallF = false, bWallL = false, bWallR = false, bRoomF = false, bRoomL = false, bRoomR = false;
  
     

 	if(arrDist[2] != 0 && arrDist[3] != 0) // Hay pared enfrente
 		bWallF = true;
 		
	if(arrDist[0] != 0 && arrDist[1] != 0) // Hay pared a la izquierda
		bWallL = true;
	
	if(arrDist[4] != 0 && arrDist[5] != 0) // Hay pared a la derecha
		bWallR = true;
 	
	if(direct=='U')
	{
		if(bWallL && Maze[row][column-1] != '-')
			Maze[row][column-1]='-';
		else{
			if(Maze[row][column-2] == 'x')
				bRoomL = true;	// Ya visito ese cuarto
		} 

		if(bWallF && Maze[row-1][column] != '-')
			Maze[row-1][column]='-';
		else{
			if(Maze[row-2][column] == 'x')
				bRoomF = true;	// Ya visito ese cuarto
		}

		if(bWallR && Maze[row][column+1] != '-')
			Maze[row][column+1]='-';
		else{
			if(Maze[row][column+2] == 'x')
				bRoomR = true;	// Ya visito ese cuarto
		}

		// fin de preparacion de desiciones

		if(!bWallL && !bRoomL){
			turnLeft();
			updatePosition('l');
			forward();
			updatePosition('f');
		}
		else if(!bWallF && !bRoomF){
			forward();
			updatePosition('f');

		}
		else if(!bWallR && !bRoomR){
			turnRight();
			updatePosition('r');
			forward();
			updatePosition('f');
		}
		else if(!bWallL){
			turnLeft();
			updatePosition('l');
			forward();
			updatePosition('f');
		}
		else if(!bWallF){
			forward();
			updatePosition('f');
		}
		else if(!bWallR){
			turnRight();
			updatePosition('r');
			forward();
			updatePosition('f');
		}
		else{// Este caso es donde se encierra
			turnRight();
			updatePosition('r');
			turnRight();
			updatePosition('r');
		}
	}
//------------------------------------------------------------------------->
	else if(direct=='D') // Si esta mirando hacia abajo en la matriz
	{
		if(bWallL && Maze[row][column+1] != '-')
			Maze[row][column+1]='-';
		else{
			if(Maze[row][column+2] == 'x')
				bRoomL = true;
		}

		if(bWallF && Maze[row+1][column] != '-')
			Maze[row+1][column]='-';
		else{
			if(Maze[row+2][column]=='x')
				bRoomF =true;
		}

		if(bWallR && Maze[row][column-1] != '-')
			Maze[row][column-1]='-';
		else{
			if(Maze[row][column-2]=='x')
				bRoomR =true;
		}

		// fin de preparacion de desiciones

		if(!bWallL && !bRoomL){
			turnLeft();
			updatePosition('l');
			forward();
			updatePosition('f');
		}
		else if(!bWallF && !bRoomF){
			forward();
			updatePosition('f');

		}
		else if(!bWallR && !bRoomR){
			turnRight();
			updatePosition('r');
			forward();
			updatePosition('f');
		}
		else if(!bWallL){
			turnLeft();
			updatePosition('l');
			forward();
			updatePosition('f');
		}
		else if(!bWallF){
			forward();
			updatePosition('f');

		}
		else if(!bWallR){
			turnRight();
			updatePosition('r');
			forward();
			updatePosition('f');
		}
		else{// Este caso es donde se encierra
			turnRight();
			updatePosition('r');
			turnRight();
			updatePosition('r');
		}
	}
//------------------------------------------------------------------------->
	else if (direct=='R') // Su frente es hacia la derecha
	{
		if(bWallL && Maze[row-1][column]!='-')
			Maze[row-1][column]='-';
		else{
			if(Maze[row-2][column]=='x')
				bRoomL= true;
		}

		if(bWallF && Maze[row][column+1]!='-')
			Maze[row][column+1]='-';
		else{
			if(Maze[row][column+2]=='x')
				bRoomF=true;
		}

		if(bWallR && Maze[row+1][column]!='-')
			Maze[row+1][column]='-';
		else{
			if(Maze[row+2][column]=='x')
				bRoomR=true;
		}

		// fin de preparacion de desiciones

		if(!bWallL && !bRoomL){
			turnLeft();
			updatePosition('l');
			forward();
			updatePosition('f');
		}
		else if(!bWallF && !bRoomF){
			forward();
			updatePosition('f');

		}
		else if(!bWallR && !bRoomR){
			turnRight();
			updatePosition('r');
			forward();
			updatePosition('f');
		}
		else if(!bWallL){
			turnLeft();
			updatePosition('l');
			forward();
			updatePosition('f');
		}
		else if(!bWallF){
			forward();
			updatePosition('f');

		} 	
		else if(!bWallR){
			turnRight();
			updatePosition('r');
			forward();
			updatePosition('f');
		}
		else{// Este caso es donde se encierra
			turnRight();
			updatePosition('r');
			turnRight();
			updatePosition('r');
		}


	}
//------------------------------------------------------------------------>		
	else if(direct=='L')
	{
		if(bWallF && Maze[row][column-1]!='-')
			Maze[row][column-1]='-';
		else{
			if(Maze[row][column-2]=='x')
				bRoomF=true;
		}
		if(bWallR && Maze[row-1][column]!='-')
			Maze[row-1][column]='-';
		else{
			if(Maze[row-2][column]=='x')
				bRoomR=true;
		}

		if(bWallL && Maze[row+1][column]!='-')
			Maze[row+1][column]='-';
		else{
			if(Maze[row + 2][column]=='x')
				bRoomL=true;
		}	

		// fin de preparacion de desiciones

		if(!bWallL && !bRoomL){
			turnLeft();
			updatePosition('l');
			forward();
			updatePosition('f');
		}
		else if(!bWallF && !bRoomF){
			forward();
			updatePosition('f');

		}
		else if(!bWallR && !bRoomR){
			turnRight();
			updatePosition('r');
			forward();
			updatePosition('f');
		}
		else if(!bWallL){
			turnLeft();
			updatePosition('l');
			forward();
			updatePosition('f');
		}
		else if(!bWallF){
			forward();
			updatePosition('f');

		}
		else if(!bWallR){
			turnRight();
			updatePosition('r');
			forward();
			updatePosition('f');
		}
		else{// Este caso es donde se encierra
			turnRight();
			updatePosition('r');
			turnRight();
			updatePosition('r');
		}
	}
 }

 void updatePosition(char tempmove){
 	/* Posible for tempmove
			f=forward
			b=backward
			r=turnRight
			i=turnLeft
 	*/
			if( tempmove == 'f'){
				if (direct=='R')
				{
					column+=2;
					Maze[row][column]='x';
				}
				else if(direct=='L')
				{
					column-=2;
					Maze[row][column]='x';
				}
				else if(direct=='U')
				{
					row-=2;
					Maze[row][column]='x';
				}
				else if(direct=='D')
				{
					row+=2;
					Maze[row][column]='x';
				}
			}
			/*
			else if( tempmove == 'b'){
				if (direct=='R')
				{
					column--;
				}
				else if(direct=='L')
				{
					column++;
				}
				else if(direct=='U')
				{
					row++;
				}
				else if(direct=='D')
				{
					row--;
				}
			}
			*/
			else if( tempmove == 'r'){
				if (direct=='R')
				{
					direct='D';
				}
				else if(direct=='L')
				{
					direct='U';
				}
				else if(direct=='U')
				{
					direct='R';
				}
				else if(direct=='D')
				{
					direct='L';
				}

			}
			else if ( tempmove == 'l'){
				if (direct=='R')
				{
					direct='U';
				}
				else if(direct=='L')
				{
					direct='D';
				}
				else if(direct=='U')
				{
					direct='L';
				}
				else if(direct=='D')
				{
					direct='R';
				}
			} 

 }

 void printMap(){

 	for(int i =0; i<40; i++){
 		for(int k=0;k<40; k++){
 			Serial.print(Maze[i][k]); 
      
 		}
 		Serial.println("");
 	}
  Serial.println(direct);
 	 delay(3000);
 }


//----------------->Setup
 void setup(){

 	Serial3.begin(9600);
	Serial.begin(9600);
	pinMode(encoderI, INPUT);
	attachInterrupt(0, handleEncoder, CHANGE);
        //attachInterrupt(5, stopMotors, LOW);

	  //<-------------Wheels Setup---------------------------->
	pinMode(FrontLeftFor, OUTPUT);
	pinMode(BackLeftFor, OUTPUT);
	pinMode(FrontRightFor, OUTPUT);
	pinMode(BackRightFor, OUTPUT);

	pinMode(FrontLeftBack, OUTPUT);
	pinMode(BackLeftBack, OUTPUT);
	pinMode(FrontRightBack, OUTPUT);
	pinMode(BackRightBack, OUTPUT);

        pinMode(23, OUTPUT);
        pinMode(25,OUTPUT);
        pinMode(colorSensor, INPUT);
        
        digitalWrite(23, HIGH);
        digitalWrite(25, HIGH);
         Maze[10][10]='S';
         for(int iR = 0; iR < 40; iR++){
 for(int iC = 0; iC < 40; iC++){
    Maze[iR][iC]= '0';
  }
  Maze[20][20]='*';
}
        delay(1000);
 }

 //---------------->Loop
void loop(){
  /*readDistIR();
  readDistFront();

*/
  
readDistance();
mapNavegation();

	
}



