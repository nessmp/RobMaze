Explicaci�n del Algoritmo

El laberinto de la ronda lo dividimos en cuadros de 30 x 30 dej�ndonos un plano cartesiano con los ejes X, Y y Z; a lo largo del algoritmo iremos actualizando los valores de estos tres ejes y tambi�n el n�mero de paso en el que vamos, al inicio de la ronda se est� en el paso n�mero uno, tras moverse 30 cm pasamos al paso 2 y as� consecutivamente.
El cuadro en el que se inicia se le da el valor de paso 1 y los valores de Pos (ZZ/2, XX/2, YY/2), al no saber el tama�o del laberinto se est� en una desventaja por lo que los valores de XX, YY y ZZ deben ser lo suficientemente grandes como para estar preparados para cualquier tipo de laberinto, una de las grandes desventajas de tener estos valores est�ticos es que si la ronda se inicia en la esquina del laberinto se estar�an perdiendo la mitad de los valores de XX y YY por lo que se necesita mucha memoria para este algoritmo.
Adem�s de guardar el n�mero de paso en el que vamos y la coordenada perteneciente a �l guardamos las coordenadas de los cuadros adyacentes que se encuentran accesibles, o sea que no tienen pared entre cuadro y cuadro; tras obtener toda esta informaci�n comenzamos a movernos por el laberinto, nosotros escogimos tener una preferencia de: Derecha, Enfrente, Izquierda y Atr�s. Tras cada movimiento volvemos a actualizar los valores del paso en el que vamos, la coordenada en donde nos encontramos y las nuevas coordenadas adyacentes al cuadro.
Si en alg�n momento nos encontramos en alg�n cuadro donde no hay cuadro inexplorado adyacente revisamos la informaci�n del cuadro perteneciente al paso anterior y revisamos las coordenadas que guardamos en el hasta encontrar una que no se haya explorado, si en ese cuadro tampoco hay una coordenada inexplorada seguimos regres�ndonos paso a paso hasta encontrar una coordenada inexplorada. En el momento que se encuentre una coordenada inexplorada lo primero es moverse hasta el paso al que pertenece esta coordenada, para esto nos movemos al cuadro adyacente que tenga el paso m�s cercano al paso donde nos queremos mover ya estando en este cuadro nos movemos hacia la nueva coordenada. En caso de que no encontremos ninguna coordenada inexplorada podemos decir que terminamos de recorrer el laberinto por lo que nos dirigimos hacia el cuadro con el paso #1

Cosas que mejorar y a�adir (problemas que conocemos)

Este algoritmo actualmente tiene 2 problemas mayores, el primer problema (y el mayor de ellos) es el hecho que se requiere que el robot haga movimientos perfectos para que sea �til el algoritmo para resolver esto ser�a necesario est�r haciendo lecturas despu�s de cada movimiento de 30 cm, tras el movimiento revisamos en los Run del paso en el que creemos encontrarnos y los comparamos con los que tenemos, si la informaci�n concuerda significa que no hemos realizado ning�n movimiento err�neo, pero en caso de que esta informaci�n no concuerde tendr�amos que buscar a que paso le corresponde la informaci�n que tenemos y as� corregirnos.
El segundo problema es la memoria que se necesita para usar el algoritmo, ya que XX, YY, y ZZ son variables est�ticas adem�s que tenemos que guardar mucha informaci�n en cada cuadro, para solucionar esto se podr�a intentar usar memoria din�mica pero en un arduino puede no ser la mejor idea, otra soluci�n ser�a guardar toda la informaci�n en una memoria SD o en otra parte que no sea el arduino, tambi�n se podr�an buscar formas de utilizar menos memoria necesitando menos informaci�n por cuadro mejorando el algoritmo. 
Otra cosa que se podr�a mejorar seria la funci�n MoveUntil (), ya que hay caso donde la ruta que sigue el robot no es la m�s �ptima por lo que ser�a bueno buscar la manera de implementar alg�n algoritmo que te consiga la ruta m�s r�pida entre dos puntos como el algoritmo Dijkstra

Explicaci�n de funciones

Laberinto() {
Llama a GetDatos()
Llama a SearchRouteAndMove()
}
GetDatos(){
bDir = arreglo booleano de 4 [Derecha, Izquierda, Enfrente, Atr�s] si es true significa que no hay pared en esa direcci�n, estos datos los consigue con la funci�n getPossibility(bDir); con esta informaci�n llena Posibility y Run
Aumenta �Paso� y actualiza pasoActual, con esto actualiza Pos
}
getPossibility(bDir) {
llena bDir con los sensores para detectar paredes y regresa el n�mero de Trues en bDir
}
SearchRoouteAndMove(){
bData = paso que tiene una ubicaci�n inexplorada, lo llena con WhereToGo()
Sabiendo ya el paso al que se debe de mover llama a moveUntil(bData)
Por ultimo estando ya en un paso con un Run inexplorado llama a exploreNewWorlds(bData)
}
WhereToGo(){
Regresa el paso que tiene una ubicaci�n inexplorada (un Run), para esto primero revisa los Run del pasoActual, si hay un Run inexplorado y la coordenada perteneciente a ese Run no se encuentra en la listaX regresa este valor; en caso de que no haya ning�n Run inexplorado en ese paso se regresa un paso y vuelve a revisar; en caso de que haya revisado todos los pasos posibles y no haya encontrado ning�n Run inexplorado llama a la funci�n extractionPoint()  
}
ExtractionPoint(){
Llama a MoveUntil(1) para moverse al cuadro donde se inici� la ronda, al llegar a ese cuadro se detiene durante 30 segundos.
}
moveUntil(bData) {
Se mueve hasta el paso bData, para esto revisa el paso en el que esta y el paso al que quiere llegar, y se mueve al paso m�s cercano al que quiere llegar
}
exploreNewWorlds(bHere) {
Revisa los Run de bHere y se mueve hacia el Run inexplorado para esto llama a la funci�n Move(iCoordAc, icCoord)
}
Move(iCoordAc, icCoord){
Con iCoordAc (la coordenada en la que se encuentra) y con icCoord (la coordenada a la que se quiere mover) detecta en qu� direcci�n se debe de mover y se mueve actualizando el valor correspondiente (bX, bY o bZ)
Tras moverse revisa:
 si el nuevo cuadro es un cuadro negro, en caso de serlo retrocede un cuadro e incluye la coordenada del cuadro negro a la listaX, llama a searchRouteAndMove() para encontrar un nuevo Run inexplorado y moverse a el
si el nuevo cuadro es una rampa, en caso de serlo entra en la funci�n MovRampa() y asigna un true a bARampa para modificar los Run de las coordenadas en ambos lados de la rampa
}
MovRampa() {
Detecta la inclinaci�n con el aceler�metro y se mueve hasta que esta inclinaci�n desaparezca, dependiendo de la inclinaci�n modifica el nuevo valor de bZ.
}
BeenHere(cRun, iPaso) {
Regresa True si ya se ha estado antes en el Run enviado, en caso contrario regresa un False
}
getPass(cRun, iPaso) {
Regresa el paso de Run que se le env�a como par�metro
}
getCoord(cRun, iPaso) {
regresa la coordenada del Run que se le env�a como par�metro
}
Rampa(){
True si hay inclinaci�n de rampa
}
HoyoNegro {
True si se est� sobre un cuadro negro
}

Explicaci�n Variables Globales

XX = cantidad de cuadros en el eje X
YY = cantidad de cuadros en el eje Y
ZZ = cantidad de pisos
maxPasos = n�mero m�ximo de pasos
maxNegros = n�mero m�ximo de cuadros negros
Pos[ZZ][XX][YY] = Paso perteneciente a esa coordenada
Possibility[maxPasos] = posibilidades de movimiento en la coordenada
Run[maxPasos][4] = Coordenada a la que se puede mover, puede tener varios valores cada uno significa algo diferente:
	y = puede moverse en el eje y + 1
	p = puede moverse en el eje y - 1
	t = puede bajar una rampa en el eje y + 1
	u = puede bajar una rampa en el eje y - 1
	d = puede subir una rampa en el eje y + 1
	e = puede subir una rampa en el eje y - 1
	x = puede moverse en el eje x + 1	
	o = se puede moverse en el eje x - 1
	s = puede bajar una rampa en el eje x + 1
	r = puede bajar una rampa en el eje x - 1
	b = puede subir una rampa en el eje x + 1
	c = puede subir una rampa en el eje x - 1
listaX[maxNegros] = coordenada de los cuadros negros
Direcc = Direcci�n en la que est� el robot

	1
	^
	|
    3<-- -->2
	|
	v
	4
Paso = Paso en el que va
PasoActual = Paso del cuadro en donde esta
bX = coordenada actual de X, inicia en la mitad de XX
bY = coordenada actual de Y, inicia en la mitad de YY
bZ = coordenada actual de Z, ten�amos mucha memoria entonces como solo eran dos pisos ZZ lo igual�bamos a 3 y siempre pon�amos bZ en 1
bARampa = es un booleano, true si en el movimiento anterior se subi� o se baj� una rampa, esto se usa para actualizar la variable Run
bPassRampa = paso anterior a la rampa
brRampa = paso despu�s de la rampa
