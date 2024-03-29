#!/usr/bin/env python
import rospy,sys,roslib
import cv2
from percepcion.msg import Mapa
from omnirobot_control.msg import V_waypoint
import numpy as np
 
flag=0

# Covertir mensaje con el vector del mapa a matriz
def actualiza_mapa(data):
	global mapa_matriz
	global mapa_xy
	global flag
	global X_R, Y_R

	if (data.nfil * data.ncol) == len(data.mapa):

		mapa_matriz = np.zeros((data.nfil,data.ncol))
		mapa_xy = np.ndarray((data.nfil,data.ncol,2))

		for i in range(0,data.nfil):
				for j in range(0,data.ncol):
					mapa_matriz[i][j] = data.mapa[i*data.nfil+j]
					mapa_xy[i][j][0] = data.mapa_x[i*data.nfil+j]
					mapa_xy[i][j][1] = data.mapa_y[i*data.nfil+j]

	X_R = data.j_r
	Y_R = data.i_r

	flag = 1

# Clases
# ---------------------------------------------------------------------

class Nodo:

	#Posicion  Coordenadas del nodo.
	#Padre. Almacena el nodo padre.
	#Valor de H. Distancia con el punto objetivo.
	#Valor de G. Distancia recorrida hasta ese nodo.
	#Valor de F. Suma de G mas H.

	def __init__(self, pos=[0, 0], padre=None):
		self.pos = pos
		self.padre = padre
		self.h = distancia(self.pos, pos_f) #pos_f es la posicion final (variable global)
		
		if self.padre == None:
			self.g = 0
		else:
			self.g = self.padre.g + 1
		self.f = self.g + self.h

class any_angle_path_planning:
	def __init__(self, mapa,X_R,Y_R,X_F,Y_F):
		
		self.mapa = mapa
		
		globals()["pos_f"] = [Y_F,X_F]
		
		# Nodos de inicio y fin.
		#self.inicio = Nodo(buscarPos(2, mapa))
		#self.fin = Nodo(buscarPos(3, mapa))
		self.inicio = Nodo([Y_R,X_R]) #en este ejemplo el inicio se representa con un 2
		self.fin = Nodo(pos_f)#el waypoint se representa con el numero 3

		# Crea las listas abierta y cerrada.
		self.abierta = []
		self.cerrada = []
		
		# Aniade el nodo inicial a la lista cerrada.
		self.cerrada.append(self.inicio)
		
		# Aniade vecinos a la lista abierta
		self.abierta += self.vecinos(self.inicio)
		
		# Buscar mientras objetivo no este en la lista cerrada.
		while self.objetivo():
			if not self.abierta:
				break
			self.buscar()
		
		if not self.abierta:
			self.camino = -1
		else:	
           		self.camino = self.camino()

			
	
	# Devuelve una lista con los nodos vecinos transitables.	
	def vecinos(self, nodo):
		vecinos = []
		if self.mapa[nodo.pos[0]+1][nodo.pos[1]] != 1:
			vecinos.append(Nodo([nodo.pos[0]+1, nodo.pos[1]], nodo))
		if self.mapa[nodo.pos[0]-1][nodo.pos[1]] != 1:
			vecinos.append(Nodo([nodo.pos[0]-1, nodo.pos[1]], nodo))
		if self.mapa[nodo.pos[0]][nodo.pos[1]-1] != 1:
			vecinos.append(Nodo([nodo.pos[0], nodo.pos[1]-1], nodo))
		if self.mapa[nodo.pos[0]][nodo.pos[1]+1] != 1:
			vecinos.append(Nodo([nodo.pos[0], nodo.pos[1]+1], nodo))
		if self.mapa[nodo.pos[0]+1][nodo.pos[1]+1] != 1:
			vecinos.append(Nodo([nodo.pos[0]+1, nodo.pos[1]+1], nodo))
		if self.mapa[nodo.pos[0]+1][nodo.pos[1]-1] != 1:
			vecinos.append(Nodo([nodo.pos[0]+1, nodo.pos[1]-1], nodo))
		if self.mapa[nodo.pos[0]-1][nodo.pos[1]+1] != 1:
			vecinos.append(Nodo([nodo.pos[0]-1, nodo.pos[1]+1], nodo))
		if self.mapa[nodo.pos[0]-1][nodo.pos[1]-1] != 1:
			vecinos.append(Nodo([nodo.pos[0]-1, nodo.pos[1]-1], nodo))
		return vecinos
	
	# Pasa el elemento de f menor de la lista abierta a la cerrada. 	
	def f_menor(self):
		a = self.abierta[0]
		n = 0
		for i in range(1, len(self.abierta)):
			if self.abierta[i].f < a.f:
				a = self.abierta[i]
				n = i
		self.cerrada.append(self.abierta[n])
		del self.abierta[n]
		
	
	# Comprueba si un nodo esta en una lista.	
	def en_lista(self, nodo, lista):
		for i in range(len(lista)):
			if nodo.pos == lista[i].pos:
				return 1
		return 0
	
	
	# Gestiona los vecinos del nodo seleccionado.	
	def ruta(self):
		for i in range(len(self.nodos)):
			if self.en_lista(self.nodos[i], self.cerrada):
				continue
			elif not self.en_lista(self.nodos[i], self.abierta):
				self.abierta.append(self.nodos[i])
			else:
				if self.select.g+1 < self.nodos[i].g:
					for j in range(len(self.abierta)):
						if self.nodos[i].pos == self.abierta[j].pos:
							del self.abierta[j]
							self.abierta.append(self.nodos[i])
							break
	
	# Analiza el ultimo elemento de la lista cerrada.
	def buscar(self):
		self.f_menor()
		self.select = self.cerrada[-1]
		self.nodos = self.vecinos(self.select)
		self.ruta()
	
	# Comprueba si el objetivo objetivo esta en la lista abierta.
	def objetivo(self):
		for i in range(len(self.abierta)):
			if self.fin.pos == self.abierta[i].pos:
				return 0
		return 1
	
	# Retorna una lista con las posiciones del camino a seguir.
	def camino(self):
		for i in range(len(self.abierta)):
			if self.fin.pos == self.abierta[i].pos:
				objetivo = self.abierta[i]
				
		camino = []
		while objetivo.padre != None:
			camino.append(objetivo.pos)
			objetivo = objetivo.padre
		camino.reverse()
		return camino

# ---------------------------------------------------------------------

# Funciones
# ---------------------------------------------------------------------

# Devuelve la posicion de "x" en una lista.
def buscarPos(x, mapa):
	for f in range(len(mapa)):
		for c in range(len(mapa[0])):
			if mapa[f][c] == x:
				return [f, c]
	return 0

# Distancia entre dos puntos.
def distancia(a, b):
	return abs(a[0] - b[0]) + abs(a[1] - b[1]) #Valor absoluto.

#Transformamos la lista de puntos de paso de filas x columnas a coordenadas
def convierte_camino(camino):
	global mapa_xy
	trayectoria = []
	
	for punto in camino:
		trayectoria.append([mapa_xy[punto[1]][punto[0]][0],mapa_xy[punto[1]][punto[0]][1]])

	return trayectoria

#Mostramos la trayectoria en imagen
def muestra_trayectoria(mapa_matriz,X_R,Y_R,X_F,Y_F,camino):
	#Mostramos
	mapa_completo = mapa_matriz.copy()
	mapa_completo[Y_R,X_R] = 1
	mapa_completo[Y_F,X_F] = 1

	for punto in camino:
		mapa_completo[punto[0],punto[1]]=0.5
	
	mapa_completo = cv2.resize(mapa_completo,(int(300),int(300)),interpolation = cv2.INTER_AREA)
	cv2.imshow("Trayectoria",mapa_completo)
	cv2.waitKey(1)

#Detecta si hemos pulsado la interfaz de seleccion
def on_click(event,x,y,flags,param):
	global X_F,Y_F
	if(event == cv2.EVENT_LBUTTONDOWN):
		X_F = x/10
		Y_F = y/10
		print(x/10,y/10)

#Comprobamos si existe solapamiento entre la trayectoria que va a seguir y un nuevo elemento del mapa
def check_colision(mapa, trayectoria):
	global obstaculo_detectado
	for punto in trayectoria :
		if mapa[punto[0],punto[1]] != 0 :
			obstaculo_detectado = 1

# ---------------------------------------------------------------------

def listener():
	global X_F,Y_F
	global flag
	global mapa_matriz
	global obstaculo_detectado

	rospy.init_node('planificador', anonymous=True)
	rospy.Subscriber('mapper/mapa', Mapa, actualiza_mapa)
	pub = rospy.Publisher('planificador/trayectoria', V_waypoint, queue_size=10)	
	rate = rospy.Rate(10)

	#Trayectoria vacia inicialmente
	trayectoria = -1
	
	while 1 :

		#Inicio. Esperamos mapa
		while flag==0:
			rate.sleep()
		flag = 0
		
		#Kernel para dilatar el mapa y establecer una distancia de seguridad
		kernel = np.ones((3,3),np.uint8)
		
		#Punto final pulsado por pantalla
		X_F, Y_F = -1,-1
		
		#Espera a la seleccion de un nuevo punto o de la interseccion de un obstaculo con la trayectoria
		cv2.namedWindow('Seleccion_de_punto_final')
		cv2.setMouseCallback('Seleccion_de_punto_final', on_click)
		obstaculo_detectado = 0
		while((X_F == -1 or Y_F == -1) and not obstaculo_detectado):
			mapa_dilatado = mapa_matriz.copy()
			mapa_dilatado = cv2.dilate(mapa_dilatado,kernel,iterations = 1)
			if trayectoria != -1:
				check_colision(mapa_matriz,trayectoria)
			mapa_dil_escalado = cv2.resize(mapa_matriz,(int(300),int(300)),interpolation = cv2.INTER_AREA)
			cv2.imshow("Seleccion_de_punto_final", mapa_dil_escalado)
			cv2.waitKey(1)
			rate.sleep()
		
		#Si ha habido un obstaculo, replanificamos con el punto anterior
		if(obstaculo_detectado):
			X_F, Y_F = oldX_F, oldY_F

		#Guardamos el punto que enviamos por si es necesario replanificar
		oldX_F , oldY_F = X_F, Y_F

		#PLANIFICADOR:
		planificador = any_angle_path_planning(mapa_dilatado,X_R,Y_R,X_F,Y_F)
		trayectoria = planificador.camino

		#Si habia un camino posible...
		if planificador.camino != -1 :
			#Mostramos el resultado
			muestra_trayectoria(mapa_matriz,X_R,Y_R,X_F,Y_F,planificador.camino)

			#Pasamos las coordenadas de la trayectoria a coordenadas del mundo
			camino_xy = convierte_camino(planificador.camino)
			
			#Enviamos por el topic
			tam = len(camino_xy)
			camino_xy = np.array(camino_xy)
			data = V_waypoint()
			x = camino_xy[:,0]
			data.x = x.astype(np.float64).tolist()
			y = camino_xy[:,1]
			data.y = y.astype(np.float64).tolist()
			data.tam = tam

		#Si no habia un camino posible...
		else :
			data.x = [mapa_xy[X_R][Y_R][0]]
			data.y = [mapa_xy[X_R][Y_R][1]]
			data.tam = 1
		pub.publish(data)
	
if __name__ == '__main__':
	listener()