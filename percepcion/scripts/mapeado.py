#!/usr/bin/env python
import rospy,sys,cv2,roslib

from math import sin,cos,sqrt,pow,atan2
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
from percepcion.msg import Mapa
from geometry_msgs.msg import Pose
import numpy as np
import time as time

#Activar si se va a ejecutar todo a la vez
espera_inicial = 1 # 0 -> Se ejecuta al llamarlo / 1 -> Espera al inicio de la simulacion

bridge = CvBridge()

currentImage = np.zeros((256,256,3),dtype="uint8")

#Matriz de parametros intrinsecos de la camara
K  = np.array([[476.7030836014194, 0.0, 400.5], [0.0, 476.7030836014194, 400.5], [0.0, 0.0, 1.0]])

#Rotacion y traslacion
phi = np.pi
psi = -0
Rx = np.array([[1, 0, 0],[0, cos(phi), -sin(phi)], [0,  sin(phi),  cos(phi)]])
Rz = np.array([[cos(psi), -sin(psi), 0],[sin(psi),  cos(psi), 0], [0,  0,  1]])
cRw = np.dot(Rz,Rx)
ctw = np.array([[0],[0],[4]])

def getImage(data):
	global currentImage
	try:
		currentImage = bridge.imgmsg_to_cv2(data,'bgr8')
		
	except CvBridgeError as e:
		print(e)


def newInfo(data):
	global myCameraInfo
	myCameraInfo = data
	rospy.loginfo("\nRecibido nueva info:\n" + str(data))

def getPosicion(imagenOR, h_color, th, mostrar):
	imagen = imagenOR.copy()

	## BINARIZACION Y APLICACION DE CRITERIO DE COLOR EN HSV
	hmin = h_color - th[0]
	hmax = h_color + th[1]
	frameHSV = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
	frameThreshold = cv2.inRange(frameHSV,(hmin/360.0*179,0,0),(hmax/360.0*179,255,255))
	im2, contours, hier = cv2.findContours(frameThreshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


	## CALCULO DEL CENTRO DEL BLOB
	# Usamos los momentos de CV2 para su calculo mas rapido
	n = 0
	cX = 0
	cY = 0
	# Si hay varios bloques, hacemos la media ponderada por peso
	M = cv2.moments(frameThreshold)
	if(M["m00"] != 0):
		cX += int(M["m10"]/M["m00"])
		cY += int(M["m01"]/M["m00"])
		n = n + M["m00"]
	else:
		cX, cY = 0, 0

	if(mostrar == True):
		# Pintamos un circulo en la imagen original para ver el
		cv2.circle(imagen, (cX, cY), 2, (255, 255, 255), -1)
		#cv2.putText(imagen, "centro", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

		## MOSTRAMOS IMAGENES
		cv2.imshow("Image", currentImage)
		cv2.imshow("HSV Image", frameHSV)
		cv2.imshow("Threshold Image", frameThreshold)
		cv2.waitKey(1)

	return cX, cY


def segmentacionBinaria(imagenOR, h_color, th, mostrar):
    imagen = imagenOR.copy()
    hmin = h_color - th[0]
    hmax = h_color + th[1]
    frameHSV = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
    frameThreshold = cv2.inRange(frameHSV,(hmin/360.0*179,0,0),(hmax/360.0*179,255,255))


    if(mostrar == True):
        # Mostramos la imagen binaria
		cv2.imshow("Threshold Image", frameThreshold)
		cv2.waitKey(1)

    return frameThreshold

def morf(im_o, rMaskOp, rMaskCl,mostrar):

	#Aplicamos morfologia al mapa para eliminar pequenios huecos o imperfecciones

	im = np.copy(im_o)

	if rMaskCl != 0:
		kernelCl = np.ones((2*rMaskCl+1, 2*rMaskCl+1))
		im = cv2.morphologyEx(im, cv2.MORPH_CLOSE, kernelCl)

	if rMaskOp != 0:
		kernelOp = np.ones((2*rMaskOp+1, 2*rMaskOp+1))
		im = cv2.morphologyEx(im, cv2.MORPH_OPEN, kernelOp)

	if mostrar == True:
		cv2.imshow("Morf Image", im)
		cv2.waitKey(1)

	return im

def inv_transf(pix_,K,R,t,z):

	#Realizamos la transformada inversa del punto que le pasemos asumiendo que estamos al altura del suelo
	xyz = z*np.dot(np.dot(np.transpose(R),np.linalg.inv(K)),pix_) - np.dot(np.transpose(R), t)
	
	"""
	#Desglose de la transformacion inversa a modo de Debug
	pix = pix_[0:2,0]
	print("----pix----")
	print(pix)

	xy_n = np.dot((pix-K[0:2,2]),np.linalg.inv(K[0:2,0:2]))
	print("----xy_n----")
	print(xy_n)

	xyz_c  = np.append(xy_n*z,z)[np.newaxis]
	print("----xyz_c----")
	print(xyz_c.T)

	xyz = np.dot(R.T,(xyz_c.T - t))
	print("----xyz----")
	"""
   
	return xyz

def discretizador(im,nfil,ncol,mostrar):
	#Reducimos el mapa a una cuadricula de resolucion nfil x ncol
	mapa_res = np.zeros((nfil,ncol))
	M,N = im.shape
	Mi = int(round(M/nfil))
	Ni = int(round(N/ncol))

	for i in range(0,nfil):
		for j in range(0,ncol):
			#Tomamos la cuadricula en la posicion del mapa
			bloque = np.add.reduce(im[i*Mi:(i+1)*Mi,j*Ni:(j+1)*Ni],None)

			#Con que haya un pixel de mapa en la cuadricula se considera pared
			if bloque != 0 :
				bloque = 1
			mapa_res[i,j]=bloque

	if mostrar == True:
		cv2.imshow("Map Image", mapa_res)
		cv2.waitKey(1)
	
	#Devolvemos el mapa en la cuadricula
	return mapa_res
			
def linealizador(mapa,nfil,ncol):
	#Funcion que convierte el mapa en un vector nfil*ncol para su envio
	tamVector = nfil*ncol
	vector = np.ndarray((1,tamVector))

	for i in range(0,nfil):
		for j in range(0,ncol):
			vector[0,i*nfil+j]= mapa[i,j]

	return vector

def xy_mapa(M,N,nfil,ncol,mostrar):
	#Calculo de las coordenadas x,y de los puntos de la cuadricula del mapa
	global K, cRw, ctW
	x_mapa = np.ndarray((1,nfil*ncol))
	y_mapa = np.ndarray((1,nfil*ncol))

	Mi = int(round(M/nfil))
	Mi_2 = int(round(Mi/2))
	Ni = int(round(N/ncol))
	Ni_2 = int(round(Ni/2))

	#Calculamos las coordenadas de los puntos centrales de la cuadricula
	for i in range(0,nfil):
		for j in range(0,ncol):
			pix_ = np.array([[Mi_2+i*Mi], [Ni_2+j*Ni], [1]])
			xyz  = inv_transf(pix_,K,cRw,ctw,ctw[2])
			x_mapa[0,i*nfil+j] = xyz[0]
			y_mapa[0,i*nfil+j] = xyz[1]

	#Mostramos por pantalla si lo deseamos
	if mostrar == True :
		print("X_mapa")
		print(x_mapa)
		print("Y_mapa")
		print(y_mapa)

	#Devolvemos en dos vectores nfil*ncol el resultado
	return x_mapa,y_mapa

def calc_coord_robot(x_r,y_r,M,N,nfil,ncol,mostrar):
	Mi = int(round(M/nfil))
	Mi_2 = int(round(Mi/2))
	Ni = int(round(N/ncol))
	Ni_2 = int(round(Ni/2))

	j_r = round((x_r - Ni_2)/Ni)
	i_r = round((y_r - Mi_2)/Mi)

	#Mostramos el resultado si lo deseamos
	if mostrar == True:
		print([i_r,j_r])

	#Devolvemos el resultado
	return i_r,j_r


def actualiza_coord_xy(data) :
	global xy_robot

	xy_robot = [0,0]
	xy_robot[0]=data.position.x 
	xy_robot[1]=data.position.y



def listener():
	global myCameraInfo
	global K
	global cRw
	global ctw
	global xy_robot
	global espera_inicial

	#Tamanio del mapa
	fil = 30
	col = 30

	myCameraInfo = CameraInfo()    
	rospy.init_node('mapper', anonymous=True)
	rospy.Subscriber("camera1/image_raw", Image, getImage)
	rospy.Subscriber('camera1/position_pix', Pose, actualiza_coord_xy)
	rate = rospy.Rate(30)

	#Publisher para enviar los datos
	mapa_pub=rospy.Publisher('mapper/mapa', Mapa, queue_size = 10)
	data = Mapa()

	#Espera de 5s al inicio de la simulacion
	if espera_inicial :
		time.sleep(4)

	time.sleep(1)
	#Calculo de las coordenadas X, Y de cada punto del mapa
	M,N,C = currentImage.shape
	x_mapa,y_mapa = xy_mapa(M,N,fil,col,False)

	while not rospy.is_shutdown():
        #Segmentamos la imagen actual
		im_bin = segmentacionBinaria(currentImage,240,(10,10),False)
		im_morf = morf(im_bin,1,1,False)

		#Discretizamos el mapa a la cuadricula
		mapa = discretizador(im_morf,fil,col,True)
		
		#Ponemos la cuadricula como vector para su envio
		mapa_vector = linealizador(mapa,fil,col)

		#Calculo de la posicion del robot en coordenadas de mapa
		i_r,j_r = calc_coord_robot(xy_robot[0],xy_robot[1],M,N,fil,col,False)
		
		#Enviamos los datos
		data.mapa = mapa_vector.astype(np.uint8).tolist()[0][:]
		data.mapa_x = x_mapa.astype(np.float32).tolist()[0][:]
		data.mapa_y = y_mapa.astype(np.float32).tolist()[0][:]
		data.nfil = fil
		data.ncol = col
		data.i_r = i_r
		data.j_r = j_r
		mapa_pub.publish(data)

		rate.sleep()


if __name__ == '__main__':
	listener()
