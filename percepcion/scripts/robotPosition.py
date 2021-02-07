#!/usr/bin/env python
import rospy,sys,cv2,roslib

from math import sin,cos,sqrt,pow,atan2
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
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

#Obtencion de la imagen de la camara
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

#Obtencio de la posicion del robot en pixeles
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
		cv2.waitKey(3)

	return cX, cY

#Funcion para pintar en la imagen en centro
def pintarCentros(imagenOR,centros,angulo):
	imagen = imagenOR.copy()
	for cX, cY in centros:
		cv2.circle(imagen, (cX, cY), 2, (255, 255, 255), -1)

	finalx, finaly = centros[0]
	finalx += 30*cos(angulo)
	finaly += 30*sin(angulo)
	imagen = cv2.arrowedLine(imagen, centros[0], (int(finalx),int(finaly)), (0,255,255), 1) 
	cv2.imshow("Image", imagen)
	cv2.waitKey(1)

def inv_transf(pix_,K,R,t,z):
	xyz = z*np.dot(np.dot(np.transpose(R),np.linalg.inv(K)),pix_) - np.dot(np.transpose(R), t)
	
	"""
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

def listener():
	global myCameraInfo
	global K
	global cRw
	global ctw
	global espera_inicial
	myCameraInfo = CameraInfo()    
	rospy.init_node('robotPosition', anonymous=True)
	rospy.Subscriber("camera1/image_raw", Image, getImage)
	rate = rospy.Rate(30)

	#Publisher para enviar los datos
	posicion_pub=rospy.Publisher('camera1/position_orientation', Pose, queue_size = 10)
	data = Pose()

	#Publisher para enviar centro en pixeles del robot al mapeado
	pix_pub = rospy.Publisher('camera1/position_pix',Pose,queue_size = 10)
	data_pix = Pose()

	#Espera de 1s al inicio de todo
	if espera_inicial :
		time.sleep(4)

	while not rospy.is_shutdown():
		
		# Sacamos los dos puntos necesarios con segmentacion
		centroX, centroY = getPosicion(currentImage,30,(10,15), False)
		
		delanteraX, delanteraY = getPosicion(currentImage,122,(10,10), False)

		#Orientacion: Angulo entre eje "x" del robot (direccion de avance) y el eje "x" [0,2*pi]
		orientacion = atan2(float(delanteraY-centroY),float(delanteraX-centroX))
		
		#Enviamos el centro en pix (necesario para mapeado)
		data_pix.position.x = centroX
		data_pix.position.y = centroY
		pix_pub.publish(data_pix)

		#Pintamos el centro de la imagen
		pintarCentros(currentImage,[(centroX,centroY),(delanteraX,delanteraY)],orientacion)
		
		#Orientacion calculada
		orientacion = -orientacion

		#Calculo de la transformada inversa
		pix_ = np.array([[centroX], [centroY], [1]])
		xyz  = inv_transf(pix_,K,cRw,ctw,ctw[2])
		
		#Se incluyen los valores de posicion y orientacion en 'data', que tipo Pose()
		data.position.x = xyz[0]
		data.position.y = xyz[1]
		data.position.z = xyz[2]
		data.orientation.w = orientacion
		posicion_pub.publish(data)

		rate.sleep()


if __name__ == '__main__':
	listener()
