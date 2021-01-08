#!/usr/bin/env python
import rospy,sys,cv2,roslib

from math import sin,cos,sqrt,pow,atan2
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
import numpy as np

bridge = CvBridge()

currentImage = np.zeros((256,256,3),dtype="uint8")

#Matriz de parametros intrinsecos de la camara
K  = np.array([[476.7030836014194, 0.0, 400.5], [0.0, 476.7030836014194, 400.5], [0.0, 0.0, 1.0]])

#Rotacion y traslacion
phi = np.pi
psi = -np.pi/2
Rx = np.array([[1, 0, 0],[0, cos(phi), -sin(phi)], [0,  sin(phi),  cos(phi)]])
Rz = np.array([[cos(psi), -sin(psi), 0],[sin(psi),  cos(psi), 0], [0,  0,  1]])
cRw = np.dot(Rz,Rx)
ctw = np.array([[0],[0],[1]])

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
	imagen = imagenOR

	## BINARIZACION Y APLICACION DE CRITERIO DE COLOR EN HSV
	hmin = h_color - th[0]
	hmax = h_color + th[1]
	frameHSV = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
	frameThreshold = cv2.inRange(frameHSV,(hmin/360.0*179,0,0),(hmax/360.0*179,255,255))
	

	## CALCULO DEL CENTRO DEL BLOB
	# Usamos los momentos de CV2 para su calculo mas rapido
	M = cv2.moments(frameThreshold)
	if(M["m00"] != 0):
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
	else:
		cX, cY = 0, 0
		print("\nERROR CENTRO")



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

def pintarCentros(imagenOR,centros):
	imagen = imagenOR
	for cX, cY in centros:
		cv2.circle(imagen, (cX, cY), 2, (255, 255, 255), -1)

	cv2.imshow("Image", currentImage)
	cv2.waitKey(3)

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
	myCameraInfo = CameraInfo()    
	rospy.init_node('lector', anonymous=True)
	#rospy.Subscriber("camera1/camera_info", CameraInfo, newInfo)
	rospy.Subscriber("camera1/image_raw", Image, getImage)
	rate = rospy.Rate(10)

	#Publisher para enviar los datos
	posicion_pub=rospy.Publisher('camera1/position_orientation', Pose, queue_size = 10)
	data = Pose()

	while not rospy.is_shutdown():
		#Orientacion: Angulo entre eje "y" del robot (direccion de avance) y el eje "x" [0,2*pi]
		centroX, centroY = getPosicion(currentImage,30,(10,15), False)
		traseraX, traseraY = getPosicion(currentImage,122,(10,10), False)
		pintarCentros(currentImage,[(centroX,centroY),(traseraX,traseraY)])
		orientacion = np.pi + atan2(float(centroX-traseraX),float(centroY-traseraY))
		if orientacion > np.pi :
			orientacion = orientacion - 2*np.pi

		pix_ = np.array([[centroX], [centroY], [1]])
		xyz  = inv_transf(pix_,K,cRw,ctw,1)
		print("Posicion: " + str(xyz[0]) + " - " + str(xyz[1]) + "\t//\t"  + "\t" + str(orientacion))
		
		#Se incluyen los valores de posicion y orientacion en 'data', que tipo Pose()
		data.position.x = xyz[0]
		data.position.y = xyz[1]
		data.position.z = xyz[2]
		data.orientation.w = orientacion
		posicion_pub.publish(data)

		
		rate.sleep()




if __name__ == '__main__':
	listener()
