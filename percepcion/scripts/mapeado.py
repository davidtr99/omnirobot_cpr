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
		print("\nERROR CENTRO")



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

def discretizador(im,nfil,ncol,mostrar):
	mapa_res = np.zeros((nfil,ncol))
	M,N = im.shape
	Mi = round(M/nfil)
	Ni = round(N/ncol)

	for i in range(0,nfil):
		for j in range(0,ncol):
			bloque = np.add.reduce(im[i*Mi:(i+1)*Mi,j*Ni:(j+1)*Ni],None)
			if bloque != 0 :
				bloque = 1
			mapa_res[i,j]=bloque

	if mostrar == True:
		cv2.imshow("Map Image", mapa_res)
		cv2.waitKey(1)
	
	return mapa_res
			



def listener():
	global myCameraInfo
	global K
	global cRw
	global ctw
	myCameraInfo = CameraInfo()    
	rospy.init_node('mapper', anonymous=True)
	#rospy.Subscriber("camera1/camera_info", CameraInfo, newInfo)
	rospy.Subscriber("camera1/image_raw", Image, getImage)
	rate = rospy.Rate(30)

	#Publisher para enviar los datos
	posicion_pub=rospy.Publisher('camera1/position_orientation', Pose, queue_size = 10)
	data = Pose()

	while not rospy.is_shutdown():
        #Segmentamos la imagen actual
		im_bin = segmentacionBinaria(currentImage,240,(10,10),False)
		im_morf = morf(im_bin,1,1,True)
		mapa = discretizador(im_morf,30,30,True)

		
		rate.sleep()




if __name__ == '__main__':
	listener()
