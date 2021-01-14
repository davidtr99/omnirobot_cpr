#!/usr/bin/env python
import rospy,sys,cv2,roslib

from math import sin,cos,sqrt,pow,atan2
from omnirobot_control.msg import Waypoint,Flag
import numpy as np

pto = 0
nVector = 0
def actualizaPunto(data):
	global pto
	global nVector
	if (data.flag==1):
		if(pto >= nVector-1):
			pto = 0
		else:
			pto = pto + 1

def listener():
	global pto
	global nVector
	
	rospy.init_node('dosificador', anonymous=True)
	#Nos suscribimos al planificador
	rospy.Subscriber('/flag_check', Flag, actualizaPunto) #Topic de salida del planificador
	rate = rospy.Rate(30)

	#Publisher para enviar los datos waypoints
	posicion_pub=rospy.Publisher('/omnirobot_control/dosificador', Waypoint, queue_size = 10)
	data = Waypoint()

	#Codigo auxiliar: Tomamos lista de puntos (Cambiar por un topic)
	nVector = int(input("Numero de datos: " ))
	xi = np.zeros((nVector,1))
	yi = np.zeros((nVector,1))
	thetai = np.zeros((nVector,1))

	for i in range(1,nVector+1):
		strX = "X%d : "%i
		strY = "Y%d : "%i
		strTheta = "Theta%d : "%i

		xi[i-1,0] = float(input(strX))
		yi[i-1,0] = float(input(strY))
		thetai[i-1,0] = float(input(strTheta))

	while not rospy.is_shutdown():
		
		#Se incluyen los valores de posicion y orientacion en 'data', que tipo Pose()
		data.x = xi[pto]
		data.y = yi[pto]
		data.theta = thetai[pto]

		posicion_pub.publish(data)		
		rate.sleep()




if __name__ == '__main__':
	listener()
