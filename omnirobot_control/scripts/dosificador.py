#!/usr/bin/env python
import rospy,sys,cv2,roslib

from math import sin,cos,sqrt,pow,atan2
from omnirobot_control.msg import Waypoint,Flag,V_waypoint
from geometry_msgs.msg import Pose
import numpy as np

pto = 0
nVector = 0
pose_act = Pose()
flag = 0
#xi = np.array([0,0.5,1,1.5,2,2.5,2.5,2.5,2.5,2.5])
#yi = np.array([0,0,0,0,0,0,0.5,1,1.5,2])

#Orientacion del robot -> rosparam
rospy.set_param('angle_robot',0)

#Funcion con los puntos de la trayectoria a seguir
def vector_waypoints(data):
	global xi
	global yi
	global thetai
	global nVector
	global flag
	global pto

	flag = 1
	xi = np.array(data.x)
	yi = np.array(data.y)
	nVector = data.tam
	pto = 0 # Resetamos el inicio si replanifica o le llega una nueva trayectoria
	thetai = np.ones((nVector,1))*rospy.get_param('angle_robot')

def distancia(x,y):
	return sqrt(pow((x - pose_act.position.x), 2) + pow((y - pose_act.position.y), 2))

#Funcion que actualiza la posicion actual de robot, desde la camara percepcion
def posicion(data):
    global pose_act
    pose_act.position.x = data.position.x
    pose_act.position.y = data.position.y
    pose_act.position.z = data.position.z
    pose_act.orientation.w = data.orientation.w

def listener():
	global pto
	global nVector
	global xi, yi,thetai
	global flag

	LookAhead = 0.3
	
	rospy.init_node('dosificador', anonymous=True)
	rate = rospy.Rate(30)

	#Subscriptor del topic de la camara
	pos_sub = rospy.Subscriber('camera1/position_orientation', Pose, posicion)

	#Subscriptor del topic de A*
	rospy.Subscriber('planificador/trayectoria',V_waypoint,vector_waypoints)

	#Publisher para enviar los datos waypoints
	posicion_pub=rospy.Publisher('/omnirobot_control/dosificador', Waypoint, queue_size = 10)
	data = Waypoint()

	#Codigo auxiliar: Tomamos lista de puntos (Cambiar por un topic)
	
	
	#Esperar a que le llegue
	flag = 0
	while(flag == 0):
		rate.sleep()
	
	while not rospy.is_shutdown():
		#Actualizamos la orientacion del robot
		thetai = np.ones((nVector,1))*rospy.get_param('angle_robot')
		
		#Se incluyen los valores de posicion y orientacion en 'data', de tipo Pose()
		data.x = xi[pto]
		data.y = yi[pto]
		data.theta = thetai[pto]
		
		if (LookAhead > distancia(xi[pto],yi[pto])):
			if(pto >= nVector-1):
				pto = pto
			else:
				pto = pto + 1

		posicion_pub.publish(data)		
		rate.sleep()


if __name__ == '__main__':
	listener()
