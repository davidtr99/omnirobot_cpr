#!/usr/bin/env python
import rospy,sys,cv2,roslib

from math import sin,cos,sqrt,pow,atan2
from omnirobot_control.msg import Waypoint
import numpy as np

def listener():
	
	
	rospy.init_node('dosificador', anonymous=True)
	#Nos suscribimos al planificador
	#rospy.Subscriber('', Image, getImage) #Topic de salida del planificador
	rate = rospy.Rate(30)

	#Publisher para enviar los datos waypoints
	posicion_pub=rospy.Publisher('/dosificador/waypoint', Waypoint, queue_size = 10)
	data = Waypoint()

	while not rospy.is_shutdown():
		
		#Se incluyen los valores de posicion y orientacion en 'data', que tipo Pose()
		data.x = 1
		data.y = 2
		data.theta = 3

		posicion_pub.publish(data)

		
		rate.sleep()




if __name__ == '__main__':
	listener()
