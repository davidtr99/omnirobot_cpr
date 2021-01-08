#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64 #Tipo de dato del controlador de velocidad
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from math import sin,cos,sqrt,pow,atan2

#Posicion actual (variable global)
global pose_act
pose_act = Pose()

#Tolerancia
rospy.set_param('tolerancia', 0.01)
rospy.set_param('Kv', 2)
rospy.set_param('Kw', 10)

#Funcion que se llama cuando el suscriptor recibe un nuevo mensaje de tipo Pose
def posicion(data):
    global pose_act
    pose_act.position.x = data.position.x
    pose_act.position.y = data.position.y
    pose_act.position.z = data.position.z
    pose_act.orientation.w = data.orientation.w

#Funcion que calcula la distancia desde la posicion actual hasta el destino (waypoint)
def error_dist(waypoint):
	global pose_act
	return sqrt(pow((waypoint.x - pose_act.position.x), 2) + pow((waypoint.y - pose_act.position.y), 2))

#Funcion que calcula el angulo que tiene que girar
def error_ang(waypoint):
	global pose_act
	return atan2(waypoint.y - pose_act.position.y, waypoint.x - pose_act.position.x) - (pose_act.orientation.w)

#Funcion que realiza el control
def control():

	waypoint = Point()
    	rate = rospy.Rate(10)
    #Pedimos las coordenas del punto destino
        waypoint.x = float(input("x: "))
        waypoint.y = float(input("y: "))

    #Creamos el mensaje que publicaremos
	controlsignal = Twist()

	controlsignal.linear.x = 0
	controlsignal.linear.z = 0
	controlsignal.angular.x = 0
	controlsignal.angular.y = 0

	print(error_ang(waypoint))

    #Mientras el error sea mayor que la tolerancia (0.1) aplicamos un control proporcional
	while (True):
		while abs(error_ang(waypoint)) >= rospy.get_param('tolerancia'):
		
	        	controlsignal.angular.z = rospy.get_param('Kw')*error_ang(waypoint)
			print('error angular: ',error_ang(waypoint))
			print('senal de control: ',controlsignal.angular.z)

			kinetic.publish(controlsignal)
			rate.sleep()

if __name__ == '__main__':

	#inicio el nodo control tortuga
	rospy.init_node('control_omnirobot',anonymous=True)
	
	#Subscriptor del topic de la camara
	pos_sub = rospy.Subscriber('camera1/position_orientation', Pose, posicion)
	
	#Publisher del InverseKinetic, para enviarselo al control mas bajo
	kinetic = rospy.Publisher('/omniRobot_controlsignal', Twist, queue_size = 10)

	#frecuencia de publicacion
	rate = rospy.Rate(10)

	#llamamos a la funcion de control
	control()

