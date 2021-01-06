#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64 #Tipo de dato del controlador de velocidad
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from math import sin,cos,sqrt,pow,atan2

#Posicion actual (variable global)
global pose_act
pose_act = Pose()

#Tolerancia
rospy.set_param('tolerancia', 0.1)
rospy.set_param('Kv', 1.5)
rospy.set_param('Kw', 4)

#Funcion que se llama cuando el suscriptor recibe un nuevo mensaje de tipo Pose
def posicion(data):
    global pose_act
    pose_act.position.x = data.pose[1].position.x
    pose_act.position.y = data.pose[1].position.y
    pose_act.position.z = data.pose[1].position.z
    pose_act.orientation.w = data.pose[1].orientation.w

#Funcion que calcula la distancia desde la posicion actual hasta el destino (waypoint)
def error_dist(waypoint):
	global pose_act

	return sqrt(pow((waypoint.x - pose_act.position.x), 2) + pow((waypoint.y - pose_act.position.y), 2))

#Funcion que calcula el angulo que tiene que girar
def error_ang(waypoint):
	global pose_act

	return atan2(waypoint.y - pose_act.position.y, waypoint.x - pose_act.position.x) - (pose_act.orientation.w - 2.0943951)

#Funcion que realiza el control
def control():

	waypoint = Point()
    
    #Pedimos las coordenas del punto destino
        waypoint.x = float(input("x: "))
        waypoint.y = float(input("y: "))

    #Creamos el mensaje que publicaremos
	controlsignal = Twist()

	controlsignal.linear.x = 0
	controlsignal.linear.z = 0
	controlsignal.angular.x = 0
	controlsignal.angular.y = 0

    #Mientras el error sea mayor que la tolerancia (0.1) aplicamos un control proporcional
	while error_dist(waypoint) >= rospy.get_param('tolerancia'):
		
        	controlsignal.linear.y = rospy.get_param('Kv')*error_dist(waypoint)
        	controlsignal.angular.z = rospy.get_param('Kw')*error_ang(waypoint)

		kinetic.publish(controlsignal)
        	rate.sleep()

	rospy.spin()

if __name__ == '__main__':

	#inicio el nodo control tortuga
	rospy.init_node('control_omnirobot',anonymous=True)
	
	#Subscriptor del topic
	pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, posicion)
	
	#Publisher del InverseKinetic, para enviarselo al control mas bajo
	kinetic = rospy.Publisher('/omniRobot_controlsignal', Twist, queue_size = 10)

	#frecuencia de publicacion
	rate = rospy.Rate(10)

	#llamamos a la funcion de control
	control()

