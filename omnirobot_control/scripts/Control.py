#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64 #Tipo de dato del controlador de velocidad
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from math import sin,cos,sqrt,pow,atan2,pi

#Posicion actual (variable global)
global pose_act
pose_act = Pose()

#Tolerancia
rospy.set_param('tolerancia', 0.01)

#Ganancia de los controladores
rospy.set_param('Kv', 1)
rospy.set_param('Kw', 3)

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
	if (waypoint.z > 0 and pose_act.orientation.w > 0):
		return waypoint.z * (pi/180) - pose_act.orientation.w
	elif (waypoint.z > 0 and pose_act.orientation.w < 0):
		if (waypoint.z*(pi/180) - pose_act.orientation.w <= abs(waypoint.z*(pi/180) - (pose_act.orientation.w + 2*pi))):
			return  waypoint.z*(pi/180) - pose_act.orientation.w
		else:
			return waypoint.z*(pi/180) - (pose_act.orientation.w + 2*pi)
	elif (waypoint.z < 0 and pose_act.orientation.w > 0):
		if (waypoint.z*(pi/180)+2*pi - pose_act.orientation.w <= abs(waypoint.z*(pi/180)-pose_act.orientation.w)):
			return waypoint.z*(pi/180)+2*pi - pose_act.orientation.w
		else:
			return waypoint.z*(pi/180)-pose_act.orientation.w
	else:
		if (abs(waypoint.z*(pi/180)-pose_act.orientation.w) <= (waypoint.z*(pi/180)+2*pi)-pose_act.orientation.w):
			return waypoint.z*(pi/180)-pose_act.orientation.w
		else:
			return (waypoint.z*(pi/180)+2*pi)-pose_act.orientation.w

#Funcion que realiza el control
def control():
	global pose_act

	waypoint = Point()
	rate = rospy.Rate(30)

    #Creamos el mensaje que publicaremos
	controlsignal = Twist()

    #Mientras el error sea mayor que la tolerancia (0.1) aplicamos un control proporcional
	controlsignal.linear.x = 0
	controlsignal.linear.z = 0
	controlsignal.angular.x = 0
	controlsignal.angular.y = 0

	#Pedimos las coordenas del punto destino
	waypoint.x = float(input("x: "))
	waypoint.y = float(input("y: "))
	waypoint.z = float(input("theta([-180,180]): ")) #Angulo de orientacion con el que llega al punto final
	while((abs(error_dist(waypoint)) >= rospy.get_param('tolerancia')) or (abs(error_ang(waypoint)) >= 0.06)):

		controlsignal.angular.z = rospy.get_param('Kw')*error_ang(waypoint)
		controlsignal.linear.x = rospy.get_param('Kv')*(error_dist(waypoint)*cos(atan2(waypoint.y - pose_act.position.y,waypoint.x-pose_act.position.x)-pose_act.orientation.w))
		controlsignal.linear.y = rospy.get_param('Kv')*(error_dist(waypoint)*sin(atan2(waypoint.y - pose_act.position.y,waypoint.x-pose_act.position.x)-pose_act.orientation.w))

		print("error angular: ", error_ang(waypoint),"\tsenal de control: ", controlsignal.angular.z)
		print("error lineal: ", error_dist(waypoint),"\tsenal de control: ", controlsignal.linear.y)

		kinetic.publish(controlsignal)
		rate.sleep()
		
if __name__ == '__main__':

	#inicio el nodo control tortuga
	rospy.init_node('control_omnirobot',anonymous=True)
	
	#Subscriptor del topic de la camara
	pos_sub = rospy.Subscriber('camera1/position_orientation', Pose, posicion)
	
	#Publisher del InverseKinetic, para enviarselo al control mas bajo
	kinetic = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

	#llamamos a la funcion de control
	while(True):
		control()

