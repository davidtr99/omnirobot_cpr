#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64 #Tipo de dato del controlador de velocidad
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from omnirobot_control.msg import Waypoint, Flag
from math import sin,cos,sqrt,pow,atan2,pi
import numpy as np

#Posicion actual (variable global)
global pose_act, waypoint
waypoint = Waypoint()
pose_act = Pose()

#Tolerancia
rospy.set_param('tolerancia', 0.01)

#Ganancia de los controladores
rospy.set_param('Kv', 1)
rospy.set_param('Kw', 3)

#Funcion que se llama para actualizar el punto final de la trayectoria
def update_waypoint(data):
	waypoint.x = data.x
	waypoint.y = data.y
	waypoint.theta = data.theta

#Funcion que actualiza la posicion actual de robot, desde la camara percepcion
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
	if (waypoint.theta > 0 and pose_act.orientation.w > 0):
		return waypoint.theta * (pi/180) - pose_act.orientation.w
	elif (waypoint.theta > 0 and pose_act.orientation.w < 0):
		if (waypoint.theta*(pi/180) - pose_act.orientation.w <= abs(waypoint.theta*(pi/180) - (pose_act.orientation.w + 2*pi))):
			return  waypoint.theta*(pi/180) - pose_act.orientation.w
		else:
			return waypoint.theta*(pi/180) - (pose_act.orientation.w + 2*pi)
	elif (waypoint.theta < 0 and pose_act.orientation.w > 0):
		if (waypoint.theta*(pi/180)+2*pi - pose_act.orientation.w <= abs(waypoint.theta*(pi/180)-pose_act.orientation.w)):
			return waypoint.theta*(pi/180)+2*pi - pose_act.orientation.w
		else:
			return waypoint.theta*(pi/180)-pose_act.orientation.w
	else:
		if (abs(waypoint.theta*(pi/180)-pose_act.orientation.w) <= (waypoint.theta*(pi/180)+2*pi)-pose_act.orientation.w):
			return waypoint.theta*(pi/180)-pose_act.orientation.w
		else:
			return (waypoint.theta*(pi/180)+2*pi)-pose_act.orientation.w

#Funcion que realiza el control
def control():
	global pose_act, waypoint
	check = Flag()
	check.flag = 1
	flag_check.publish(check) #Publicamos el flag a 1, para recibir el siguiente punto
	rate = rospy.Rate(30)

    #Creamos el mensaje que publicaremos
	controlsignal = Twist()

    #Mientras el error sea mayor que la tolerancia (0.1) aplicamos un control proporcional
	controlsignal.linear.x = 0
	controlsignal.linear.z = 0
	controlsignal.angular.x = 0
	controlsignal.angular.y = 0

	rate.sleep() #Pequeño 'sleep' para asegurarnos de que recibimos el nuevo punto a tiempo

	while((abs(error_dist(waypoint)) >= rospy.get_param('tolerancia')) or (abs(error_ang(waypoint)) >= rospy.get_param('tolerancia'))):

		controlsignal.angular.z = rospy.get_param('Kw')*error_ang(waypoint)
		controlsignal.linear.x = rospy.get_param('Kv')*(error_dist(waypoint)*cos(atan2(waypoint.y - pose_act.position.y,waypoint.x-pose_act.position.x)-pose_act.orientation.w))
		controlsignal.linear.y = rospy.get_param('Kv')*(error_dist(waypoint)*sin(atan2(waypoint.y - pose_act.position.y,waypoint.x-pose_act.position.x)-pose_act.orientation.w))

		print("error angular: ", error_ang(waypoint),"\tsenal de control: ", controlsignal.angular.z)
		print("error lineal: ", error_dist(waypoint),"\tsenal de control: ", controlsignal.linear.y)

		kinetic.publish(controlsignal)
		rate.sleep()
		
if __name__ == '__main__':

	#Inicio del nodo del control propomnidireccional (CPAL) 
	rospy.init_node('control_omnirobot',anonymous=True)
	
	#Subscriptor del topic de la camara
	pos_sub = rospy.Subscriber('camera1/position_orientation', Pose, posicion)

	#Subscriptor del topic del dosificador de puntos
	points_sub = rospy.Subscriber('omnirobot_control/dosificador', Waypoint, update_waypoint) 
	
	#Publisher del controlador, para el movimiento del robot
	kinetic = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

	#Publisher del flag de esta para el dosificador
	flag_check = rospy.Publisher('/flag_check', Flag, queue_size=10)

	#Llamamos a la funcion de control
	while(True):
		control()