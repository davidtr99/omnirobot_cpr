#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64 #Tipo de dato del controlador de velocidad
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from omnirobot_control.msg import Waypoint
from math import sin,cos,sqrt,pow,atan2,pi
import numpy as np
import time

#Posicion actual (variable global)
global pose_act, waypoint, err_integral, last_error, previous_time
last_error = 0
waypoint = Waypoint()
pose_act = Pose()
err_integral = 0

#Tolerancia
rospy.set_param('tolerancia', 0.05)

#Ganancia de los controladores
rospy.set_param('Kp', 1)
rospy.set_param('Ki', 0.001)
rospy.set_param('Kd', 0.1)
rospy.set_param('Kw', 3)

#Funcion para calcular el tiempo transcurrido en milisegundos
def millis():
	return round(time.time() * 1000)

previous_time = millis()

#Funcion que se llama para actualizar el punto final de la trayectoria
def update_waypoint(data):
	global waypoint, err_integral, last_error
	waypoint.x = data.x
	waypoint.y = data.y
	waypoint.theta = data.theta
	err_integral = 0 #Reiniciamos la acumulacion del error integral cada vez que llegue un nuevo punto
	last_error = 0 #Reiniciamos el error anterior para el nuevo punto

#Funcion que actualiza la posicion actual de robot, desde la camara percepcion
def posicion(data):
    global pose_act
    pose_act.position.x = data.position.x
    pose_act.position.y = data.position.y
    pose_act.position.z = data.position.z
    pose_act.orientation.w = data.orientation.w

#Funcion que calcula la distancia desde la posicion actual hasta el destino (waypoint)
def error_dist(waypoint):
	global pose_act, err_integral, last_error, previous_time
	currentTime = millis()
	elapsed_time = currentTime - previous_time
	error = sqrt(pow((waypoint.x - pose_act.position.x), 2) + pow((waypoint.y - pose_act.position.y), 2))
	err_integral = err_integral + error*elapsed_time
	err_derivativo = (error - last_error) / elapsed_time
	previous_time = currentTime
	last_error = error
	return rospy.get_param('Kp')*error + rospy.get_param('Ki')*err_integral + rospy.get_param('Kd')*err_derivativo

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
	
	rate = rospy.Rate(30)

    #Creamos el mensaje que publicaremos
	controlsignal = Twist()

    #Mientras el error sea mayor que la tolerancia aplicamos un control proporcional
	controlsignal.linear.x = 0
	controlsignal.linear.z = 0
	controlsignal.angular.x = 0
	controlsignal.angular.y = 0

	rate.sleep() #Pequeno 'sleep' para asegurarnos de que recibimos el nuevo punto a tiempo

	while((abs(error_dist(waypoint)) >= rospy.get_param('tolerancia')) or (abs(error_ang(waypoint)) >= rospy.get_param('tolerancia'))):

		controlsignal.angular.z = rospy.get_param('Kw')*error_ang(waypoint)
		controlsignal.linear.x = error_dist(waypoint)*(cos(atan2(waypoint.y - pose_act.position.y,waypoint.x-pose_act.position.x)-pose_act.orientation.w))
		controlsignal.linear.y = error_dist(waypoint)*(sin(atan2(waypoint.y - pose_act.position.y,waypoint.x-pose_act.position.x)-pose_act.orientation.w))

		print("error angular: ", error_ang(waypoint),"\tsenal de control: ", controlsignal.angular.z)
		print("error lineal: ", error_dist(waypoint),"\tsenal de control: ", controlsignal.linear.x , " " , controlsignal.linear.y)

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

	#Llamamos a la funcion de control
	while(True):
		control()