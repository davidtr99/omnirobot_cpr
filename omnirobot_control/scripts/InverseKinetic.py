#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64 #Tipo de dato del controlador de velocidad
from math import pow, atan2, sqrt

rospy.set_param('/Vx',10) #Esto seria lo que obtiene de suscribirse al nodo del control de alto nivel
rospy.set_param('/Vy',0) #Las ecuaciones estan referidas al marco del bicho electro-mecanico (robot, para los mas puristas)
rospy.set_param('/omega',0)

rospy.set_param('/stop',1) #Con esta variable se puede parar el cacharro


class Omnirobot:

    def __init__(self):
	#Creamos un nodo unico:
        rospy.init_node('Cinematica_inversa', anonymous=True)

        # Publisher para la velocidad de cada rueda:
        self.velocity_right = rospy.Publisher('/omniRobot/right_joint_velocity_controller/command', Float64 , queue_size=10)
	self.velocity_left = rospy.Publisher('/omniRobot/left_joint_velocity_controller/command', Float64 , queue_size=10)
	self.velocity_back = rospy.Publisher('/omniRobot/back_joint_velocity_controller/command', Float64 , queue_size=10)

        self.rate = rospy.Rate(10)	

    def publish(self, L=40*10^(-6)):
	right_vel = Float64()
	left_vel = Float64()
	back_vel = Float64()

	while (rospy.get_param('/stop')):

		left_vel.data = -(rospy.get_param('/Vx')/2) - (sqrt(3)*rospy.get_param('/Vy'))/2 + L*rospy.get_param('/omega')
		back_vel.data = rospy.get_param('/Vx') + L*rospy.get_param('/omega')
		right_vel.data = -(rospy.get_param('/Vx')/2) + (sqrt(3)*rospy.get_param('/Vy'))/2 + L*rospy.get_param('/omega')
		
		#Publicamos los valores actualizados
		self.velocity_right.publish(right_vel)
		self.velocity_left.publish(left_vel)
		self.velocity_back.publish(back_vel)
		self.rate.sleep() #Hacemos el muestreo

	#Paramos las ruedas
	left_vel.data = 0 
	right_vel.data = 0
	back_vel.data = 0
	self.velocity_right.publish(right_vel)
	self.velocity_left.publish(left_vel)
	self.velocity_back.publish(back_vel)


if __name__ == '__main__':
    try:
	p = Omnirobot()
	p.publish();
    except rospy.ROSInterruptException:
        pass

