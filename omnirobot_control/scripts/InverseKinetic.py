#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64 #Tipo de dato del controlador de velocidad.
from geometry_msgs.msg import Twist #Con el mensaje 'Twist', vamos a recibir del otro programa tanto velocidad linea, angular, como el angulo de giro theta
from math import pow, atan2, sqrt, cos, sin

rospy.set_param('/stop',1) #Con esta variable se puede parar el cacharro. Habra que de alguna manera sincronizarlo con el conrtrol a mas alto nivel, una vez se acabe el recorrido.

	#QUEDARIA PODER LEER TAMBIEN EL ANGULO DE GIRO CON RESPECTO AL EJE Z, Y SUSTITUIRO POR 'THETA' EN LAS FUNCIONES

class Omnirobot:

    def __init__(self):
	#Creamos un nodo unico:
        rospy.init_node('Cinematica_inversa', anonymous=True)

        # Publisher para la velocidad de cada rueda:
        self.velocity_right = rospy.Publisher('/omniRobot/right_joint_velocity_controller/command', Float64 , queue_size=10)
        self.velocity_left = rospy.Publisher('/omniRobot/left_joint_velocity_controller/command', Float64 , queue_size=10)
        self.velocity_back = rospy.Publisher('/omniRobot/back_joint_velocity_controller/command', Float64 , queue_size=10)
	
        # Subscriber para la senal de control
        self.control_signal = rospy.Subscriber('/omniRobot_controlsignal', Twist, self.control_update)

        self.vel_control = Twist()
        self.rate = rospy.Rate(10)
        
    def control_update(self, data):
        self.vel_control = data

    def velocidad_wx(self, theta=0): #Theta viene a indicar el angulo de giro con respecto al eje z. Es comun para sistema de referencia global y local. (PROVISIONAL)
        return cos(theta)*self.vel_control.linear.x + sin(theta)*self.vel_control.linear.y

    def velocidad_wy(self, theta=0):
        return -sin(theta)*self.vel_control.linear.x + cos(theta)*self.vel_control.linear.y

    def publish(self, L=1): #L es la distancia del centro de la base a una de las ruedas.
        right_vel = Float64()
        left_vel = Float64()
        back_vel = Float64()

	while (rospy.get_param('/stop')):

		left_vel.data = -(0.5*self.velocidad_wx()) - (sqrt(3)*self.velocidad_wy())/2 + L*self.vel_control.angular.z
		right_vel.data = self.velocidad_wx() + L*self.vel_control.angular.z
		back_vel.data = -(0.5*self.velocidad_wx()) + (sqrt(3)*self.velocidad_wy())/2 + L*self.vel_control.angular.z
		
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
	p.publish()
    except rospy.ROSInterruptException:
        pass

