#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from duckietown_msgs.msg import Twist2DStamped
from geometry_msgs.msg import Twist, Point # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy


class Nodo(object):
	def __init__(self, args):
		super(Nodo, self).__init__()
		self.args = args
		self.subPosicion= rospy.Subscriber('/duckiebot/Posicion', Point, self.posicion)
		self.subcmd = rospy.Subscriber("/duckiebot/possible_cmd",Twist2DStamped, self.joystick)
		self.pub = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd",Twist2DStamped,queue_size=1)
		self.new_msg = Twist2DStamped()
		self.Joy_msg = Twist2DStamped()
	#def publicar(self):

	def joystick(self,msg):
		self.Joy_msg = msg

	def posicion(self,msg):
		if msg.z<20 and msg.z>0:
			self.new_msg.v = 0
	                self.new_msg.omega = 0
			#print("controller:",msg.z)
		else:
			self.new_msg= self.Joy_msg
		
		self.pub.publish(self.new_msg)
	#def procesar_img(self, img):
		# Cambiar espacio de color

		# Filtrar rango util

		# Aplicar mascara

		# Aplicar transformaciones morfologicas

		# Definir blobs

		# Dibujar rectangulos de cada blob

		# Publicar imagen final


def main():
	rospy.init_node('Controller') #creacion y registro del nodo!

	obj = Nodo('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
