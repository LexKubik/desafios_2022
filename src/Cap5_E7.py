#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy


class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.sub= rospy.Subscriber('/duckiebot/camera_node/image/raw', Image, self.callback)
		self.pub = rospy.Publisher("/duckiebot/mascara", Image, queue_size=10)
		#self.pub1 = rospy.Publisher("/duckiebot/patogris", Image, queue_size=10)
		#self.pub2 = rospy.Publisher("/duckiebot/patohsv", Image, queue_size=10)
		#self.pub3 = rospy.Publisher("/duckiebot/patobgr", Image, queue_size=10)
		self.bridge = CvBridge()
		self.minim = 1000
	#def publicar(self):

	def callback(self,msg):

		image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		#image_out_gris = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		#msg_gris = self.bridge.cv2_to_imgmsg(image_out_gris, "mono8")
		#self.pub1.publish(msg_gris)
		
		image_out_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		#msg_hsv = self.bridge.cv2_to_imgmsg(image_out_hsv, "bgr8")
		#self.pub2.publish(msg_hsv)
		
		#image_out_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
		#msg_bgr = self.bridge.cv2_to_imgmsg(image_out_bgr, "bgr8")
		#self.pub3.publish(msg_bgr)
		
		lower_limit = np.array([15, 100,100])
		upper_limit = np.array([50, 255, 255])

		mask = cv2.inRange(image_out_hsv, lower_limit, upper_limit) 

		kernel = np.ones((5,5),np.uint8)
		mask = cv2.erode(mask, kernel, iterations=1)
		mask = cv2.dilate(mask, kernel, iterations=1)
		_,contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		image_out = cv2.bitwise_and(image, image, mask= mask)

		for cnt in contours:
			x,y,w,h=cv2.boundingRect(cnt)
			if w*h>400:
                		cv2.rectangle(image_out, (x,y), (x+w,y+h), (0,0,255), 2)

		msg = self.bridge.cv2_to_imgmsg(image_out, "bgr8")	

		self.pub.publish(msg)
		
	#def procesar_img(self, img):
		# Cambiar espacio de color

		# Filtrar rango util

		# Aplicar mascara

		# Aplicar transformaciones morfologicas

		# Definir blobs

		# Dibujar rectangulos de cada blob

		# Publicar imagen final


def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
