#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy

class Nodo(object):
	def __init__(self, args):
		super(Nodo, self).__init__()
		self.args = args
		self.sub = rospy.Subscriber("/duckiebot/camera_node/image/rect", Image, self.callback)
		
		self.detector = cv2.CascadeClassifier("/home/duckiebot/duckietown/catkin_ws/src/desafios_2022/src/cascade3_LBP_.xml")

		self.pub = rospy.Publisher("/duckiebot/detecciones", Image, queue_size=10)
		self.bridge = CvBridge()

	#def publicar(self):

	def callback(self,msg):

		image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		image_out_gris = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		msg_gris = self.bridge.cv2_to_imgmsg(image_out_gris, "mono8")
		#self.pub1.publish(msg_gris)
		
		dets = detector.detectMultiScale(image_out_gris, 1.3 , 10)
		
		for patos in dets:
			x,y,w,h=patos
                	if w*h>400:
				cv2.rectangle(image, (x,y), (x+w,y+h), (0,0,255), 2)
		
		msg = self.bridge.cv2_to_imgmsg(image, "bgr8")	 
		self.pub.publish(msg)
		
def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Nodo('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	#rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
