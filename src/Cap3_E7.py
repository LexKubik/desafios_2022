#!/usr/bin/env python

import rospy #importar ros para python
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Joy

pi=3.14159265


class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.sub = rospy.Subscriber("/duckiebot/joy",Joy,self.callback)
		self.pub = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped,queue_size=10)
		self.new_msg = Twist2DStamped()	
	def publicar(self,msg):
		self.pub.publish(msg)
	
	def callback(self,msg):
		B=(msg.buttons)[1]
		if B==1:
			x = 0
			y = 0
		else:
			eje0=(msg.axes)[0]
			eje1=(msg.axes)[1]
		
			eje0p1=(eje0+1)/2
			eje1p1=(eje1+1)/2
			# print("\nEje 0, rango [0,1]: "+ str(eje0p1))
			# print('Eje 1, rango [0,1]: '+ str(eje1p1))
		
			eje0p2=eje0*(pi/2)
			eje1p2=eje1*(pi/2)
			# print("\nEje 0, rango [-pi/2,pi/2]: "+ str(eje0p2))
			# print("Eje 1, rango [-pi/2,pi/2]: "+ str(eje1p2))
		
			x=10*eje0
			y=eje1
			# print("\nEje 0, rango [-10,10]: "+ str(eje0p3))
			# print("Eje 1, rango [-10,10]: "+ str(eje1p3))
		
		self.new_msg.v = y
                self.new_msg.omega = -x
                self.pub.publish(self.new_msg)		

def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba
#duckiebot@duckiebot:o obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
