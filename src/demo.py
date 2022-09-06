


#!/usr/bin/env python

import rospy #importar ros para python
import numpy as np
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist


class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.sub = rospy.Subscriber("/duckiebot/joy", Joy, self.test)
		self.pub = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped)
		self.twist = Twist2DStamped()

	def publicar(self, msg):
		self.pub.publish(msg)

	def test(self,msg):
		x = msg.axes[0]
		y = msg.axes[1]
		ejes=[]
                B=(msg.buttons)[1]
                if B==1:
                        ejes=[0,0,0,0,0,0,0,0]
                        print(ejes)
		else:
			print("x a [0, 1]: " + str(mapeo(x, -1, 1, 0, 1)))
			print("x a [-pi/2, pi/2]: " + str(mapeo(x, -1, 1, -np.pi/2, np.pi/2)))
			print("x a [-10, 10]: " + str(mapeo(x, -1, 1, -10, 10)))
			print( "y a [0, 1]: "+ str(mapeo(y, -1, 1, 0, 1)))
			print("y a [-pi/2, pi/2]: " + str(mapeo(y, -1, 1, -np.pi/2, np.pi/2)))
                	print("y a [-10, 10]: " + str(mapeo(y, -1, 1, -10, 10)))

		new_msg = Twist2dStamped()
		new_msg.data = str(mapeo(x, -1, 1, 0, 1))
		self.publicar(new_msg)

def mapeo(valor, inf, sup, out_inf, out_sup):
	return np.interp(valor, [inf, sup], [out_inf, out_sup])

def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
