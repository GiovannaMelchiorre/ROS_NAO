#!/usr/bin/env python

import rospy
import random
import sys
import signal

import math
import json

from sensor_msgs.msg import PointCloud
from std_msgs.msg import String


class pointsManager():
	
	def __init__(self):
		rospy.loginfo("nodo inizi")
		

	def listener(self):

 
        	rospy.Subscriber("PointCloudChannel", PointCloud, self.points_callback)
		rospy.spin()

        def createJson(self, data,nome):
	
		with open('/home/giovanna/catkin_ws/src/astra_camera_pkg/JSON/'+nome+'.txt', 'w') as outfile:
		    json.dump(data , outfile)
	    
	

	def points_callback(self,data):
		rospy.loginfo("ciao")
		#self.createJson(str(data.points), "provaStringa")
		self.pub = rospy.Publisher('stringozza', String, queue_size=5)
                pointsStr = String()
		
		stringaPunti=""
		
		count=0
		
		while(count<3):
			if(count == 1):	
				rospy.loginfo(count)
				#mette in prec il primo vettore diverso da nan
				for i in range(0,len(data.points)):
					if (math.isnan(data.points[i].x) == False):
						prec = data.points[i]
						break

				for i in range(0,len(data.points)):
					rospy.loginfo(i)

					if( math.isnan(data.points[i].x)==False and data.points[i].z <= 2 and ((round(data.points[i].x, 1)-round(prec.x,1) != 0) or (round(data.points[i].y, 1)-round(prec.y,1) != 0) or (round(data.points[i].z, 1)-round(prec.z,1) != 0))):		
						prec = data.points[i]
						stringaPunti = stringaPunti + str(data.points[i].x) + " "  + str(data.points[i].y) + " "+ str(data.points[i].z) + ","
				
				rospy.loginfo("ciao")	
				self.createJson(stringaPunti, "provaStringa")
				pointsStr.data = stringaPunti
                		self.pub.publish(pointsStr)
				rospy.sleep(1000.)			
			count = count +1
                

	def listenerProva(self):
		self.pub = rospy.Publisher('stringozza', String, queue_size=5)
                pointsStr = String()
		stringozza="ciao"
		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
	                pointsStr.data = stringozza
	                self.pub.publish(pointsStr)
			rate.sleep()


if __name__ == '__main__':
	try:
		
	    points = pointsManager()
	    rospy.init_node('listenerPC', anonymous=True)
	    points.listener()
		

	except rospy.ROSInterruptException:
	    pass
