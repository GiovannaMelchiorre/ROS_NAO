#!/usr/bin/env python

import rospy
import random
import sys
import signal

import math
import json

from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
import numpy as np
import pylab

class pointsManager():
	
	def __init__(self):
		rospy.loginfo("nodo inizial")
		

	def listener(self):

 
        	rospy.Subscriber("PointCloudChannel", PointCloud, self.analysis)
		rospy.spin()

        def createJson(self, data,nome):
	
		with open('/home/giovanna/catkin_ws/src/astra_camera_pkg/JSON/'+nome+'.txt', 'w') as outfile:
		    json.dump(data , outfile)
	    
	

	

	def analysis(self,data):
		self.pub = rospy.Publisher('stringozza', String, queue_size=5)
                pointsStr = String()
		

		
		#genero l'array multiplo nx3 con arrotondamento alla prima cifra decimale (step di 1 cm) delle coordinate
		a = np.array([[np.around(i.x,1),np.around(i.y,1),np.around(i.z,1)] for i in data.points])
		#rospy.loginfo(a)
		#classificazione in base alla distanza: 3 fasce da 1 metro
		c1_index = np.where(np.logical_and(a[:,2] <=1.0, np.isnan(a[:,2])==False))
		c2_index = np.where(np.logical_and(a[:,2] >1.0, a[:,2]<=2.0, np.isnan(a[:,2])==False))
		c3_index = np.where(np.logical_and(a[:,2] >2.0, a[:,2]<=3.0, np.isnan(a[:,2])==False))
		#eliminazione dei doppioni dovuto ad arrotondamento 				
		c1 = a[c1_index]
		c2 = a[c2_index]
		c3 = a[c3_index]
		#rospy.loginfo(c1)
		#ordinamento dei punti in base alla x
		#c1Sorted=c1[c1[:,0].argsort()]
		#c2Sorted=c2[c2[:,0].argsort()]
		#c3Sorted=c3[c3[:,0].argsort()]
		#creazione istogramma		
		bin_counts1, edges1 = np.histogram(c1[:,0])
		bin_counts2, edges2 = np.histogram(c2[:,0])
		bin_counts3, edges3 = np.histogram(c3[:,0])

		
		
		"""	
		pylab.hist(c1[:,0],10)
		pylab.xlabel('Number range')
		pylab.ylabel('Count')
		pylab.savefig('1.pdf')
		pylab.savefig('1.png')
		pylab.show()
		
		pylab.hist(c2[:,0],10)
		pylab.xlabel('Number range')
		pylab.ylabel('Count')
		pylab.savefig('2.pdf')
		pylab.savefig('2.png')
		pylab.show()

		pylab.hist(c3[:,0],10)
		pylab.xlabel('Number range')
		pylab.ylabel('Count')
		pylab.savefig('3.pdf')
		pylab.savefig('3.png')
		pylab.show()
		"""
		coor = []
		punto = []
		for x in np.nditer(bin_counts1):
			if(x>0):
				indice = np.where(bin_counts1==x)			
				coor.append(round(edges1[indice[0][0]],1))
		coor.append('!')
		for x in np.nditer(bin_counts2):
			if(x>0):
				indice = np.where(bin_counts2==x)
				coor.append(round(edges2[indice[0][0]],1))
		coor.append('!')
		for x in np.nditer(bin_counts3):
			if(x>0):
				indice = np.where(bin_counts3==x)
				coor.append(round(edges3[indice[0][0]],1))

		stringaBuona = str(coor)
		pointsStr.data = stringaBuona[1:len(stringaBuona)-1]
                self.pub.publish(pointsStr)
		rospy.sleep(1000.)
	


if __name__ == '__main__':
	try:
		
	    points = pointsManager()
	    rospy.init_node('listenerPC2', anonymous=True)
	    points.listener()
		

	except rospy.ROSInterruptException:
	    pass
