#!/usr/bin/env python

import rospy
import random

import xml.dom.minidom
from math import pi
from threading import Thread
from functools import partial
import numpy as np
from optparse import OptionParser
import sys
import signal
import math
import json
import argparse
import almath
import motion
import cv2
import time

from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointCloud
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()


def get_param(name, value=None):
	private = "~%s" % name
	if rospy.has_param(private):
		return rospy.get_param(private)
	elif rospy.has_param(name):
		return rospy.get_param(name)
	else:
		return value




class cameraMenager():
	
	def __init__(self):
		self.success = False

		self.cloud_ = None # ground truth for cartesian computation
		self.rdis_ = None
		self.uvec_ = None
		self.extrinsics_ = None


	def save(self):
		time.sleep(2.0)
		rospy.loginfo("nodo inizi")
		rospy.init_node('camera_menager', anonymous=True)
 
        	rospy.Subscriber("camera/depth/points", PointCloud2, self.image_callback)
		rospy.spin()

		

        def createJson(self, data,nome):
	
		with open('/home/giovanna/catkin_ws/src/astra_camera_pkg/JSON/'+nome+'.txt', 'w') as outfile:
		    json.dump(data , outfile)
	    
	


	def image_callback(self, data):
        	rospy.sleep(15.)
		rospy.loginfo("Received an image!")
		count = 0
		try:
            	    
		    while( count<1):    
			
		        rospy.loginfo(data.height)
		        rospy.loginfo(data.width)
		        #rospy.loginfo(data.data)
		        count = count +1
		        if ( count == 1):
                            self.createJson(str(data), "pointCloudprova")
                            self.pub = rospy.Publisher('pointCloud', PointCloud2, queue_size=5)
                            points = PointCloud2()
                            points = data
                            self.pub.publish(points)
                            rospy.sleep(1.) #ogni 10 secondi
                    	    
    			#data.encoding = "rgb16"
    			# Convert your ROS Image message to OpenCV2
    			#depth_image = bridge.imgmsg_to_cv2(data, "32FC1")
            	    """
            	    self.createJson(data, "pointCloudProva") 
		    		    
		    dati = data.data.split(',')
		    rospy.loginfo(len(dati))
		    rospy.loginfo(type(data.data))
		    rospy.loginfo(len(data.data))    
		    i=0
		    points = Quaternion()
		    quat=[]
		    j=0
		    while(i<len(dati)):
			quat.x = int.Parse(dati[i]);
                        quat.y = int.Parse(dati[i+1]);
                        quat.z = int.Parse(dati[i+3]); 
			quat.w = int.Parse(dati[i+4]); 
			i=i+4
			points[j] = quat
		        j=j+1
	    	    self.pub = rospy.Publisher('quaternion', QuaternionArray, queue_size=5)	
		    self.pub.publish(points)
		    """
		    #self.pub = rospy.Publisher('pointCloud', PointCloud2, queue_size=5)
		    #points = PointCloud2()
		    #points = data
		    #self.pub.publish(points)
		except CvBridgeError, e:
			print(e)

		# Convert the depth image to a Numpy array since most cv2 functions
		# require Numpy arrays.
		#depth_array = np.array(depth_image, dtype=np.float32)

		# Normalize the depth image to fall between 0 (black) and 1 (white)
		#cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

		# Process the depth image
		#depth_display_image = self.process_depth_image(depth_array)

		# Display the result
		#cv2.imshow("Depth Image", depth_display_image)	    
		# Save your OpenCV2 image as a jpeg 
		#cv2.imwrite('/home/giovanna/catkin_ws/src/astra_camera_pkg/images/camera_image_depth.jpeg', cv2_img)

	def process_depth_image(self, frame):
        	return frame



if __name__ == '__main__':
	try:
		
	    camera = cameraMenager()
	    camera.save()
		

	except rospy.ROSInterruptException:
	    pass
