#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64MultiArray
from nao_pkg.msg import Float64Array, HeadPitch


import sys
import time

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

import json
import argparse
import motion
import almath
import math
import time 

from functools import partial
import numpy as np
from optparse import OptionParser

NAO_IP = "nao.local"


# Global variable to store the HumanGreeter module instance
memory = None


class naoTalker():
    """ A simple module able to react
    to facedetection events

    """
    def __init__(self):

    	self.connectNaoQi()

        self.motion   = ALProxy("ALMotion")        
        self.posture  = ALProxy("ALRobotPosture")        
        self.tts      = ALProxy("ALTextToSpeech")
        global memory
        memory = ALProxy("ALMemory")
        
        #self.tts.say('ball found')

        pub = rospy.Publisher('nao_chatter', HeadPitch, queue_size=10)
        rospy.init_node('nao_talker', anonymous=True)
        rate = rospy.Rate(10)
        names="HeadPitch"
        useSensors=False
        commandAngles=self.motion.getAngles(names,useSensors)
        rospy.loginfo(commandAngles[0])
        pub.publish(commandAngles[0])
        #self.createJson(commandAngles)
        
        """
        #r = rospy.Rate(1) # 10hz
        while not rospy.is_shutdown():        	
        	commandAngles=self.motion.getAngles(names,useSensors)
        	print commandAngles[0]
        	rospy.loginfo(commandAngles[0])
        	pub.publish(commandAngles[0])
	        rate.sleep()
        """
    """
    def createJson(self, data):
        output_data={}
        jointName = self.motion.getBodyNames("HeadPitch")
        
        for i in range(len(data)):
            output_data[jointName[i]] = data[i]
        
        with open('/home/giovanna/catkin_ws/src/nao_pkg/JSON/jointStatus.txt', 'w') as outfile:
            json.dump(output_data , outfile)
    """



    def connectNaoQi(self):
    	try:
	    	parser = OptionParser()
	    	parser.add_option("--pip",
		        help="Parent broker port. The IP address or your robot",
		        dest="pip")
	    	parser.add_option("--pport",
		        help="Parent broker port. The port NAOqi is listening to",
		        dest="pport",
		        type="int")
	    	parser.set_defaults(
		        pip="127.0.0.1",
		        pport=9559)
	    	(opts, args_) = parser.parse_args()
	    	self.pip   = opts.pip
	    	self.pport = opts.pport

	    	myBroker = ALBroker("myBroker","0.0.0.0",0,self.pip, self.pport)
    	
        	
    	except KeyboardInterrupt:
	    	print "Interrupted by user, shutting down"
	        myBroker.shutdown()
	        sys.exit(0)



       


if __name__ == "__main__":
    try:
        nao = naoTalker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
