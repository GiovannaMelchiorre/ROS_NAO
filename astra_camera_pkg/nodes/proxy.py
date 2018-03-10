#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import redis


class cameraDepthListen():

    def __init__(self):
	self.R = redis.Redis(host='192.168.0.101')
        self.pubsub = self.R.pubsub()	
	self.pubROS = rospy.Publisher('string_depth', String, queue_size=10)
        rospy.init_node('string_depth_proxy', anonymous=True)
		
    def talker(self):
    
        self.pubsub.subscribe('ASTRACAM:DEPTH')    
    	rate = rospy.Rate(10) # 10hz
    	message = self.pubsub.get_message()
    	str_depth = String()
    	while not rospy.is_shutdown():
        	message = self.pubsub.get_message()
		if message:
	    		#rospy.loginfo(message)
            
            		str_depth.data = message['data']
	
            		rospy.loginfo(str_depth.data)
            		self.pubROS.publish(str_depth)
        	rate.sleep()

if __name__ == '__main__':
    try:
        camera = cameraDepthListen()
	camera.talker()

    except rospy.ROSInterruptException:
        pass
