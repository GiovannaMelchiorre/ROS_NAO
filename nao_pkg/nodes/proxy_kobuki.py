#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import redis
import simplejson as json

class kobukiMovement():
    	def __init__(self):
	    self.R = redis.Redis(host='192.168.0.101')
	    rospy.init_node('proxy_kobuki', anonymous=True)

		

	def velocity_kobuki(self,data):
    
    	    message = json.dumps({'linear_x': data.linear.x , 
			'linear_y': data.linear.y, 
      			'linear_z': data.linear.z ,
			'angular_x': data.angular.x, 
			'angular_y': data.angular.y, 
			'angular_z': data.angular.z})
	
    
    	    self.R.publish('KOBUKI:POSITION',message)


	def proxy(self):
	    self.subROS = rospy.Subscriber('cmd_vel', Twist, self.velocity_kobuki)
	    rospy.spin()

if __name__ == '__main__':
    try:
	kobuki = kobukiMovement()
	kobuki.proxy()

    except rospy.ROSInterruptException:
        pass
