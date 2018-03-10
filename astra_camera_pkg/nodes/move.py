#!/usr/bin/env python


import rospy
import time
import redis
from geometry_msgs.msg import Twist
import json

def move():
    R = redis.Redis()
    pubsub = R.pubsub()
    pubsub.subscribe('KOBUKI:POSITION')
    
    pubROS = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)
    rospy.init_node('position_kobuki_proxy', anonymous=True)
    rate = rospy.Rate(10)
    message=pubsub.get_message()
    position_msg = Twist()
    while not rospy.is_shutdown():
	message = pubsub.get_message()
        if message:
	    msg = json.loads(message['data'])
	    position_msg.linear.x = msg['linear_x'] 
	    position_msg.linear.y = msg['linear_y'] 
	    position_msg.linear.z = msg['linear_z']
	    position_msg.angular.x = msg['angular_x']
	    position_msg.angular.y = msg['angular_y']
	    position_msg.angular.z = msg['angular_z']

	    pubROS.publish(position_msg)
	rate.sleep()


if __name__ == '__main__':
    try:

        move()
    except rospy.ROSInterruptException:
        pass
