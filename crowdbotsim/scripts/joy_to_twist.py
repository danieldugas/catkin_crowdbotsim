#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

myargv = rospy.myargv(argv=sys.argv)
script, factor_lin, factor_rot, hz, topic = myargv    

factor_lin = float(factor_lin)
factor_rot = float(factor_rot)

# initialize node
rospy.init_node('joy_to_twist', anonymous = True)


#### Setup joy_to_twist Publisher 
joy_to_twist_pub = rospy.Publisher(topic, Twist, queue_size = 1)

msg = Twist()

def on_joy(joy):
    msg.linear.x = -joy.axes[0]*factor_lin
    msg.angular.z = joy.axes[1]*180/3.141592*factor_rot
    joy_to_twist_pub.publish(msg)

def joy_to_twist():
	
        #### Setup joy subscriber
        joy_sub = rospy.Subscriber('joy', Joy, on_joy)

        rate = rospy.Rate(float(hz))

	while not rospy.is_shutdown():
            #### Publish msg
        #     rospy.loginfo(msg)
            rate.sleep()


if __name__ == '__main__':
        joy_to_twist()
