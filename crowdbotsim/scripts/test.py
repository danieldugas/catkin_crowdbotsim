#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Float64

myargv = rospy.myargv(argv=sys.argv)
script = myargv

def left_motor_callback(data):
    print(data.data)

def test():
    # initialize node
    rospy.init_node('test')

    # set up sub
    sub = rospy.Subscriber('left_motor', Float64, left_motor_callback)

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    test()



