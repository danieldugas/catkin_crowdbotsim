#!/usr/bin/env python
import rospy
import sys
import math
from time import sleep
from rosgraph_msgs.msg import Clock

myargv = rospy.myargv(argv=sys.argv)
script, sleep_time, delta_time = myargv

sleep_time = float(sleep_time)
delta_time = float(delta_time)

def publish_clock():
    # initialize node
    rospy.init_node('clock_controller', anonymous = False)

    ### Set up clock publisher
    clock_pub = rospy.Publisher('clock', Clock, queue_size = 1)

    msg = Clock()
    msg.clock = rospy.Time()

    current_time = 0

    while not rospy.is_shutdown():
        ### Publish clock
        secs = int(current_time)
        nsecs = 1e9 * (current_time - secs)
        msg.clock =  rospy.Time(secs,nsecs)
        clock_pub.publish(msg)
        current_time += delta_time
        sleep(sleep_time)

if __name__ == '__main__':
    publish_clock()



