#!/usr/bin/env python
import rospy
import sys
import random as rd
from crowdbotsim.msg import TwistArrayStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from math import cos,sin,pi

myargv = rospy.myargv(argv=sys.argv)
script, hz, topic, size = myargv    

def Talker():
    pub = rospy.Publisher(topic, TwistArrayStamped, queue_size=1)
    rospy.init_node("TwistArrayStampedTalker", anonymous=True)
    rate = rospy.Rate(float(hz))

    msg = TwistArrayStamped()
    msg.header = Header()
    msg.twist = [Twist() for i in range(0,int(size))]

    rand_bank = [((rd.random()*2)+1)*0.5 for i in range(0,int(size))]

    for t in msg.twist:
        t.linear.x = 0
        t.linear.y = 0
        t.linear.z = 0
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = 0

    start_time = rospy.get_time()
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        for index, t in enumerate(msg.twist):
            # t.linear.z = 1.0
            t.linear.x = rand_bank[index] * sin(rospy.get_time() * float(hz) / 1000 / rand_bank[index])
            t.linear.z = rand_bank[index] * cos(rospy.get_time() * float(hz) / 1000 / rand_bank[index])
        pub.publish(msg)
        # rospy.loginfo(msg.header.seq)
        rate.sleep()
            

if __name__ == '__main__':
    Talker()
