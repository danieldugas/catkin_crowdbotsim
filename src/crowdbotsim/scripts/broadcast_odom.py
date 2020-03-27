#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import tf.broadcaster

myargv = rospy.myargv(argv=sys.argv)
script = myargv

rospy.init_node('odom_sub')

br = tf.TransformBroadcaster()

# sub to the clock
ros_clock = Clock()

# first, we'll publish the transform over tf
odom_trans = TransformStamped()
odom_trans.header.stamp = ros_clock.clock
odom_trans.header.frame_id = "odom"
odom_trans.child_frame_id = "base_link"
odom_trans.transform.translation.x = 0
odom_trans.transform.translation.y = 0
odom_trans.transform.translation.z = 0
odom_trans.transform.rotation.x = 0
odom_trans.transform.rotation.y = 0
odom_trans.transform.rotation.z = 0
odom_trans.transform.rotation.w = 0

# send the transform
br.sendTransformMessage(odom_trans)

def on_clock(clock):
    global ros_clock
    ros_clock.clock = clock.clock

def on_odom(odom):
    global ros_clock, odom_trans, br
    odom_trans.header.stamp = ros_clock.clock
    odom_trans.transform.translation.x = odom.pose.pose.position.x
    odom_trans.transform.translation.y = odom.pose.pose.position.y
    odom_trans.transform.translation.z = odom.pose.pose.position.z
    odom_trans.transform.rotation = odom.pose.pose.orientation
    # br.sendTransform((msg.x, msg.y, 0), tf.transformations.quaternion_from_euler(0, 0, msg.theta),
    #                rospy.Time.now(),
    #                turtlename,
    #                "world")
    br.sendTransformMessage(odom_trans)


def setup_sub():
    #### Setup odom subscriber
    clock_sub = rospy.Subscriber('odom', Odometry, on_odom)
    odom_sub = rospy.Subscriber('clock', Clock, on_clock)

    rospy.spin()


if __name__ == '__main__':
    setup_sub()