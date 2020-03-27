#!/usr/bin/env python
import rospy
import sys
import math
import people_msgs
from people_msgs.msg import PositionMeasurementArray

myargv = rospy.myargv(argv=sys.argv)
script = myargv

legs = {}
previous_legs = {}
previous_people = {}

def leg_tracker_callback(leg):
    for i in range(0, len(leg.people)):
        # if(leg.people[i].header.frame_id == 'sick_laser_front'):
        #     legs[leg.people[i].object_id] = 
        print( "{} - {} - x : {} y : {}".format(leg.people[i].header.frame_id, leg.people[i].object_id,leg.people[i].pos.x,leg.people[i].pos.y))
    print("\n\n------------------------\n\n")
    

def people_tracker_callback(people):
    return

def clean_old_id():
    return

def footstep_predictor_setup():
    #init node
    rospy.init_node('footstep_predictor', anonymous=True)

    #init subscribers
    leg_sub = rospy.Subscriber('leg_tracker_measurements', PositionMeasurementArray, leg_tracker_callback)
    people_sub = rospy.Subscriber('people_tracker_measurements', PositionMeasurementArray, people_tracker_callback)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        clean_old_id()
        rate.sleep()


if __name__ == '__main__':
    try:
        footstep_predictor_setup()
    except Exception as e:
        print(e)