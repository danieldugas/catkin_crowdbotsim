#!/usr/bin/env python
import rospy
import sys
import numpy as np

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

myargv = rospy.myargv(argv=sys.argv)
script, scenarios_count, last_scenario_duration, directory = myargv

global front_scan, rear_scan, twist, timestamps, scenario, data

front_scan = np.zeros(818)
rear_scan = np.zeros(818)
twist = np.zeros(3) #x y thetaZ
timestamps = np.zeros(1)
scenario = np.zeros(1)

data = np.concatenate((scenario, timestamps, twist, front_scan, rear_scan),axis=0)

def on_front_scan(scan): 
    global front_scan, timestamps
    front_scan = np.array(scan.ranges)
    timestamps = np.array([scan.header.stamp.secs + float(scan.header.stamp.nsecs) / 1000000000])

def on_rear_scan(scan): 
    global rear_scan
    rear_scan = np.array(scan.ranges)

def on_twist(t):
    global twist
    twist = np.array([t.linear.x, t.linear.y, t.angular.z])

def on_scenario(val):
    global scenario
    scenario = np.array([val.data])

def data_gen():
    global front_scan, rear_scan, twist, timestamps, scenario, data
    # initialize node
    rospy.init_node('data_gen', anonymous = False)

    ### Set up subscribers
    front_scan_sub = rospy.Subscriber('/sick_laser_front/scan', LaserScan, on_front_scan, queue_size = 1)
    rear_scan_sub = rospy.Subscriber('/sick_laser_rear/scan', LaserScan, on_rear_scan, queue_size = 1)
    scenario_sub = rospy.Subscriber('/scenario', Float64, on_scenario, queue_size = 1)
    robot_twist_sub = rospy.Subscriber('/robot/twist', Twist, on_twist, queue_size = 1)

    rate = rospy.Rate(2)
    _now = rospy.Time.now()
    
    current_scenario = 0
    f = open(directory+"/scenario{}".format(current_scenario),"a+")

    while not rospy.is_shutdown() and (scenario[0] < float(scenarios_count) or timestamps[0] < float(last_scenario_duration)):
        data = np.concatenate((scenario, timestamps, twist, front_scan, rear_scan),axis=0)
        #write data
        if(current_scenario < scenario[0]):
            f.close()
            current_scenario = scenario[0]
            f = open(directory+"/scenario{}.csv".format(current_scenario+366),"a+")
        f.write(",".join([str(x) for x in data.tolist()])+'\n')
        rate.sleep()

    f.close()

if __name__ == '__main__':
    data_gen()
