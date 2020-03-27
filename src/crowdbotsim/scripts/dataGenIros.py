#!/usr/bin/env python
import rospy
import sys
import numpy as np
from time import sleep, time

from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from crowdbotsim.msg import TwistArrayStamped

myargv = rospy.myargv(argv=sys.argv)
script, crowd_size, time_step, sleep_time, total_time, scenario_number, starting_scenario = myargv
time_step = float(time_step)
crowd_size = int(crowd_size)
scenario_number = int(scenario_number)
sleep_time = float(sleep_time)
total_time = float(total_time)
starting_scenario = (int)(starting_scenario)

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

global scenario, scan_seq, scan, scanMask, crowdX, crowdY, crowdT, robotX, robotT, robotY
global dataHeader, dataLidar, dataLidarMask, dataCrowdX, dataCrowdY, dataCrowdT, dataOdom, dataFull, dataRobotOdom

scenario = np.zeros(1)
scan_seq = np.zeros(1)
simTime = np.zeros(1)
scan = np.zeros(450)
scanMask = np.zeros(450)
crowdX = np.zeros(crowd_size)
crowdY = np.zeros(crowd_size)
crowdT = np.zeros(crowd_size)
robotX = np.zeros(1)
robotY = np.zeros(1)
robotT = np.zeros(1)


dataHeader = np.concatenate((scenario, scan_seq, simTime))
print(dataHeader)

# Data format - lidar:
# - Each scenario is a .csv file, with each line corresponds to a scan, separated by comma.
# - The first number of each line should be a scan sequence number (unique within scenario)
# - The second number should be simulation time
# - The rest should be lidar reading, arranged from left to right

dataLidar = np.concatenate((dataHeader, scan))

# Data format - lidar segmentation mask:
# - Use .csv.mask for file ending, but in essence is a csv file
# - Mostly same as data format for lidar
# - Instead of having lidar reading for each entry, have corresponding pedestrian id (unique within scenario)

dataLidarMask = np.concatenate((dataHeader, scanMask))

# Data format - pedestrian trajectories:
# - Three csv files with ending .csv.traj.x, .csv.traj.y, and .csv.traj.phi
# - Use the first line to store ids of all pedestrian(unique within scenario)
# - Then for the rest of the lines:
# - First two numbers of each line are scan sequence and time (same as for lidar)
# - The rest of the line should be x/y/phi (depending on the file) of each pedestrian at that time

dataCrowdX = np.concatenate((dataHeader, crowdX))
dataCrowdY = np.concatenate((dataHeader, crowdY))
dataCrowdT = np.concatenate((dataHeader, crowdT))

# Data format - robot pose:
# - csv.odom file
# - First two numbers of each line are scan sequence and time (same as for lidar)
# - The rest of the lines should be x/y/phi of the robot

dataRobotOdom = np.concatenate((dataHeader, robotX, robotY, robotT))

# Data format - Full
# - A csv file with ending csv.full
# - For each line:
# - scan id, time, scan, scan asks, odom, crowdX, crowdY, crowdT

dataFull = np.concatenate((scenario, simTime, scan, scanMask, crowdX, robotX, crowdY, robotY, crowdT, robotT))

global scan_updated, crowd_updated, robot_updated

scan_updated = False
crowd_updated = False
robot_updated = False

def on_scan(rcv_scan):
    global scan, scanMask, scan_updated
    scan = np.around(rcv_scan.ranges, decimals=3)
    # ad hoc trick, masks on intensities
    scanMask = np.around(rcv_scan.intensities)
    scan_updated = True

def on_crowd(t_arr):
    global crowdX, crowdY, crowdT, crowd_updated
    for i,t in enumerate(t_arr.twist):
        crowdX[i] = t.linear.x
        crowdY[i] = t.linear.y
        crowdT[i] = np.deg2rad(t.angular.z)
    crowd_updated = True

def on_robot_pose(t):
    global robotX, robotY, robotT, robot_updated
    robotX = np.array([t.linear.x])
    robotY = np.array([t.linear.y])
    robotT = np.array([np.deg2rad(t.angular.z)])
    robot_updated = True

def publish_clock(current_time, clock_pub, msg):
    ### Publish clock
    secs = int(current_time)
    nsecs = 1e9 * (current_time - secs)
    msg.clock =  rospy.Time(secs,nsecs)
    clock_pub.publish(msg)


def write_data(path,name,scenNb,ext,data):
        f = open(path+"/"+name+"_{}.".format(scenNb)+ext,"a+")
        f.write(",".join([str(x) for x in data.tolist()])+'\n')
        f.close()

def data_gen():
    global scan_updated, crowd_updated
    global scenario, scan_seq, scan, scanMask, crowdX, crowdY, crowdT
    global dataHeader, dataLidar, dataLidarMask, dataCrowdX, dataCrowdY, dataCrowdT, dataOdom, dataFull, dataRobotOdom

    # initialize node
    rospy.init_node('data_gen', anonymous = False)

    ### Set up clock publisher
    clock_pub = rospy.Publisher('clock', Clock, queue_size = 1)

    msg = Clock()
    msg.clock = rospy.Time()

    current_time = 0
    scenario = np.array([0+starting_scenario])
    scan_seq = np.array([0])

    write_data("./test","RVO",scenario[0],"csv.traj.x",np.arange(crowd_size+1)) # +1 : add the robot
    write_data("./test","RVO",scenario[0],"csv.traj.y",np.arange(crowd_size+1))
    write_data("./test","RVO",scenario[0],"csv.traj.t",np.arange(crowd_size+1))

    ### Set up subscribers
    scan_sub = rospy.Subscriber('/scan', LaserScan, on_scan, queue_size = 1)
    crowd_sub = rospy.Subscriber('/crowd', TwistArrayStamped, on_crowd, queue_size = 1)
    robot_sub = rospy.Subscriber('/robot_pose', Twist, on_robot_pose, queue_size = 1)

    start = time()
    
    last_timestamp = current_time
    update_freq = 0

    while not rospy.is_shutdown() and scenario[0] < scenario_number:
        while not rospy.is_shutdown() and current_time < total_time:
            publish_clock(current_time, clock_pub, msg)
            sleep(sleep_time)

            if scan_updated and crowd_updated and robot_updated :

                simTime = np.array([current_time])

                dataHeader = np.concatenate((scenario, scan_seq, simTime))
                dataLidar = np.concatenate((dataHeader, scan))
                dataLidarMask = np.concatenate((dataHeader, scanMask))
                dataCrowdX = np.concatenate((dataHeader, crowdX))
                dataCrowdY = np.concatenate((dataHeader, crowdY))
                dataCrowdT = np.concatenate((dataHeader, crowdT))
                dataRobotOdom = np.concatenate((dataHeader, robotX, robotY, robotT))
                dataFull = np.concatenate((scenario, simTime, scan, scanMask, crowdX, robotX, crowdY, robotY, crowdT, robotT))

                write_data("./test","RVO",scenario[0],"csv",dataLidar)
                write_data("./test","RVO",scenario[0],"csv.mask",dataLidarMask)
                write_data("./test","RVO",scenario[0],"csv.traj.x",dataCrowdX)
                write_data("./test","RVO",scenario[0],"csv.traj.y",dataCrowdY)
                write_data("./test","RVO",scenario[0],"csv.traj.t",dataCrowdT)
                write_data("./test","RVO",scenario[0],"csv.odom",dataRobotOdom)
                write_data("./test","RVO",scenario[0],"csv.full",dataFull)

                scan_seq[0] = scan_seq[0] + 1
                scan_updated = False
                crowd_updated = False
                if not current_time == last_timestamp:
                    update_freq = 1.0/(current_time - last_timestamp)
                last_timestamp = current_time

            current_time += time_step

            print(scenario[0], "{0:.2f}".format(current_time),"{0:.2f}".format(time() - start), "scan : {} Hz".format(np.around(update_freq)))
        scenario[0] = scenario[0] + 1
        current_time = 0
        scan_seq = np.array([0])
        print("New scenario")
        sleep(2)

if __name__ == '__main__':
    data_gen()
