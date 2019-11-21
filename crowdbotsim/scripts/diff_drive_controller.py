#!/usr/bin/env python
import rospy
import sys
import math
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

myargv = rospy.myargv(argv=sys.argv)
script, robot_ns = myargv

# state variables
Vc = 0  # linear velocity command
Wc = 0  # ang veloctiy command
V = 0   # lin vel measure
W = 0   # ang vel measure
Wr = 0  # right motor vel input
Wl = 0  # left motor vel input

# controller variables
sum_eps = np.array([0.0,0.0]) #[V,W]
last_eps = np.array([0.0,0.0]) #[V,W]
reset_sum_eps = False
last_joy_timestamp = 0


def on_joy(joy):
    global reset_sum_eps, last_joy_timestamp
    reset_sum_eps = False
    last_joy_timestamp = rospy.get_time()

def odom_callback(odom):
    global V, W
    v_ = [odom.twist.twist.linear.x, odom.twist.twist.linear.y]
    th = 2 * math.acos(odom.pose.pose.orientation.w)
    v_th_ = [math.cos(th), math.sin(th)]
    V = np.dot(v_,v_th_)
    W = math.radians(odom.twist.twist.angular.z)

def cmd_vel_callback(cmd_vel):
    global Vc, Wc
    Vc = cmd_vel.linear.x
    Wc = math.radians(cmd_vel.angular.z)

def diffdrive_IK(in_V, in_W, radius, spacing):
    global Wr, Wl
    Wr = math.degrees(-(in_V + in_W/(2*spacing))/radius)
    Wl = math.degrees(-(in_V - in_W/(2*spacing))/radius)

def reshape_input(toshape, scale, max, min):
    global sum_eps, last_eps
    shape = toshape / scale
    for i, val in enumerate(shape):
        if val > max:
            shape[i] = max 
        if val < min:
            shape[i] = min 
    return shape


def PID_controller(eps, P, I, D, Te):
    global sum_eps, last_eps
    eps[1] = eps[1]/4
    sum_eps += eps

    #reset the sum if no input
    if rospy.get_time() > (last_joy_timestamp + 1):
        sum_eps = 0

    Ki = 0
    if not I == 0:
        Ki = Te/I
    U = P *( eps + Ki * sum_eps + (D/Te) * (eps - last_eps))
    last_eps = eps
    return U

def diff_drive_controller():
    global Vc, Wc, V, W, Wr, Wl
    
    # initialize node
    rospy.init_node('diff_drive_controller')

    ### Set up subscribers
    odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)
    cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    joy_sub = rospy.Subscriber('joy', Joy, on_joy)

    ### Set up publishers
    left_wheel_pub = rospy.Publisher('left_motor', Float64, queue_size = 1)
    right_wheel_pub = rospy.Publisher('right_motor', Float64, queue_size = 1)


    robot_param = rospy.get_param("/" + robot_ns)
    diff_drive_pid_param = robot_param['diff_drive_pid']
    radius = diff_drive_pid_param['radius']
    spacing = diff_drive_pid_param['spacing']
    P = diff_drive_pid_param['P']
    I = diff_drive_pid_param['I']
    D = diff_drive_pid_param['D']
    max_vel = diff_drive_pid_param['max_velocity']
    loop_hz = robot_param['loop_hz']

    rate = rospy.Rate(loop_hz)

    while not rospy.is_shutdown():
        ### Publish
        eps = np.array([Vc - V , Wc - W])
        in_V, in_W = PID_controller(eps, P, I, D, loop_hz)
        diffdrive_IK(in_V,in_W,radius, spacing)
        Wr, Wl = reshape_input(np.array([Wr,Wl]), max_vel, 1, -1)
        right_wheel_pub.publish(Float64(Wr))
        left_wheel_pub.publish(Float64(Wl))
        # print(Vc, Wc, V, W,  Wr, Wl)

        rate.sleep()

if __name__ == '__main__':
    diff_drive_controller()



