#!/usr/bin/env python3
import rospy
import sys
from crowdbotsimcontrol import helpers, socket_handler
from time import sleep, time
from signal import signal, SIGINT
from sys import exit

#messages
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

myargv = rospy.myargv(argv=sys.argv)
script, HOST, PORT, delta_time, sleep_time, scenarios_size, min_x, max_time = myargv

PORT = int(PORT)
sleep_time = float(sleep_time)
delta_time = float(delta_time)
scenarios_size = int(scenarios_size)
min_x = float(min_x)
max_time = float(max_time)

def update_ros_messages(dico,publishers):
    if 'clock' in publishers.keys():
        publishers['clock'] = (rospy.Time(float(dico['clock'])), publishers['clock'])

def twist_msg_to_vel_cmd_tcp_msg(twist_msg, pub):
    pub['vel_cmd'] = (twist_msg.linear.x, twist_msg.angular.z)

def run_challenge(HOST, PORT, delta_time, sleep_time, scenarios_size, min_x, max_time):
    s = socket_handler.init_socket(HOST, PORT)

    def handler(signal_received, frame):
        # Handle any cleanup here
        print('SIGINT or CTRL-C detected. Exiting gracefully')
        socket_handler.stop(s)
        exit(0)
    signal(SIGINT, handler)

    #### initialize messages 
    publishers = {}
    # initialize node
    rospy.init_node('simulation_controller')

    ##### Subscribers 
    def on_vel_cmd(twist_msg):
        received_first_vel_cmd = True
        msg_vel_cmd.linear.x = twist_msg.linear.x
        msg_vel_cmd.angular.z = twist_msg.angular.z

    msg_vel_cmd = Twist()
    vel_cmd_sub = rospy.Subscriber('vel_cmd', Twist, on_vel_cmd)

    ##### inputs (from sim)
    msg_clock = Clock()
    clock_pub = rospy.Publisher('clock', Clock, queue_size=1)
    publishers['clock'] = (msg_clock, clock_pub)

    for i in range(scenarios_size):

        # building initial dict to send to simulator
        pub = {'clock':0, 'vel_cmd': (0,0), 'sim_control':'i'}


        received_first_vel_cmd = False

        while not rospy.is_shutdown():

            while not received_first_vel_cmd:
                print("waiting for first vel_cmd message")

            # getting current time
            time_in = time()

            #### Handling simulation step
            # making the raw string to send to simulator using dict 
            to_send = helpers.publish_all(pub)
            # sending and receiving raw data
            raw = socket_handler.send_and_receive(s,to_send)
            # getting dict from raw data
            dico = helpers.raw_data_to_dict(raw)

            # update ros messages using dico entries
            update_ros_messages(dico, publishers)

            ### Publish msgs
            clock_pub.publish(msg_clock)

            # using updated vel_cmd message for new tcp packet
            twist_msg_to_vel_cmd_tcp_msg(msg_vel_cmd, pub)

            # checking ending conditions
            if helpers.check_ending_conditions(max_time, min_x, dico):
                break

            # doing a step
            pub = helpers.do_step(delta_time, pub)

            time_out = time()
            if sleep_time > 0: 
                sleep(sleep_time)
            elif time_out < time_in + delta_time : # real time simulation
                sleep(time_in + delta_time - time_out)

        socket_handler.send_and_receive(s, helpers.publish_all(helpers.next()))

        #wait for sim to load new scenario
        sleep(1)

    socket_handler.stop(s)

if __name__ == '__main__':
    run_challenge(HOST, PORT, delta_time, sleep_time, scenarios_size, min_x, max_time)
