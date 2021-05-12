#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
import rospy
import csv

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from agrobot_arm_v2.msg import FloatList

from struct import *

FILEPATH = './rviz_data/new_rviz_data_RENAME.csv'

def callback(motor_data):
    # This function receives motor data and assigns it to the appropriate list in the structure
    # publish_data.data = [ID_R, POS_R, VEL_R, I_R]
    # rospy.loginfo(motor_data.data)
    id = int(motor_data.data[0])
    motor_state.position[id-1] = motor_data.data[1]


def listener():
    # Initialize subscriber of ROS nodes
    global motor_state
    rospy.Subscriber("motor_data", FloatList, callback)

    # Plays data on Rviz and then writes the data to a .csv file for later replayability at the end
    get_rviz_data = rospy.get_param('~get_rviz_data')
    if get_rviz_data:
        with open(FILEPATH, 'w') as csvfile:
            fieldnames = ['Time', 'Motor1', 'Motor2', 'Motor3', 'Motor4']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            t_start = rospy.Time.now()
            while not rospy.is_shutdown():
                motor_state.header.stamp = rospy.Time.now()
                writer.writerow({'Time': (rospy.Time.now()-t_start).to_sec(), 'Motor1': motor_state.position[0],
                                 'Motor2': motor_state.position[1], 'Motor3': motor_state.position[2],
                                 'Motor4': motor_state.position[3]})
                pub.publish(motor_state)
                rate.sleep()
    else:
        while not rospy.is_shutdown():
            motor_state.header.stamp = rospy.Time.now()
            pub.publish(motor_state)
            rate.sleep()


if __name__ == "__main__":
    # Initialize ROS publisher and node
    pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
    rospy.init_node('rviz_start', anonymous=True)
    rate = rospy.Rate(50)

    # Creates stucture of list containing position of joint states
    motor_state = JointState()
    motor_state.header = Header()
    motor_state.name = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4']
    motor_state.position = [0, 0, 0, 1.571]

    listener()
