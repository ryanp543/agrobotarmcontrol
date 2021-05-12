#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import ikpy
import numpy as np
import ikpy.utils.plot as plot_utils
from math import pi
from std_msgs.msg import String
from agrobot_arm_v2.msg import FloatList
from check_validity import check_validity, check_validity_angles

# NOTE: This line assumes you are rosrun-ing from your workspace directory, i.e. catkin_ws
URDF_PATH = "./src/agrobot_arm_v2/urdf/agrobot_arm_v2.urdf"

""" CHANGE THIS PATH TO CSV NAME """
CSV_PATH = "./src/agrobot_arm_v2/scripts/motion_plans/motion_plan_forpic.csv"
""" CHANGE THIS PATH TO CSV NAME """

# Intiailizes the publisher and 
pub = rospy.Publisher('motion_plan', FloatList, queue_size=1)
pub_tool = rospy.Publisher("tool_changer", String, queue_size=1)
rospy.init_node('run_motion_plan_csv', anonymous=True)
arm_chain = ikpy.chain.Chain.from_urdf_file(URDF_PATH)

joint_goal = FloatList()
joint_goal.data = [None] * 4

# PLANNING EXAMPLE POSE MOVEMENT

with open(CSV_PATH) as csvfile:

    for k, line in enumerate(csvfile):
        joint_goal.data = [float(i) for i in line.split()]
        check_validity_angles(joint_goal)
        # print(joint_goal.data)
        pub.publish(joint_goal)


# target_position = [0.15, 0.2, 0.3]
# [_, joint_goal.data[0], joint_goal.data[1], joint_goal.data[2], _] = arm_chain.inverse_kinematics(target_position)
# joint_goal.data[3] = 1.0
# check_validity(target_position)
# pub.publish(joint_goal)
#
# target_position = [0.15, -0.2, 0.4]
# joint_goal.data[3] = 0.0
# [_, joint_goal.data[0], joint_goal.data[1], joint_goal.data[2], _] = arm_chain.inverse_kinematics(target_position)
# pub.publish(joint_goal)