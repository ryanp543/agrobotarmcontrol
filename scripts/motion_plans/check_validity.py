#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import ikpy
import numpy as np
import ikpy.utils.plot as plot_utils
from math import pi

# Initialize all variables needed to determine soft limits on motor angular positions
M1_MIN = -3.1415
M1_MAX = 3.1415
M2_MIN = -3.1415
M2_MAX = 0
M3_MIN = -1.68
M3_MAX = 2.5
M4_MIN = -3.1415/2
M4_MAX = 3.1415/2
M_MIN = [M1_MIN, M2_MIN, M3_MIN, M4_MIN]
M_MAX = [M1_MAX, M2_MAX, M3_MAX, M4_MAX]

ALLOWABLE_ERROR = 0.0000001

# This function checks the validity of the position (i.e. if it is a singularity, etc)
def check_validity(target_position):
    # Uploads a URDF of the robot arm and creates an ikpy kinematic chain from it
    URDF_PATH = "./src/agrobot_arm_v2/urdf/agrobot_arm_v2.urdf"
    arm_chain = ikpy.chain.Chain.from_urdf_file(URDF_PATH)
    actual_position = arm_chain.forward_kinematics(arm_chain.inverse_kinematics(target_position))[:3, 3]

    # If the difference between actual position and target position is high, usually indicates singularity.
    if np.any(abs(actual_position-target_position) > ALLOWABLE_ERROR):
        rospy.logerr("Position %s is NOT valid.", target_position)
    else:
        rospy.loginfo("Position %s is valid.", target_position)

# This function checks to make sure the angle is within the min/max possible motor angles, defined by global variables
def check_validity_angles(joint_goal):
    if all(joint_goal.data[k] >= M_MIN[k] for k in range(len(joint_goal.data))) and \
            all(joint_goal.data[g] <= M_MAX[g] for g in range(len(joint_goal.data))):
        rospy.loginfo("Joint goal %s is valid.", joint_goal.data)
    else:
        rospy.logerr("Joint goal %s is NOT valid.", joint_goal.data)

# target_position = [0.15, 0.2, 0.3]
# check_validity(target_position)