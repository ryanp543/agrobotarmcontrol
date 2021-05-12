#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from agrobot_arm_v2.msg import FloatList

from struct import *

HOST = "localhost"
PORT = 4223

# Global variables declaring resolution of movement in radians
POS_RESOLUTION = 0.01
POS_MIN_STEP = 0.01

# Stepper motor properties for simulation
STEPPER_MAX_VELOCITY = 65535            # step/s
STEP_RESOLUTION = 1.8 * (np.pi / 180)   # rad/step
STEP_GEAR_RATIO = 25.2
STEP_MICROSTEP = 256
STEP_FROM_POS = STEP_GEAR_RATIO * (STEP_MICROSTEP) / STEP_RESOLUTION

# Motor start points for the robot arm at start up
MOTOR3_START = -1.68
MOTOR4_START = 1.571  # STEPPER MOTOR START POINT (TO BE CHANGED)

# PD gain settings for the Mini Cheetah motors (doesn't matter here)
K_P = 10.00
K_D = 0.70

# Soft limits on range of motor movement
M1_MIN = -3.1415
M1_MAX = 3.1415
M2_MIN = -3.1415
M2_MAX = 0
M3_MIN = MOTOR3_START
M3_MAX = 2.5                            # less than P_MAX/PULLEY_RATIO
M4_MIN = -3.1415/2                        # STEPPER MOTOR LIMIT (TO BE CHANGED)
M4_MAX = 3.1415/2                         # STEPPER MOTOR LIMIT (TO BE CHANGED)

ID_R = POS_R = VEL_R = I_R = 0

# The speed at which commands can be sent to the simulation
SLEEP_INTERVAL = 0.002                  # in seconds


# Basic class and functions, based on MotorModuleController class python file.
class MotorModuleController_Simulation():
    def __init__(self):
        try:
            rospy.loginfo('Connected to motor module controller')
        except:
            rospy.loginfo('Failed to connect to motor module controller')
            pass

    def send_command(self, id, p_des, v_des, kp_des, kd_des, i_ff):
        # Simply publishes to topic that is read by the Gazebo simulation
        # print(p_des)
        joint_pub[id-1].publish(p_des)


def ramp_to_pos(motor_commands):
    """
    Takes array of motor ID, current positions, and desired positions and returns same array except with a different
    second column (altered current positions)
    """
    # Calculates the time it will take for the stepper motor to move (so we know wait time before simulation makes the
    # next motion command)
    step_time = (abs(motor_commands[3,2]-motor_commands[3,1])/(STEP_MAX_VELOCITY/STEP_FROM_POS))

    # Send command to stepper motor wrist
    mmc.send_command(4, motor_commands[3, 2], 0.00, K_P, K_D, 0.00)
    motor_commands[3, 1] = motor_commands[3, 2]

    current_time = time.clock()

    # Send incremental commands to ramp the motor position on the arm
    while np.any(np.absolute(motor_commands[:, 2] - motor_commands[:, 1]) >= POS_RESOLUTION * np.ones(4)):
        pos_inc = (motor_commands[:, 2] - motor_commands[:, 1]) * POS_RESOLUTION

        pos_inc[(pos_inc > 0) & (pos_inc <= POS_MIN_STEP)] = POS_MIN_STEP
        pos_inc[(pos_inc < 0) & (pos_inc >= -POS_MIN_STEP)] = -POS_MIN_STEP

        motor_commands[:, 1] = motor_commands[:, 1] + pos_inc

        if round(abs(motor_commands[0, 2] - motor_commands[0, 1]), 2) >= POS_RESOLUTION:
            mmc.send_command(1, motor_commands[0, 1], 0.00, K_P, K_D, 0.00)
        if round(abs(motor_commands[1, 2] - motor_commands[1, 1]), 2) >= POS_RESOLUTION:
            mmc.send_command(2, motor_commands[1, 1], 0.00, K_P, K_D, 0.00)
        if round(abs(motor_commands[2, 2] - motor_commands[2, 1]), 2) >= POS_RESOLUTION:
            mmc.send_command(3, motor_commands[2, 1], 0.00, K_P, K_D, 0.00)

        time.sleep(SLEEP_INTERVAL)

    # Wait until stepper motor reaches final position
    if (step_time-(time.clock()-current_time)) > 0:
        time.sleep(step_time-(time.clock()-current_time))

    return motor_commands


def go_to_start():
    """
    Goes to start position and zeroes the encoder.
    """
    global ID_R, POS_R, VEL_R, I_R

    motor_commands = np.array([[1, 0, 0.0],
                               [2, 0, 0.0],
                               [3, 0, 0.0],
                               [4, 0, 0.0]])

    rospy.loginfo("Moving to start position")

    return motor_commands


def return_to_start(motor_commands):
    """
    Returns to zeroed position, then returns to start position and then zeros in preparation for power cycle or turn off.
    This way, when the motor is turned back on, the encoder's absolute zero allows for go_to_start to work properly.
    """

    """ Return to start positions """
    motor_commands[0, 2] = 0
    motor_commands[1, 2] = 0
    motor_commands[2, 2] = -MOTOR3_START
    motor_commands[3, 2] = -MOTOR4_START
    """ END EXAMPLE """

    motor_commands = ramp_to_pos(motor_commands)


def callback(new_commands):
    """
    Retrieves commands from the rostopic, sends the commands to the motor.
    """
    global motor_commands
    rospy.loginfo("Received command. Simulating...")
    print(new_commands.data)

    # print(new_commands)
    new_angles = np.asarray(new_commands.data[0:4])
    new_angles = np.clip(new_angles, a_min=[M1_MIN, M2_MIN, M3_MIN, M4_MIN], a_max=[M1_MAX, M2_MAX, M3_MAX, M4_MAX])

    motor_commands[0, 2] = new_angles[0] * 1  # originally -1
    motor_commands[1, 2] = new_angles[1] * 1  # originally -1
    motor_commands[2, 2] = new_angles[2] * 1  # originally -PULLEY_RATIO
    motor_commands[3, 2] = new_angles[3] * 1  # originally -1

    motor_commands = ramp_to_pos(motor_commands)
    motor_commands[:, 1] = motor_commands[:, 2]


def listener():
    """
    Typical listener function for listener ros nodes.
    """
    rospy.Subscriber("motion_plan", FloatList, callback)
    rospy.spin()


if __name__ == "__main__":
    # Initialize all ROS publishers and ROS node
    joint1_pub = rospy.Publisher('agrobot_arm_v2/joint1_position_controller/command', Float64, queue_size=1)
    joint2_pub = rospy.Publisher('agrobot_arm_v2/joint2_position_controller/command', Float64, queue_size=1)
    joint3_pub = rospy.Publisher('agrobot_arm_v2/joint3_position_controller/command', Float64, queue_size=1)
    joint4_pub = rospy.Publisher('agrobot_arm_v2/joint4_position_controller/command', Float64, queue_size=1)
    joint_pub = [joint1_pub, joint2_pub, joint3_pub, joint4_pub]
    rospy.init_node('motor_controller_simulation', anonymous=True)

    # Create mmc class object
    global mmc
    mmc = MotorModuleController_Simulation()

    # Go to start position
    motor_commands = go_to_start()

    # Wait for commands
    listener()
