#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
import rospy
import MotorModuleController as mm
from std_msgs.msg import String
from std_msgs.msg import Float32
from agrobot_arm_v2.msg import FloatList

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_solid_state_relay_v2 import BrickletSolidStateRelayV2
from tinkerforge.bricklet_industrial_dual_relay import BrickletIndustrialDualRelay
from struct import *

HOST = "localhost"
PORT = 4223
UID_ssr = "F1t"
UID_idr = "KqZ"

# Global variables declaring resolution of movement in radians
POS_RESOLUTION = 0.01
POS_MIN_STEP = 0.01

# Motor start points for the robot arm at start up
MOTOR3_START = -1.34
MOTOR4_START = 1.571  # STEPPER MOTOR START POINT (TO BE CHANGED)

# PD gain settings for the Mini Cheetah motors
K_P = 10.0 # PD = 10, 0.6 for soft smooth movement 30,1
K_D = 0.6

# Pulley ratio for forearm link joint
PULLEY_RATIO = 1.56

# Soft limits on range of motor movement
M1_MIN = -3.1415
M1_MAX = 3.1415
M2_MIN = -3.1415
M2_MAX = 0
M3_MIN = MOTOR3_START
M3_MAX = 2.5                            # less than P_MAX/PULLEY_RATIO
M4_MIN = -3.1415/2                        # STEPPER MOTOR LIMIT (TO BE CHANGED)
M4_MAX = 3.1415/2                         # STEPPER MOTOR LIMIT (TO BE CHANGED)

# Global variables for data received
ID_R = POS_R = VEL_R = I_R = POS_R_STEP = 0

# The speed at which commands can be sent to the motors over CAN
SLEEP_INTERVAL = 0.002                  # in seconds


def ramp_to_pos(motor_commands):
    """
    Takes array of motor ID, current positions, and desired positions and returns same array except with a different
    second column (altered current positions). Note for manual control of motors, this function is only used for
    bringing the arm to start position and returning to start position at the end. Another function is used when
    commands are actually sent to the motors
    """
    # For stepper motor
    final_pos_step = motor_commands[3, 2]
    mmc.send_command(4, motor_commands[3, 2], 1.0, K_P, K_D, 0.00)
    motor_commands[3, 1] = motor_commands[3, 2]
    stepper_not_done = True

    # Ramps up motor position until the final position is reached (either MC motors or stepper motor)
    while np.any(np.absolute(motor_commands[:, 2] - motor_commands[:, 1]) >= POS_RESOLUTION * np.ones(4)) or \
            stepper_not_done:
        # One position increment between the current motor position and the final
        pos_inc = (motor_commands[:, 2] - motor_commands[:, 1]) * POS_RESOLUTION

        # Makes sure the position increment doesn't get too small (there's a minimum step size)
        pos_inc[(pos_inc > 0) & (pos_inc <= POS_MIN_STEP)] = POS_MIN_STEP
        pos_inc[(pos_inc < 0) & (pos_inc >= -POS_MIN_STEP)] = -POS_MIN_STEP

        # Adds the position increment to the current position (which is changed for the next iteration)
        motor_commands[:, 1] = motor_commands[:, 1] + pos_inc

        # Sends the motor position command to the respective motor if the motor has not yet reached final position
        if round(abs(motor_commands[0, 2] - motor_commands[0, 1]), 2) >= POS_RESOLUTION:
            mmc.send_command(1, motor_commands[0, 1], 0.00, K_P, K_D, 0.00)
        if round(abs(motor_commands[1, 2] - motor_commands[1, 1]), 2) >= POS_RESOLUTION:
            mmc.send_command(2, motor_commands[1, 1], 0.00, K_P, K_D, 0.00)
        if round(abs(motor_commands[2, 2] - motor_commands[2, 1]), 2) >= POS_RESOLUTION:
            mmc.send_command(3, motor_commands[2, 1], 0.00, K_P, K_D, 0.00)
        if abs(final_pos_step - mmc.get_position_stepper()) <= POS_RESOLUTION:
            stepper_not_done = False

        time.sleep(SLEEP_INTERVAL)

    return motor_commands


def manual_to_pos(motor_commands):
    """
    Takes array of motor ID, current positions, and desired positions and returns same array except with a different
    second column (altered current positions). Only for incremental manual commands that come over ROS topic.
    """
    # For stepper motor--essentially just takes small steps
    final_pos_step = round(motor_commands[3, 2], 2)
    if (int(round(final_pos_step*100, 2)) % 4) <= POS_RESOLUTION:
        mmc.send_command(4, motor_commands[3, 2], 0.8, K_P, K_D, 0.00)
    motor_commands[3, 1] = motor_commands[3, 2]
    stepper_not_done = False  # instead of True like ramp_to_pos

    # Ramps to final position, but steps are so small that generally it just makes an incremental step.
    while np.any(np.absolute(motor_commands[:, 2] - motor_commands[:, 1]) >= POS_RESOLUTION * np.ones(4)) or \
            stepper_not_done:
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
        # if abs(final_pos_step - POS_R_STEP) <= POS_RESOLUTION:
        #     stepper_not_done = False

        time.sleep(SLEEP_INTERVAL)

    return motor_commands


def go_to_start():
    """
    Goes to start position and zeroes the encoder.
    """
    # Set motor commands
    motor_commands = np.array([[1, 0, 0],
                               [2, 0, 0],
                               [3, 0, 0],
                               [4, 0, MOTOR4_START]])

    # Find current position of motors relative to absolute zero
    for k in range(1, 4):
        mmc.send_command(k, 0, 0, 0, 0, 0)
        time.sleep(0.1)
        motor_commands[(k - 1), 1] = mmc.get_position()  # POS_R

    # Keep motor 3 in place at startup
    motor_commands[2, 2] = motor_commands[2, 1]

    rospy.loginfo("Moving to start position")

    # Stiffening base motor in case it is already at start position
    if abs(motor_commands[0, 1]) <= POS_RESOLUTION:
        mmc.send_command(1, 0.00, 0.00, 10.00, 0.70, 0.00)

    # Move stepper motor to position first (otherwise Link 2 jerks forward, overtorquing stepper)
    motor_commands = ramp_to_pos(motor_commands)
    time.sleep(2)

    # Move motor 3 to position
    motor_commands = np.array([[1, 0, 0],
                               [2, 0, 0],
                               [3, 0, MOTOR3_START],
                               [4, 0, MOTOR4_START]])
    motor_commands = ramp_to_pos(motor_commands)
    motor_commands[:, 1] = motor_commands[:, 2] = 0

    rospy.loginfo("Moved to start position!")

    # Zeroing motor 3 specifically
    mmc.send_command(3, 0, 0, 0, 0, 0)
    mmc.zero_motor(3)
    mmc.send_command(3, 0.00, 0.00, 10.00, 0.70, 0.00)

    # Zeroing motor 4 specifically (the stepper)
    mmc.zero_motor(4)

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
    # rospy.loginfo("Received command")
    # print(new_commands.data)

    # Converts data from ROS message to array and then clips final positions within soft limits
    new_angles = np.asarray(new_commands.data[0:4])
    new_angles = np.clip(new_angles, a_min=[M1_MIN, M2_MIN, M3_MIN, M4_MIN], a_max=[M1_MAX, M2_MAX, M3_MAX, M4_MAX])

    # Adjust commands (they differ between simulation/urdf and the real thing)
    motor_commands[0, 2] = new_angles[0] * -1
    motor_commands[1, 2] = new_angles[1] * -1
    motor_commands[2, 2] = new_angles[2] * -PULLEY_RATIO
    motor_commands[3, 2] = new_angles[3] * -1

    motor_commands = manual_to_pos(motor_commands)
    motor_commands[:, 1] = motor_commands[:, 2]


def turn_off():
    """
    Turns off all motors and solid state relay upon pressing "Ctrl-C" in the terminal
    """
    global motor_commands

    # Return to start position
    rospy.loginfo("Returning to start.")
    return_to_start(motor_commands)
    rospy.loginfo("Shutting down relays and motors.")

    # Disable all the motors
    mmc.disable_motor(1)
    mmc.disable_motor(2)
    mmc.disable_motor(3)

    # Turn off the relays
    # time.sleep(0.5)
    # idr.set_value(False, False)
    time.sleep(0.5)
    ssr.set_state(False)
    ipcon.disconnect()


def listener():
    """
    Typical listener function for listener ros nodes.
    """
    rospy.Subscriber("motion_plan", FloatList, callback)
    rospy.on_shutdown(turn_off)
    rospy.spin()


if __name__ == "__main__":
    # Connect to TinkerForge stack
    ipcon = IPConnection()
    ssr = BrickletSolidStateRelayV2(UID_ssr, ipcon)
    idr = BrickletIndustrialDualRelay(UID_idr, ipcon)
    ipcon.connect(HOST, PORT)

    # Create class object (see MotorModuleController class)
    mmc = mm.MotorModuleController()

    # Turn on relays
    rospy.loginfo("Turning on relays.")
    # idr.set_value(True, False)
    # rospy.loginfo("Precharge circuit enabled.")
    # time.sleep(2)
    rospy.loginfo("Solid state relay opened.")
    ssr.set_state(True)
    time.sleep(2)

    # Enable motors
    mmc.enable_motor(1)
    mmc.enable_motor(2)
    mmc.enable_motor(3)
    mmc.enable_motor(4)

    # Go to start position
    motor_commands = go_to_start()

    # Listen for commands
    listener()
