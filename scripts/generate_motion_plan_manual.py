#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_solid_state_relay_v2 import BrickletSolidStateRelayV2
from tinkerforge.bricklet_can_v2 import BrickletCANV2
from tinkerforge.brick_silent_stepper import BrickSilentStepper
from struct import *

import csv
from pynput import keyboard

FILEPATH = './src/agrobot_arm_v2/scripts/motion_plans/new_motion_plan_RENAME.csv'

HOST = "localhost"
PORT = 4223
UID_ssr = "F1t"
UID_can = "GmQ"
UID_step = "6yJzPW"

P_MIN = -25.0 / 6
P_MAX = 25.0 / 6
V_MIN = -45.0
V_MAX = 45.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
T_MIN = -18.0
T_MAX = 18.0
I_MIN = -40
I_MAX = 40
POS_RESOLUTION = 0.01
POS_MIN_STEP = 0.01
DATA_LENGTH = 8
MOTOR3_START = -1.68
MOTOR4_START = 1.571  # STEPPER MOTOR START POINT (TO BE CHANGED)

K_P = 30.00
K_D = 1.0

PULLEY_RATIO = 1.56

STEP_GEAR_RATIO = 25.2
STEP_MAX_VELOCITY = 65535
STEP_CURRENT = 0.67 * 2
STEP_ACCEL = 60000
STEP_DECEL = 60000
STEP_RESOLUTION = 1.8 * (np.pi / 180)   # in rad/step
STEP_BACKLASH = 150 / 60                # in deg (note: backlash is given as 150 arc minutes)
STEP_CURRENT = 1000                     # stepper drive current
STEP_SET_MICROSTEP = 0                  # Tinkerforge setting for microsteps
STEP_FROM_POS = STEP_GEAR_RATIO * (2 ** (8 - STEP_SET_MICROSTEP)) / STEP_RESOLUTION

M1_MIN = -3.1415
M1_MAX = 3.1415
M2_MIN = -3.1415
M2_MAX = 0
M3_MIN = MOTOR3_START
M3_MAX = 2.5                            # less than P_MAX/PULLEY_RATIO
M4_MIN = -3.1415                        # STEPPER MOTOR LIMIT (TO BE CHANGED)
M4_MAX = 3.1415                         # STEPPER MOTOR LIMIT (TO BE CHANGED)

ID_R = POS_R = VEL_R = I_R = 0
WRITE_LIST = [0, 0, 0, 0]

SLEEP_INTERVAL = 0.002                  # in seconds


class MotorModuleController():
    def __init__(self):
        try:
            self.rx_data = [0, 0, 0, 0, 0, 0]
            self.rx_values = [0, 0, 0, 0]
            self.tx_data = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            rospy.loginfo('Connected to motor module controller')
        except:
            rospy.loginfo('Failed to connect to motor module controller')
            pass

    def send_command(self, id, p_des, v_des, kp_des, kd_des, i_ff):
        """
        send_command(desired position, desired velocity, position gain, velocity gain, feed-forward current)
        Sends data over CAN, reads response, and populates rx_data with the response.
        """
        data = [0] * DATA_LENGTH

        id = int(id)
        pos = float_to_uint(p_des, P_MIN, P_MAX, 16)
        vel = float_to_uint(v_des, V_MIN, V_MAX, 12)
        kp = float_to_uint(kp_des, KP_MIN, KP_MAX, 12)
        kd = float_to_uint(kd_des, KD_MIN, KD_MAX, 12)
        current_ff = float_to_uint(i_ff, T_MIN, T_MAX, 12)
        data[0] = pos >> 8
        data[1] = pos & 0xF
        data[2] = vel >> 4
        data[3] = ((vel & 0xF) << 4) | (kp >> 8)
        data[4] = kp & 0xFF
        data[5] = kd >> 4
        data[6] = ((kd & 0xF) << 4) | (current_ff >> 8)
        data[7] = current_ff & 0xff

        can.write_frame(can.FRAME_TYPE_STANDARD_DATA, id, data)
        # print("Sent data: ", data)

    def enable_motor(self, id):
        """
        Puts motor with CAN ID "id" into torque-control mode.  2nd red LED will turn on
        """

        data = [0] * DATA_LENGTH
        data[0] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[1] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[2] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[3] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[4] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[5] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[6] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[7] = int.from_bytes(b'\xFC', byteorder='big', signed=False)

        can.write_frame(can.FRAME_TYPE_STANDARD_DATA, id, data)
        time.sleep(0.1)
        print("Enabled Motor ", id)

    def disable_motor(self, id):
        """
        Removes motor with CAN ID "id" from torque-control mode.  2nd red LED will turn off
        """

        data = [0] * DATA_LENGTH
        data[0] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[1] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[2] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[3] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[4] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[5] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[6] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[7] = int.from_bytes(b'\xFD', byteorder='big', signed=False)

        can.write_frame(can.FRAME_TYPE_STANDARD_DATA, id, data)
        time.sleep(0.1)
        print("Disabled Motor ", id)

    def zero_motor(self, id):

        data = [0] * DATA_LENGTH
        data[0] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[1] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[2] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[3] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[4] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[5] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[6] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[7] = int.from_bytes(b'\xFE', byteorder='big', signed=False)

        can.write_frame(can.FRAME_TYPE_STANDARD_DATA, id, data)
        time.sleep(0.1)
        # print("Zeroed Motor ", id)


def float_to_uint(x, x_min, x_max, bits):
    """
    Converts float to unsigned int
    """
    span = float(x_max - x_min)
    offset = float(x_min)
    return int((x - offset) * float(((1 << bits) - 1) / span))


def uint_to_float(x_int, x_min, x_max, bits):
    """
    Converts unsigned int to float
    """
    span = float(x_max - x_min)
    offset = float(x_min)
    return (float(x_int)) * span / (float((1 << bits) - 1)) + offset


def cb_frame_read(frame_type, identifier, data):
    """
    Every time the motor sends back its position, velocity, and current data, data is sent to the motor_data rostopic.
    """
    global ID_R, POS_R, VEL_R, I_R, WRITE_LIST

    ID_R = data[0]
    pos_int = (data[1] << 8) | data[2]
    vel_int = (data[3] << 4) | (data[4] >> 4)
    i_int = ((data[4] & 0xF) << 8) | data[5]

    POS_R = uint_to_float(pos_int, P_MIN, P_MAX, 16) * -1
    VEL_R = uint_to_float(vel_int, V_MIN, V_MAX, 12) * -1
    I_R = uint_to_float(i_int, I_MIN, I_MAX, 12)

    if ID_R == 3:
        POS_R = POS_R / PULLEY_RATIO
        VEL_R = VEL_R / PULLEY_RATIO

    WRITE_LIST[ID_R-1] = POS_R


def ramp_to_pos(motor_commands):
    """
    Takes array of motor ID, current positions, and desired positions and returns same array except with a different
    second column (altered current positions)
    """

    stepper.set_target_position(motor_commands[3, 2] * STEP_FROM_POS)
    motor_commands[3, 1] = motor_commands[3, 2]

    while np.any(np.absolute(motor_commands[:, 2] - motor_commands[:, 1]) > POS_RESOLUTION * np.ones(4)) or \
            abs(stepper.get_remaining_steps()) > 0:
        pos_inc = (motor_commands[:, 2] - motor_commands[:, 1]) * POS_RESOLUTION

        pos_inc[(pos_inc > 0) & (pos_inc < POS_MIN_STEP)] = POS_MIN_STEP
        pos_inc[(pos_inc < 0) & (pos_inc > -POS_MIN_STEP)] = -POS_MIN_STEP

        motor_commands[:, 1] = motor_commands[:, 1] + pos_inc

        if abs(motor_commands[0, 2] - motor_commands[0, 1]) > POS_RESOLUTION:
            mmc.send_command(1, motor_commands[0, 1], 0.00, K_P, K_D, 0.00)
        if abs(motor_commands[1, 2] - motor_commands[1, 1]) > POS_RESOLUTION:
            mmc.send_command(2, motor_commands[1, 1], 0.00, K_P, K_D, 0.00)
        if abs(motor_commands[2, 2] - motor_commands[2, 1]) > POS_RESOLUTION:
            mmc.send_command(3, motor_commands[2, 1], 0.00, K_P, K_D, 0.00)

        time.sleep(SLEEP_INTERVAL)

    return motor_commands


def go_to_start():
    """
    Goes to start position and zeroes the encoder.
    """
    global ID_R, POS_R, VEL_R, I_R
    stepper.set_current_position(0)

    motor_commands = np.array([[1, 0, 0],
                               [2, 0, 0],
                               [3, 0, MOTOR3_START],
                               [4, 0, MOTOR4_START]])

    for k in range(1, 4):  # change range when motor 3 is connected
        mmc.send_command(k, 0, 0, 0, 0, 0)
        time.sleep(0.1)
        motor_commands[(k - 1), 1] = POS_R

    rospy.loginfo("Moving to start position")
    motor_commands = ramp_to_pos(motor_commands)
    motor_commands[:, 1] = motor_commands[:, 2] = 0

    # Zeroing motor 3 specifically
    mmc.send_command(3, 0, 0, 0, 0, 0)
    mmc.zero_motor(3)
    mmc.send_command(3, 0.00, 0.00, 10.00, 0.70, 0.00)

    # Zeroing motor 4 specifically (the stepper)
    stepper.set_current_position(0)

    return motor_commands


def return_to_start():
    """
    Returns to zeroed position, then returns to start position and then zeros in preparation for power cycle or turn off.
    This way, when the motor is turned back on, the encoder's absolute zero allows for go_to_start to work properly.
    """

    stepper.set_target_position(-MOTOR4_START * STEP_FROM_POS)
    while abs(stepper.get_remaining_steps()) > 0:
        pass
    print("Disabled Motor  4")


def turn_off():
    """
    Turns off all motors and solid state relay upon pressing "Ctrl-C" in the terminal
    """
    global motor_commands
    ssr.set_state(False)
    mmc.disable_motor(1)
    mmc.disable_motor(2)
    mmc.disable_motor(3)
    return_to_start()
    stepper.disable()
    ipcon.disconnect()


def prepare_manual_motion():
    counter = 1
    t_final = 5
    rospy.logwarn("HOLD ARM SO IT DOESN'T FREE FALL. ARM DISABLING IN...")
    rospy.logwarn(t_final)
    t_start = time.clock()
    while (time.clock()-t_start) < t_final:
        if (time.clock()-t_start) > counter:
            rospy.logwarn(t_final-counter)
            counter = counter + 1
    rospy.logwarn("DISABLING FOR MANUAL CONTROL NOW.")
    mmc.send_command(1, 0, 0, 0, 0, 0)
    mmc.send_command(2, 0, 0, 0, 0, 0)
    mmc.send_command(3, 0, 0, 0, 0, 0)
    print("[1] Use down or left arrow keys for negative rotation.")
    print("[2] Use up or right arrow keys for positive rotation.")
    print("[3] Press enter to record position.")
    print("[4] Press esc to exit control")


def on_press(key):
    if (key == keyboard.Key.up) or (key == keyboard.Key.right):
        stepper.drive_backward()
        print(" UP")
    elif (key == keyboard.Key.down) or (key == keyboard.Key.left):
        stepper.drive_forward()
        print(" DOWN")


def on_release(key):
    global WRITE_LIST

    if key == keyboard.Key.esc:
        return False  # stop listener
    elif (key == keyboard.Key.up) or (key == keyboard.Key.right):
        stepper.stop()
    elif (key == keyboard.Key.down) or (key == keyboard.Key.left):
        stepper.stop()
    elif key == keyboard.Key.enter:
        rospy.loginfo("Setting position...")
        mmc.send_command(1, 0, 0, 0, 0, 0)
        mmc.send_command(2, 0, 0, 0, 0, 0)
        mmc.send_command(3, 0, 0, 0, 0, 0)
        WRITE_LIST[3] = (stepper.get_current_position() / STEP_FROM_POS) * -1
        writer.writerow(WRITE_LIST)
        rospy.loginfo("Position set. Move on or exit.")


if __name__ == "__main__":
    ipcon = IPConnection()
    ssr = BrickletSolidStateRelayV2(UID_ssr, ipcon)
    can = BrickletCANV2(UID_can, ipcon)
    stepper = BrickSilentStepper(UID_step, ipcon)
    rospy.init_node('generate_motion_plan_manual', anonymous=True)
    rospy.on_shutdown(turn_off)

    ipcon.connect(HOST, PORT)

    can.set_transceiver_configuration(1000000, 625, can.TRANSCEIVER_MODE_NORMAL)
    can.register_callback(can.CALLBACK_FRAME_READ, cb_frame_read)
    can.set_frame_read_callback_configuration(True)

    stepper.set_max_velocity(STEP_MAX_VELOCITY)
    stepper.set_speed_ramping(STEP_ACCEL, STEP_DECEL)
    stepper.set_motor_current(STEP_CURRENT)
    stepper.set_step_configuration(STEP_SET_MICROSTEP, True)
    # stepper.register_callback(stepper.CALLBACK_ALL_DATA, cb_stepper)
    # stepper.set_all_data_period(3*SLEEP_INTERVAL*1000)

    mmc = MotorModuleController()
    ssr.set_state(True)
    mmc.enable_motor(1)
    mmc.enable_motor(2)
    mmc.enable_motor(3)
    stepper.enable()

    motor_commands = go_to_start()

    # Sets up manual motion. Make sure to catch arm before timer is up!!!
    prepare_manual_motion()

    with open(FILEPATH, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

    rospy.loginfo("Stepper control exited. Press ctrl-C to exit ROS node.")
    rospy.spin()


