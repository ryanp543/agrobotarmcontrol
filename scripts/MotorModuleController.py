#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from agrobot_arm_v2.msg import FloatList

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_can_v2 import BrickletCANV2
from struct import *


# This file defines a class type for reference in motor_controller.py, motor_controller_manual.py, and
# motor_controller_simulation.py
# All information regarding the bit commands sent to the Mini Cheetah motors can be found at the following link:
# https://docs.google.com/document/d/1dzNVzblz6mqB3eZVEMyi2MtSngALHdgpTaDJIW_BpS4/edit
class MotorModuleController():
    def __init__(self):
        # Initializing a class object creates variables IDs for the TinkerForge stack.
        HOST = "localhost"
        PORT = 4223
        UID_ssr = "F1t"
        UID_can = "GmQ"
        UID_idr = "KqZ"

        # Initialize ROS node and publisher for motor_data, throws error if otherwise
        try:
            rospy.init_node('motor_controller', anonymous=True)
            self.pub = rospy.Publisher('motor_data', FloatList, queue_size=1)
        except:
            rospy.loginfo("Failed to create ROS Node")

        # Connect to Tinkerforge stack and configure all settings for CANbus bricklet
        try:
            ipcon = IPConnection()
            self.can = BrickletCANV2(UID_can, ipcon)

            ipcon.connect(HOST, PORT)
            self.can.set_transceiver_configuration(1000000, 625, self.can.TRANSCEIVER_MODE_NORMAL)
            self.can.register_callback(self.can.CALLBACK_FRAME_READ, self.cb_frame_read)
            self.can.set_frame_read_callback_configuration(True)

            rospy.loginfo('Connected to motor module controller')
        except:
            rospy.loginfo('Failed to connect to Tinkerforge Bricklets.')
            pass

        # Constants for command transfer over CAN lines
        self.P_MIN = -25.0 / 6
        self.P_MAX = 25.0 / 6
        self.V_MIN = -45.0
        self.V_MAX = 45.0
        self.KP_MIN = 0.0
        self.KP_MAX = 500.0
        self.KD_MIN = 0.0https://docs.google.com/document/d/1dzNVzblz6mqB3eZVEMyi2MtSngALHdgpTaDJIW_BpS4/edit
        self.KD_MAX = 5.0
        self.T_MIN = -18.0
        self.T_MAX = 18.0
        self.I_MIN = -40
        self.I_MAX = 40
        self.DATA_LENGTH = 8

        # Initialize received motor data variables
        self.ID_R = 0
        self.POS_R = 0
        self.VEL_R = 0
        self.I_R = 0
        self.POS_R_STEP = 0

        # Mechanical constants needed for calculating motor data variables
        self.PULLEY_RATIO = 1.56


    def send_command(self, id, p_des, v_des, kp_des, kd_des, i_ff):
        """
        send_command(desired position, desired velocity, position gain, velocity gain, feed-forward current)
        Sends data over CAN, reads response, and populates rx_data with the response.
        """
        data = [0] * self.DATA_LENGTH

        # Convert float value to unsigned integer
        id = int(id)
        pos = self.float_to_uint(p_des, self.P_MIN, self.P_MAX, 16)
        vel = self.float_to_uint(v_des, self.V_MIN, self.V_MAX, 12)
        kp = self.float_to_uint(kp_des, self.KP_MIN, self.KP_MAX, 12)
        kd = self.float_to_uint(kd_des, self.KD_MIN, self.KD_MAX, 12)
        current_ff = self.float_to_uint(i_ff, self.T_MIN, self.T_MAX, 12)

        # Convert unsigned integer to bit commands to send over CANBus
        data[0] = pos >> 8
        data[1] = pos & 0xF
        data[2] = vel >> 4
        data[3] = ((vel & 0xF) << 4) | (kp >> 8)
        data[4] = kp & 0xFF
        data[5] = kd >> 4
        data[6] = ((kd & 0xF) << 4) | (current_ff >> 8)
        data[7] = current_ff & 0xff

        self.can.write_frame(self.can.FRAME_TYPE_STANDARD_DATA, id, data)
        # print("Sent data: ", data)


    def enable_motor(self, id):
        """
        Puts motor with CAN ID "id" into torque-control mode.  2nd red LED will turn on
        """
        # Directly writes bit commands to be sent over CANBus to enable motor
        data = [0] * self.DATA_LENGTH
        data[0] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[1] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[2] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[3] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[4] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[5] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[6] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[7] = int.from_bytes(b'\xFC', byteorder='big', signed=False)

        self.can.write_frame(self.can.FRAME_TYPE_STANDARD_DATA, id, data)
        time.sleep(0.1)
        print("Enabled Motor ", id)


    def disable_motor(self, id):
        """
        Removes motor with CAN ID "id" from torque-control mode.  2nd red LED will turn off
        """
        # Directly writes bit commands to be sent over CANBus to disable motor
        data = [0] * self.DATA_LENGTH
        data[0] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[1] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[2] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[3] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[4] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[5] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[6] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[7] = int.from_bytes(b'\xFD', byteorder='big', signed=False)

        self.can.write_frame(self.can.FRAME_TYPE_STANDARD_DATA, id, data)
        time.sleep(0.1)
        # print("Disabled Motor ", id)


    def zero_motor(self, id):
        # Directly writes bit commands to be sent over CANBus to zero motor encoder
        data = [0] * self.DATA_LENGTH
        data[0] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[1] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[2] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[3] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[4] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[5] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[6] = int.from_bytes(b'\xFF', byteorder='big', signed=False)
        data[7] = int.from_bytes(b'\xFE', byteorder='big', signed=False)

        self.can.write_frame(self.can.FRAME_TYPE_STANDARD_DATA, id, data)
        time.sleep(0.1)
        # print("Zeroed Motor ", id)


    def float_to_uint(self, x, x_min, x_max, bits):
        """
        Converts float to unsigned int
        """
        span = float(x_max - x_min)
        offset = float(x_min)
        return int((x - offset) * float(((1 << bits) - 1) / span))


    def uint_to_float(self, x_int, x_min, x_max, bits):
        """
        Converts unsigned int to float
        """
        span = float(x_max - x_min)
        offset = float(x_min)
        return (float(x_int)) * span / (float((1 << bits) - 1)) + offset


    def cb_frame_read(self, frame_type, identifier, data):
        """
        Every time the motor sends back its position, velocity, and current data, data is sent to the motor_data rostopic.
        """
        # Collects bit data packet reception and converts it to unsigned integer
        self.ID_R = data[0]
        pos_int = (data[1] << 8) | data[2]
        vel_int = (data[3] << 4) | (data[4] >> 4)
        i_int = ((data[4] & 0xF) << 8) | data[5]

        # Converts unsigned integer back to float
        self.POS_R = self.uint_to_float(pos_int, self.P_MIN, self.P_MAX, 16) * -1
        self.VEL_R = self.uint_to_float(vel_int, self.V_MIN, self.V_MAX, 12) * -1
        self.I_R = self.uint_to_float(i_int, self.I_MIN, self.I_MAX, 12)

        # For motor 3, divide by pulley ratio to get the actual joint angle position and velocity
        if self.ID_R == 3:
            self.POS_R = self.POS_R / self.PULLEY_RATIO
            self.VEL_R = self.VEL_R / self.PULLEY_RATIO

        # For stepper motor wrist, flip sign
        if self.ID_R == 4:
            self.POS_R_STEP = -1.0 * self.POS_R

        # Print just for debugging
        print("Motor ", self.ID_R, " is at position ", self.POS_R)
        print("Motor ", self.ID_R, " has velocity ", self.VEL_R)
        print("Motor ", self.ID_R, " has current ", self.I_R)

        # Publish data to ROS topic for later analysis (or storage in .csv file)
        # print(publish_data)
        publish_data = FloatList()
        publish_data.data = [self.ID_R, rospy.get_time(), self.POS_R, self.VEL_R, self.I_R]
        self.pub.publish(publish_data)


    def get_position(self):
        return self.POS_R
    

    def get_position_stepper(self):
        return self.POS_R_STEP


# if __name__ == "__main__":
#     mmc = MotorModuleController()
