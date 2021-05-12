#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
import rospy
import csv
import os
import matplotlib.pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import Float32
from agrobot_arm_v2.msg import FloatList

from struct import *

PULLEY_RATIO = 1.56

# This function receives the position, velocity, and current data from each motor and sticks it into a dataset list.
def callback(new_data):
    global motor_data
    #print(new_data.data)

    data_batch = np.asarray(new_data.data)
    motor_data[int(data_batch[0]-1)] = np.append(motor_data[int(data_batch[0]-1)], np.reshape(data_batch[1:5], (4,1)), axis=1)


# This function plots the collected data and exports it as a .csv file.
def plot_and_csv():
    global motor_data

    for k in range(len(motor_data)):
        # remove zeros from first column, generated when initializing
        motor_data[k] = np.delete(motor_data[k], 0, axis=1)

        # correct time vector
        motor_data[k][0, :] = motor_data[k][0, :] - motor_data[k][0, 0]

    # Place motor data into .csv file
    folder_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(folder_path, 'motor_data_analysis/motor_data.csv')
    with open(file_path, 'w', newline='') as my_file:
        csv_writer = csv.writer(my_file, delimiter=',')

        for k in range(len(motor_data)):
            for row in motor_data[k]:
                csv_writer.writerow(row)

    print("Exported motor data to .csv file.")

    # Plot motor position over time.
    plt.figure(1)
    for k in range(len(motor_data)):
        plt.plot(motor_data[k][0, :], motor_data[k][1, :], label="Motor "+str(k+1))
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
    plt.title("Position vs Time")
    plt.legend()
    plt.grid()

    # Plot motor velocity over time.
    plt.figure(2)
    for k in range(len(motor_data)):
        plt.plot(motor_data[k][0, :], motor_data[k][2, :], label="Motor "+str(k+1))
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rad/s)")
    plt.title("Velocity vs Time")
    plt.legend()
    plt.grid()

    # Plot motor current over time.
    plt.figure(3)
    for k in range(len(motor_data)):
        plt.plot(motor_data[k][0, :], motor_data[k][3, :], label="Motor "+str(k+1))
    plt.xlabel("Time (s)")
    plt.ylabel("Current (A)")
    plt.title("Current vs Time")
    plt.legend()
    plt.grid()

    plt.show()


# This function is the listener that waits until data is received, during which the callback is executed.
def listener():
    """
    Typical listener function for listener ros nodes.
    """
    rospy.Subscriber("motor_data", FloatList, callback)
    rospy.on_shutdown(plot_and_csv)
    rospy.spin()


if __name__ == "__main__":
    # Initialize ROS publisher and node
    pub = rospy.Publisher('motor_data', FloatList, queue_size=1)
    rospy.init_node('plot_motor_data', anonymous=True)

    # Four motors, three data points
    motor1_data = np.zeros((4, 1))
    motor2_data = np.zeros((4, 1))
    motor3_data = np.zeros((4, 1))
    motor4_data = np.zeros((4, 1))

    motor_data = [motor1_data, motor2_data, motor3_data, motor4_data]

    # motor1_data = np.asarray([[1, 2, 3, 4], [5, 6, 7, 8]])
    # print(motor1_data)
    # motor1_data = np.delete(motor1_data, 0, axis=1)
    # print(motor1_data)

    rospy.loginfo("Connected to ROS network, awaiting incoming data...")

    listener()
