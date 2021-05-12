#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import numpy as np
import csv
import os
# import kinpy as kp
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
# from kinpy import jacobian, chain

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

from struct import *

PULLEY_RATIO = 1.56
GEAR_RATIO = 6

FIT_CURVE_A = -0.0005
FIT_CURVE_B = 0.0884
FIT_CURVE_C = -0.0304


def calculate_joint_torques(joint_amps):
    joint_torques = []
    for n in range(len(joint_amps)):
        tau = FIT_CURVE_A*(joint_amps[n]**2) + FIT_CURVE_B*joint_amps[n] + FIT_CURVE_C
        joint_torques.append(GEAR_RATIO * tau)

    return np.asarray(joint_torques)


def calculate_endpoint_forces(motor_data):

    robot = URDF.from_xml_file("agrobot_arm_v2_motor_analysis.urdf")
    # tree = kdl_tree_from_urdf_model(robot)
    # print tree.getNrOfSegments()
    # chain = tree.getChain("base_link", "Link_4")
    # print chain.getNrOfJoints()

    kdl_kin = KDLKinematics(robot, "base_link", "Link_4")
    force_x = []
    force_y = []
    force_z = []
    for j in range(len(motor_data[0][0])):
        q = np.asarray([motor_data[0][1, j], motor_data[1][1, j], motor_data[2][1, j], motor_data[3][1, j]])
        J = np.asarray(kdl_kin.jacobian(q)[0:3, 0:3])
        tau = np.asarray([motor_data[0][3, j], motor_data[1][3, j], motor_data[2][3, j], motor_data[3][3, j]])

        invJT = np.linalg.inv(np.transpose(J))
        forces = np.dot(invJT, tau[:3])

        force_x.append(forces[0])
        force_y.append(forces[1])
        force_z.append(forces[2])

    return force_x, force_y, force_z


def plot_and_csv(motor_data):
    # Making all current data lists the same length
    data_lengths = []
    t_final_list = []
    for k in range(len(motor_data)):
        data_lengths.append(len(motor_data[k][0, :]))
        t_final_list.append(motor_data[k][0, -1])
    min_t_final = min(t_final_list)
    max_length = 10*max(data_lengths)

    t = np.linspace(0, min_t_final, num=max_length)
    motor_data_new = []
    for k in range(len(motor_data)):
        motor_data_new.append(np.zeros((4, max_length)))

        f_pos = interp1d(motor_data[k][0, :], motor_data[k][1, :])
        f_vel = interp1d(motor_data[k][0, :], motor_data[k][2, :])
        f_cur = interp1d(motor_data[k][0, :], motor_data[k][3, :])

        motor_data_new[k][0] = t
        motor_data_new[k][1] = f_pos(t)
        motor_data_new[k][2] = f_vel(t)
        motor_data_new[k][3] = calculate_joint_torques(f_cur(t))


    plt.figure(1)
    for k in range(len(motor_data)):
        plt.plot(motor_data_new[k][0], motor_data_new[k][1], label="Motor "+str(k+1))
        # plt.plot(motor_data[k][0], motor_data[k][1], label="Motor "+str(k+1))
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
    plt.title("Position vs Time")
    plt.legend()
    plt.grid()

    plt.figure(2)
    for k in range(len(motor_data)):
        plt.plot(motor_data_new[k][0], motor_data_new[k][2], label="Motor "+str(k+1))
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rad/s)")
    plt.title("Velocity vs Time")
    plt.legend()
    plt.grid()

    plt.figure(3)
    for k in range(len(motor_data)):
        plt.plot(motor_data_new[k][0], motor_data_new[k][3], label="Motor "+str(k+1))
    plt.xlabel("Time (s)")
    plt.ylabel("Joint Torque (Nm)")
    plt.title("Joint Torque vs Time")
    plt.legend()
    plt.grid()

    force_x, force_y, force_z = calculate_endpoint_forces(motor_data_new)

    plt.figure(4)
    plt.plot(motor_data_new[0][0], force_x, label="F_x")
    plt.plot(motor_data_new[0][0], force_y, label="F_y")
    plt.plot(motor_data_new[0][0], force_z, label="F_z")
    plt.xlabel("Time (s)")
    plt.ylabel("Force (N)")
    plt.ylim([-75, 75])
    plt.title("End Effector Force vs Time")
    plt.legend()
    plt.grid()

    plt.show()


if __name__ == "__main__":
    robot = URDF.from_xml_file("agrobot_arm_v2_motor_analysis.urdf")
    # tree = kdl_tree_from_urdf_model(robot)
    # print tree.getNrOfSegments()
    # chain = tree.getChain("base_link", "Link_4")
    # print chain.getNrOfJoints()

    kdl_kin = KDLKinematics(robot, "base_link", "Link_4")

    q = np.asarray([0, -np.pi/4, 0, 0])
    print(kdl_kin.jacobian(q))
    J = np.asarray(kdl_kin.jacobian(q)[0:3, 0:3])
    tau = np.asarray([0, -16, 16, 0])

    invJT = np.linalg.inv(np.transpose(J))

    print(invJT)
    forces = np.dot(invJT, tau[:3])
    print(forces)



    # Four motors, three data points
    motor1_data = np.zeros((4, 1))
    motor2_data = np.zeros((4, 1))
    motor3_data = np.zeros((4, 1))
    motor4_data = np.zeros((4, 1))

    motor_data = [motor1_data, motor2_data, motor3_data, motor4_data]

    folder_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(folder_path, 'motor_data_test2.csv')
    with open(file_path, 'r') as myfile:
        csvreader = csv.reader(myfile, delimiter=',')
        data = []
        for row in csvreader:
            data.append(row)
        data = [[float(y) for y in x] for x in data]

        for j in range(0, len(data), 4):
            batch = np.asarray([data[j][:], data[j+1][:], data[j+2][:], data[j+3][:]])
            motor_data[int(j/4)] = np.append(motor_data[int(j/4)], batch, axis=1)
            motor_data[int(j/4)] = np.delete(motor_data[int(j/4)], 0, axis=1)

    # plot_and_csv(motor_data)
