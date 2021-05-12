#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
import rospy
import pygame
from std_msgs.msg import String
from agrobot_arm_v2.msg import FloatList
from struct import *

HOST = "localhost"
PORT = 4223
UID_step = "6yJzPW"

# Motor settings
POS_MIN_STEP = 0.02   # Minimum step to be taken by stepper at command
MOTOR3_START = -1.68  # Motor 3 Start
MOTOR4_START = 1.571  # STEPPER MOTOR START POINT (TO BE CHANGED)

# Soft limits on motor angular positions
M1_MIN = -3.1415
M1_MAX = 3.1415
M2_MIN = -3.1415
M2_MAX = 0
M3_MIN = MOTOR3_START
M3_MAX = 2.5                            # less than P_MAX/PULLEY_RATIO
M4_MIN = -3.1415/2                      # STEPPER MOTOR LIMIT (TO BE CHANGED)
M4_MAX = 3.1415/2                       # STEPPER MOTOR LIMIT (TO BE CHANGED)

DISPLAY = (1, 1)
FLAGS = 0
DEPTH = 32

# Speed at which commands are sent if a key is pressed and held
SPEED = 50 # 50


# Exits pygame if the ROS node is shut down via Ctrl-C in the terminal
def turn_off():
    rospy.loginfo("Exiting manual control via ROSLaunch shutdown.")
    running = False
    pygame.quit()


if __name__ == "__main__":
    # Initializes ROS node and publishers
    rospy.init_node('manual_control', anonymous=True)
    rospy.on_shutdown(turn_off)
    pub = rospy.Publisher('motion_plan', FloatList, queue_size=1)
    pub_tool = rospy.Publisher('tool_changer', String, queue_size=1)

    # Initialize and opten pygame display
    pygame.init()
    screen = pygame.display.set_mode(DISPLAY, FLAGS, DEPTH)
    running = True

    # Initialize variables for user commands
    joint_angles = FloatList()
    joint_angles.data = [0, 0, 0, 0]
    tool_command = String()
    tool_attached = False

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                sys.exit()

        # Retrieve keys pressed
        keys = pygame.key.get_pressed()
        keypressed = False

        # Keys that will exit manual control and quit pygame
        if keys[pygame.K_ESCAPE] or keys[pygame.K_p] or ((keys[pygame.K_LCTRL] or keys[pygame.K_RCTRL]) and keys[pygame.K_c]):
            rospy.loginfo("Exiting manual control")
            running = False
            pygame.quit()

        # Pressing space will send a command to start the tool changer attaching process
        if keys[pygame.K_SPACE] and not tool_attached:
            tool_command.data = "screw"
            tool_attached = True
            rospy.loginfo("Sending tool attach command")
            pub_tool.publish(tool_command)

        # Pressing backspace will detach tool. The 'o' option is for debugging purposes (or tool is attached at startup)
        if (keys[pygame.K_BACKSPACE] and tool_attached) or keys[pygame.K_o]:
            tool_command.data = "unscrew"
            tool_attached = False
            rospy.loginfo("Sending tool detach command")
            pub_tool.publish(tool_command)

        # Pressing q causes motor 1 to go in the positive direction
        if keys[pygame.K_q]:
            keypressed = True
            if joint_angles.data[0] < M1_MAX:
                joint_angles.data[0] += POS_MIN_STEP

        # Pressing a causes motor 1 to go in the negative direction
        if keys[pygame.K_a]:
            keypressed = True
            if joint_angles.data[0] > M1_MIN:
                joint_angles.data[0] -= POS_MIN_STEP

        # Pressing w causes motor 2 to go in the positive direction
        if keys[pygame.K_w]:
            keypressed = True
            if joint_angles.data[1] < M2_MAX:
                joint_angles.data[1] += POS_MIN_STEP

        # Pressing s causes motor 2 to go in the negative direction
        if keys[pygame.K_s]:
            keypressed = True
            if joint_angles.data[1] > M2_MIN:
                joint_angles.data[1] -= POS_MIN_STEP

        # Pressing e causes motor 3 to go in the positive direction
        if keys[pygame.K_e]:
            keypressed = True
            if joint_angles.data[2] < M3_MAX:
                joint_angles.data[2] += POS_MIN_STEP

        # Pressing d causes motor 3 to go in the negative direction
        if keys[pygame.K_d]:
            keypressed = True
            if joint_angles.data[2] > M3_MIN:
                joint_angles.data[2] -= POS_MIN_STEP

        # Pressing r causes motor 4 to go in the positive direction
        if keys[pygame.K_r]:
            keypressed = True
            if joint_angles.data[3] < M4_MAX:
                joint_angles.data[3] += POS_MIN_STEP

        # Pressing f causes motor 4 to go in the negative direction
        if keys[pygame.K_f]:
            keypressed = True
            if joint_angles.data[3] > M4_MIN:
                joint_angles.data[3] -= POS_MIN_STEP

        # Can send multiple joint commands at the same time for different motors.
        if keypressed:
            print(['%.2f' % elem for elem in joint_angles.data])
            pub.publish(joint_angles)

        pygame.time.wait(SPEED)



