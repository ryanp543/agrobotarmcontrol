# Agrobot Arm Motor Control ROS Package

## Overview

This repository provides the ROS package used to operate the robotic arm tool changer mounted on the Bioinstrumentation agricultural rover.

The joints on the robotic arm are controlled over the CANBus network with commands that are packaged in the same packets as those described in Benjamin Katz's thesis (see citations) and sent over the CANBus TinkerForge bricklet in the control box. Each time a command is sent to the motors, the drivers on the Mini Cheetah actuators and the stepper motor send a packet back to the control box containing position, velocity, and current data. The structure of the command and data retrieval packets can be seen in my thesis or Ben's.

Though the Mini Cheetah actuators allow the PD gains to be tuned, sending incremental position commands to the robotic arm motors is a more precise method of getting close to the desired orientation. Furthermore, the speed at which these position commands are sent makes it easier to control the velocity of the arm. For general computer-controlled movements, a script was written to simply ramp the position of the robotic arm until it reached the target endpoint. More scripts were written such that these target orientations can be set up one of two ways--1) inputting the intermediary positions along a trajectory into a .CSV file to be read later, or 2) running the Python script that allows a user to manually move the arm to desired positions and then records such positions into a .CSV file to be read later. A process flow of motion planning can be seen in the figure below:

![alt text](/robotarmautocon.JPG?raw=true)

When manually controlling the arm using a computer keyboard, simultaneous, multiple-joint control was desired. One of the only Python libraries that featured concurrent recording of several key strokes was Pygame. Hence, a window-less Pygame browser was generated, and the ASDFQWER keys could be used to control the positive and negative motions of each of the four joints. However, using Pygame came with one caveat--when wirelessly connecting to the rover's Intel NUC over a secure shell (SSH), the Pygame browser does not show up on the host computer upon running the manual control script, preventing any key strokes from being detected. To resolve this, Xming, an X11 display server, had to be installed and run every time manual control was desired, as this software will open the Pygame browser on the host computer. The ROS node structure can be seen in the figure below:

![alt text](/robotarmmanualcontrol.JPG?raw=true)

The firmware on the custom stepper motor driver was written to take the same CANBus packet commands as the Mini Cheetah actuators. The stepper driver also returns the same motor position, velocity, and current data. However, there are a handle of differences in the code and added functionality. The stepper driver is technically already enabled once power is supplied, so the "enter motor mode" command that enables the Mini Cheetah actuators only tells the stepper driver when it is at boot-up. The "zero position sensor" command zeroes the encoder if the driver was just booted up. If the driver was not just booted up and the "zero position sensor" command is received, the current position as measured by the Texas Instruments DRV8825 stepper driver is overwritten by the position measured by the stepper motor encoder. This would be done in the case where the stepper motor skips steps due to over-torquing, which could occur if the arm collides into an immovable object or if the end effector tool is too heavy.

## Contributors

#### Primary Contributors
Ryan Poon from the [MIT Bioinstrumentation Lab](https://bioinstrumentation.mit.edu/) is the primary contributor of this project.

#### Other Contributors
Professor Ian Hunter served as the main advisor.

## Citations

Information on the MIT Mini Cheetah actuators and the CANBus commands that operate them can be found in Ben's thesis:

Benjamin Katz. A low cost modular actuator for dynamic robots. Master’s thesis, Massachusetts Institute of Technology, Department of Mechanical Engineering, 2018.





