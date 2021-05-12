#!/usr/bin/env python3

import matplotlib.pyplot as plt
import time
import rospy

HOST = "localhost"
PORT = 4223
UID_Master = "6SwGJu"
UID_DC = "6xi9bm"
UID_ssr = "F1t"
UID_idr = "KqZ"

from std_msgs.msg import String
from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_master import BrickMaster
from tinkerforge.brick_dc import BrickDC
from tinkerforge.bricklet_industrial_dual_relay import BrickletIndustrialDualRelay
from tinkerforge.bricklet_solid_state_relay_v2 import BrickletSolidStateRelayV2

# Global variables needed to operate tool changer
DC_MAX_VELOCITY = 32767
CURRENT_THRESHOLD = 75  # mA
UNSCREW_TIME = 18  # seconds
WANT_CURRENT_DATA = True
CHECK_TOOL = 0
TIME_GLOBAL = []
CURRENT_GLOBAL = []

# For controlling tool changer

class Motor:
    def __init__(self, torque_gradient, torque_constant, speed_constant, gear_ratio, gear_eff, max_velocity):
        # Initialize all class object variables (motor properties)
        self.torque_gradient = torque_gradient  # rpm/mNm
        self.torque_constant = torque_constant  # mNm/A
        self.speed_constant = speed_constant  # rpm/V
        self.gear_ratio = gear_ratio
        self.gear_eff = gear_eff
        self.max_velocity = max_velocity

        # For data collection
        self.want_data = True
        self.tout = []
        self.current = []

    def get_velocity_from_torque(self, torque_des):
        # torque_des in mNm
        return (-self.torque_gradient * torque_des / (self.gear_ratio * self.gear_eff)) + self.max_velocity

    def get_duty_cycle(self, vel_des):
        # Returns the needed duty cycle to generate the desired velocity
        return vel_des / self.max_velocity

    def screw(self, desired_torque, want_data):
        # This function attaches a new tool based on the desired torque
        # Set duty cycle according to desired velocity
        duty_cycle = self.get_duty_cycle(self.get_velocity_from_torque(desired_torque))
        print("Duty cycle: ", duty_cycle * 100, "%")

        # Configure the velocity settings on the Tinkerforge bricklets based on the duty cycle
        dc.set_velocity(int(duty_cycle * DC_MAX_VELOCITY))
        dc.set_acceleration(int(duty_cycle * DC_MAX_VELOCITY))
        dc.enable()

        # Spin the threaded stud until the shank is completely seated within the socket, at which the current will
        # spike and cause the stud to stop spinning.
        start_time = rospy.get_time()
        while True:
            mA = dc.get_current_consumption()
            if mA > CURRENT_THRESHOLD:
                dc.full_brake()
                print("Current threshold reached--tool screwed in.")
                break

            # If data is desired, append current and time data to a list for future plotting
            if want_data:
                self.current.append(mA)
                self.tout.append(rospy.get_time() - start_time)

        time.sleep(0.1)
        dc.disable()

        # if want_data:
        #     self.plot_current()

    def unscrew(self, desired_torque):
        # This function detaches the tool
        # Set duty cycle according to desired velocity
        duty_cycle = self.get_duty_cycle(self.get_velocity_from_torque(desired_torque))
        print("Duty cycle: ", duty_cycle * 100, "%")

        # Configure the velocity settings on the Tinkerforge bricklets based on the duty cycle
        dc.set_velocity(-int(duty_cycle * DC_MAX_VELOCITY))
        dc.set_acceleration(int(duty_cycle * DC_MAX_VELOCITY))
        dc.enable()

        # Spins the threaded stud for a set amount of time so the shank coupling nut detaches.
        time_trigger = False
        start_time = time.clock()
        while (time.clock() - start_time) < UNSCREW_TIME:
            if (time.clock() - start_time) > 2 and not time_trigger:
                dc.set_velocity(-DC_MAX_VELOCITY)
                time_trigger = True
            pass

        dc.full_brake()
        time.sleep(0.1)
        dc.disable()


    def get_time(self):
        return self.tout


    def get_current(self):
        return self.current


def callback(data):
    # This function waits for a command from a motor controller node, manual or otherwise.
    global CHECK_TOOL, TIME_GLOBAL, CURRENT_GLOBAL
    motor = Motor(2750, 16.3, 587, 370, 0.65, 11300)

    # Depending on the received command, the torque is set and the command is executed using the proper class function
    if data.data == 'screw' and CHECK_TOOL == 0:
        desired_torque = 100  # mNm
        motor.screw(desired_torque, WANT_CURRENT_DATA)
        CHECK_TOOL = 1
    elif data.data == 'unscrew':
        desired_torque = 350
        motor.unscrew(desired_torque)
        CHECK_TOOL = 0
    else:
        rospy.logerr("Tool changer does not recognize command!")

    # Time list is expanded with the collected time and current data.
    if WANT_CURRENT_DATA and data.data == 'screw':
        TIME_GLOBAL.extend(motor.get_time())
        CURRENT_GLOBAL.extend(motor.get_current())


def turn_off():
    # uncomment if only running tool changer
    # Turns off all of the relays if they are still on. Disconnects from Tinkerforge stack
    if idr.get_value().channel0 == True:
       idr.set_value(False, False)
    if ssr.get_state() == True:
       ssr.set_state(False)
    rospy.loginfo("Tool Changer turning off.")
    ipcon.disconnect()

    # Plots current and time data if that data was desired.
    plt.rc('font', size=12)
    plt.figure(1)
    plt.plot(TIME_GLOBAL, CURRENT_GLOBAL)
    plt.xlabel("Time (s)")
    plt.ylabel("Current (mA)")
    plt.title("Brushed Motor Current (Tool Attach)")
    plt.grid(True, which='both')
    plt.subplots_adjust(left=0.15, bottom=0.15, right=0.71)
    plt.show()


def listener():
    # Initializes the ROS node and subscriber
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("tool_changer", String, callback)
    rospy.on_shutdown(turn_off)
    rospy.loginfo("Tool Changer activated, waiting for command...")
    rospy.spin()


if __name__ == "__main__":
    # Connects to Tinkerforge bricks and bricklets
    ipcon = IPConnection()  # create ip connection
    master = BrickMaster(UID_Master, ipcon)
    dc = BrickDC(UID_DC, ipcon)
    ssr = BrickletSolidStateRelayV2(UID_ssr, ipcon)
    idr = BrickletIndustrialDualRelay(UID_idr, ipcon)
    ipcon.connect(HOST, PORT)

    # Opens all relays in the control box to turn the tool changer on.
    if idr.get_value().channel0 == False:
        idr.set_value(True, False)
    if ssr.get_state() == False:
        ssr.set_state(True)

    listener()




