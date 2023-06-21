#!/usr/bin/env python
import copy
import rospy
import time
from xela_2f_force_control.srv import Setpoint,SetpointResponse
from xela_2f_force_control.msg import SetPointTarget
from threading import Lock


class SetpointGenerator():

    def __init__(self):

        rospy.init_node('xela_2f_force_setpoint')
        self.pub = rospy.Publisher('~setpoint', SetPointTarget, queue_size=10)
        s = rospy.Service('~generate', Setpoint, self.user_callback)

        # Initialize Setpoint variables
        self.actual_value = 0.0
        self.new_value = 0.0
        self.old_value = 0.0
        self.setpoint_counter = 0

        # Initialize time variables
        actual_time = time.time()
        self.duration = 0
        self.actual_time = actual_time
        self.start_time = actual_time

        # Initialize the semaphore
        self.mutex = Lock()

        # Setup a rate object
        self.rate = rospy.Rate(100)


    def loop(self):
        # Check if the program should exit
        while not rospy.is_shutdown():

            with self.mutex:
                self.actual_time = time.time()
                if (self.actual_time - self.start_time) < self.duration:
                    setpoint = self.generate_setpoint()
                    self.pub.publish(setpoint)

            self.rate.sleep()


    def generate_setpoint(self):
        '''
        Returns a setpoint in the form of a xela_2f_force_control/SetPointTarget
        '''

        new_step = SetPointTarget()
        new_step.setpoint_header.seq = self.setpoint_counter
        new_step.setpoint_header.stamp = rospy.Time.now()
        new_step.setpoint_target = self.old_value + (10 * (self.new_value - self.old_value) / (pow(self.duration, 3))) * pow(self.actual_time - self.start_time, 3) \
                                   - (15 * (self.new_value - self.old_value) / (pow(self.duration, 4))) * pow(self.actual_time - self.start_time, 4) \
                                   + (6 * (self.new_value - self.old_value) / (pow(self.duration, 5))) * pow(self.actual_time - self.start_time, 5)
        self.setpoint_counter += 1

        return new_step


    def user_callback(self, data):

        with self.mutex:
            self.start_time = time.time()
            self.old_value = self.new_value
            self.new_value = data.setpoint
            self.duration = data.duration
            self.setpoint_counter = 0

        return SetpointResponse("Done")


def main():
    generator = SetpointGenerator()
    generator.loop()
