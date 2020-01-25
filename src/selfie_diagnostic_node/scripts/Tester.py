#!/usr/bin/env python
import rospy
import os
from sensor_msgs.msg import LaserScan #/scan
from sensor_msgs.msg import Image #/image_rect
from std_msgs.msg import Bool #/left_turn_indicator, /right_turn_indicator
from State import State

class Tester:

    def callback(self, data):
        if(self.last_stamp_ == None):
            self.last_stamp_ = data.header.stamp
        else:
            self.frequency_ = 1/(data.header.stamp.to_sec() - self.last_stamp_.to_sec()) # Hz
            self.last_stamp_ = data.header.stamp
        pass

    def checkDeviceAvailability(self, directory):
        print("Mouse: " + str(os.path.exists(directory)))
        return os.path.exists(directory)
    
    def checkDevice(self):
        

    def __init__(self, name, topic, directory, msg_type, desired_frequency):
        self.name = name
        self.sub_ = rospy.Subscriber(topic, msg_type, self.callback)
        if(directory != None):
            self.is_plugged_ = self.checkDeviceAvailability(directory)
        else:
            self.is_plugged_ = None
        self.desired_frequency_ = desired_frequency
        self.last_stamp_ = None
        self.frequency_ = None
        self.state_ = State.UNDEFINED
    
        def isValid(self):
            pass