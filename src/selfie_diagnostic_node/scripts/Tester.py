#!/usr/bin/env python
import rospy
import os
from sensor_msgs.msg import LaserScan #/scan
from sensor_msgs.msg import Image #/image_rect
from std_msgs.msg import Bool #/left_turn_indicator, /right_turn_indicator
import State

class Tester:
    def __init__(self, topic, directory, msg_type):
        self.sub_ = rospy.Subscriber(topic, msg_type, self.callback)
        self.is_plugged_ = self.checkDeviceAvailability(directory)
        self.last_stamp_ = None
        self.frequency_ = None
    
        def callback(self, data):
        if(self.last_stamp_ == None):
            self.last_stamp_ = data.header.stamp
        else:
            self.frequency_ = 1/(data.header.stamp.to_sec() - self.last_stamp_.to_sec()) # Hz
            self.last_stamp_ = data.header.stamp
        pass
        
        def checkDeviceAvailability(self, directory):
            return os.access(directory, os.F_OK)


    