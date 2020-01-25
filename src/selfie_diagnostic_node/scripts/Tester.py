#!/usr/bin/env python
import rospy
import os
from sensor_msgs.msg import LaserScan #/scan
from sensor_msgs.msg import Image #/image_rect
from std_msgs.msg import Bool, String #/left_turn_indicator, /right_turn_indicator
from State import State

class Tester:

    def callback(self, data):
        if(self.last_stamp_ == None):
            self.last_stamp_ = data.header.stamp
        else:
            self.frequency_ = 1/(data.header.stamp.to_sec() - self.last_stamp_.to_sec()) # Hz
            self.last_stamp_ = data.header.stamp
        pass

    def checkDeviceAvailability(self):
        if self.directory_ == None:
            self.is_plugged_ = None
        else:
            self.is_plugged_ = os.path.exists(self.directory_)
    
    def checkDevice(self):
        msg = ""
        # checking if device is plugged
        self.state_ = State.OK
        self.checkDeviceAvailability()
        # is plugged == None -> we dont want to check if device is plugged 
        if self.is_plugged_ != None:
            if self.is_plugged_ == False:
                msg += self.name + " is not plugged! "
                self.pub_.publish(msg)
                self.state_ = State.FATAL
                return
        # TODO uncomment on car
        # if self.last_stamp_ != None :
        #     if rospy.get_time() - self.last_stamp_.to_sec() > 3 :
        #         self.frequency_ = 0.0
        #         self.state = State.ERROR
        #         print(self.name + " stopped publishing ")
                # msg += "Lidar stopped publishing! "
        # checking msg rates
        if self.frequency_ == None:
            msg += self.name + " hasn't published messages yet! "
            self.state_ = State.FATAL
        elif self.frequency_ == 0.0:
            msg += self.name + " stopped publishing "
            self.state_ = State.ERROR
        elif self.frequency_ < self.desired_frequency_ - 0.1 or self.frequency_ > self.desired_frequency_ + 0.1:
            msg += self.name + " publishes with wrong frequency "
            msg = self.name + " rate: " + str(self.frequency_) + " " + msg
            self.state_ = State.WARNING
        else:
            msg = self.name + " rate: " + str(self.frequency_) + " " + msg
        self.pub_.publish(msg)

    def __init__(self, name, topic, directory, msg_type, desired_frequency):
        self.name = name
        self.directory_ = directory
        self.sub_ = rospy.Subscriber(topic, msg_type, self.callback)
        self.pub_ = rospy.Publisher("~" + self.name, String, queue_size=10)
        self.desired_frequency_ = desired_frequency
        self.last_stamp_ = None
        self.frequency_ = None
        self.state_ = State.UNDEFINED
    