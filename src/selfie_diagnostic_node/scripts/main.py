#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan #/scan
from sensor_msgs.msg import Image #/image_rect
from std_msgs.msg import Bool,String #/left_turn_indicator, /right_turn_indicator
from enum import Enum

from Tester import Tester
from State import State
import os

def checkDevices():
    pass


if __name__ =="__main__":
    rospy.init_node("diagnose", anonymous=True)
    pub = rospy.Publisher('selfie_diagnostics', String, queue_size=10) # warning, errors publishers
    print("Starting Diagnostics")
    start_time = rospy.get_time() # czas poczatkowy dzialania programu
    rate = rospy.Rate(1)
    devices = []
    devices.append(Tester("/scan","/dev/amouse", LaserScan))
    while not rospy.is_shutdown():
        msg = ""
        for device in devices:
            # checking if device is plugged
            if device.is_plugged_ != None:
                if device.is_plugged_ == False:
                    msg += "Lidar is not plugged! "
                    device.state = State.FATAL
            # checking msg rates
            if device.frequency_ == None:
                msg += "Lidar hasn't published messages from the beginning! "
            else:
                msg = "Lidar rate: " + str(device.frequency_) + " " + msg
            # check topic frequency
            # check last message time
            if device.last_stamp_ != None :
                if rospy.get_time() - device.last_stamp_.to_sec() > 3 :
                    device.frequency_ = 0.0
                    print("Lidar stopped publishing ")
                    # msg += "Lidar stopped publishing! "
            # check if plugged
        
        
        msg = "Lidar status:" + device.state_.key() + " " + msg
        pub.publish(msg)
        rate.sleep()
        pass



