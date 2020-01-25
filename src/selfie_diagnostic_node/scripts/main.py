#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan #/scan
from sensor_msgs.msg import Image #/image_rect
from std_msgs.msg import Bool,String #/left_turn_indicator, /right_turn_indicator
from enum import Enum

from Tester import Tester
from State import State
import os
import sys

def checkDevices(devices):
    msg = ""
    for device in devices:
        device.checkDevice()
        msg += device.name + "'s status: " + device.state_.key() + " "
    pub.publish(msg)

def getRosMsgType(type):
    if type == "LaserScan":
        return LaserScan
    elif type == "Image":
        return Image
    elif type == "String":
        return String
    elif type == "Bool":
        return Bool
    else:
        rospy.logfatal("Incorrect message type, please revise launch file")
        # TODO Kill process
        sys.exit(0)


if __name__ =="__main__":
    rospy.init_node("diagnose", anonymous=True)
    pub = rospy.Publisher('~selfie_diagnostics', String, queue_size=10) # warning, errors publishers
    print("Starting Diagnostics")
    # getting delay og end of programme
    delay = rospy.get_param("~delay",10)
    parameters = [] 
    try:
        for i in range(0,10000):
            device = {}
            device["name"] = rospy.get_param("~" + str(i) + "_sensor_name")
            device["directory"] = rospy.get_param("~" + str(i) + "_sensor_directory", None)
            device["topic"] = rospy.get_param("~" + str(i) + "_sensor_topic")
            device["type"] = getRosMsgType(rospy.get_param("~" + str(i) + "_sensor_datatype"))
            device["frequency"] = rospy.get_param("~" + str(i) + "_sensor_hz")
            print("{} {} {} {} {}".format(device["name"],device["directory"], device["topic"], device["type"], device["frequency"]))
            parameters.append(device)
    except:
        print("{} devices detected".format(len(parameters)))

    devices = []
    for device in parameters:
        devices.append(Tester(device["name"], device["topic"], device["directory"], device["type"], device["frequency"]))
    # devices.append(Tester("/scan","/dev/amouse", LaserScan))
    rate = rospy.Rate(1)
    start_time = rospy.get_time() # czas poczatkowy dzialania programu
    print("Start")
    while not rospy.is_shutdown():
        checkDevices(devices)        
        rate.sleep()
        pass



