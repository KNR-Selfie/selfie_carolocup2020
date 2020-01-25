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

#  performing current state of every device registered
def checkDevicesStates(devices):
  status = State.OK
  for device in devices:
    if device.state_ == State.FATAL:
      return State.FATAL
    elif device.state_ == State.WARNING:
      status = State.WARNING
  return status

# performing check on all device
def checkDevices(devices):
  msg = ""
  for device in devices:
    device.checkDevice()
    msg += device.name + "'s status: " + device.state_.key() + " "
  pub.publish(msg)

# method converting string ros msg type to actual type
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
    rospy.signal_shutdown("Incorrect message type, please revise launch file")
    sys.exit(1)

# method blinking selfie lights for defined time
def blinkLights(time):
  left_blink_.publish(Bool(True))
  right_blink_.publish(Bool(True))
  rospy.sleep(time)
  left_blink_.publish(Bool(False))
  right_blink_.publish(Bool(False))

def shutdownPerformance():
  # self.left_blink_.publish(Bool(False))
  # self.right_blink_.publish(Bool(False))
  print("SHUTDOWN")

def getParameters():
  parameters = [] 
  try:
    for i in range(0,10000):
      device = {}
      device["name"] = rospy.get_param("~" + str(i) + "_sensor_name")
      device["directory"] = rospy.get_param("~" + str(i) + "_sensor_directory", None)
      device["topic"] = rospy.get_param("~" + str(i) + "_sensor_topic")
      device["type"] = getRosMsgType(rospy.get_param("~" + str(i) + "_sensor_datatype"))
      device["frequency"] = rospy.get_param("~" + str(i) + "_sensor_hz")
      # print("{} {} {} {} {}".format(device["name"],device["directory"], \
      #                               device["topic"], device["type"], device["frequency"]))
      rospy.loginfo("%s detected", device["name"])
      parameters.append(device)
  except:
    rospy.loginfo("SUMMARY: %d devices detected",len(parameters))
  return parameters


if __name__ =="__main__":
  # init and shutdown
  rospy.init_node("diagnose", anonymous=True)
  rospy.on_shutdown(shutdownPerformance)
  # ros communication
  pub = rospy.Publisher('~selfie_diagnostics', String, queue_size=10) # warning, errors publishers
  left_blink_ = rospy.Publisher("/left_turn_indicator", Bool, queue_size=10)
  right_blink_ = rospy.Publisher("/right_turn_indicator", Bool, queue_size=10)
  rospy.loginfo("Starting  Selfie Diagnostics")
  # getting delay when to stop programme
  delay = rospy.get_param("~delay",10)
  # debug value true -> programme spins to infinity
  # debug value false -> delay is taken into consideration during computation
  debug = rospy.get_param("~debug",True)
  # reading devices from parameters server
  parameters = getParameters()
  # creating testers for every device
  devices = [Tester(device["name"], device["topic"], device["directory"],\
                    device["type"], device["frequency"]) for device in parameters]
  # defining rate of performing diagnostics
  rate = rospy.Rate(1)
  # reading start time of node, needed for
  start_time = rospy.get_time() 
  print("Start")
  while not rospy.is_shutdown():
    checkDevices(devices)
    # checking if diagnostics ended
    if rospy.get_time() - start_time > delay and not debug:
      print("OVERALL STATUS")
      # showing that diagnostics time is up
      blinkLights(1.0)
      rospy.sleep(0.5)
      # checking overall status
      devices_status = checkDevicesStates(devices)
      print(devices_status)
      now = rospy.get_time()
      diff = rospy.get_time() - start_time
      print("now: {}, start: {}, diff: {}".format(now, start_time, diff))
      # defining diagnose
      if devices_status == State.FATAL or devices_status == State.ERROR:
        rospy.logfatal("DIAGNOSE: FATAL ERROR")
      elif devices_status == State.WARNING:
        rospy.loginfo("DIAGNOSE: WARNING")
        blinkLights(6.0)
      elif devices_status == State.OK:
        rospy.loginfo("DIAGNOSE: OK")
        blinkLights(2.0)
      else:
        rospy.loginfo("DIAGNOSE: WTF")
      rospy.signal_shutdown("Diagnostics Done")
      sys.exit(0)         
    rate.sleep()
