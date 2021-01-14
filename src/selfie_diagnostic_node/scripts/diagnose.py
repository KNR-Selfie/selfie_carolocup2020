#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, Image, Imu
from std_msgs.msg import Bool, String, Float32
from enum import Enum

from Tester import Tester
from State import State
import os
import sys

class Diagnose:
  def __init__(self):
    # ros communication
    self.pub_ = rospy.Publisher('~selfie_diagnostics', String, queue_size=10) # warning, errors publishers
    self.left_blink_ = rospy.Publisher("left_turn_indicator", Bool, queue_size=10)
    self.right_blink_ = rospy.Publisher("right_turn_indicator", Bool, queue_size=10)
    rospy.on_shutdown(self.shutdownPerformance)
    rospy.loginfo("[Diagnostic Node] Starting  Selfie Diagnostics")

  def setupDevices(self):
    # reading devices from parameters server
    parameters = self.getParameters()
    # creating testers for every device
    self.devices_ = [Tester(device["name"], device["topic"], device["directory"],\
                      device["type"], device["frequency"]) for device in parameters]

  #  checking current state of every device registered
  def checkDevicesStates(self):
    status = State.OK
    for device in self.devices_:
      if device.state_ == State.FATAL:
        return State.FATAL
      elif device.state_ == State.WARNING:
        status = State.WARNING
    return status

  # performing check on all devices
  def checkDevices(self):
    msg = ""
    for device in self.devices_:
      device.checkDevice()
      msg += device.name + "'s status: " + device.state_.key() + " "
    self.pub_.publish(msg)

  # method converting string to ros msg type
  def getRosMsgType(self,type):
    if type == "LaserScan":
      return LaserScan
    elif type == "Image":
      return Image
    elif type == "String":
      return String
    elif type == "Float32":
      return Float32
    elif type == "Bool":
      return Bool
    elif type == "Imu":
      return Imu
    else:
      rospy.logfatal("[Diagnostic Node] Incorrect message type, please revise launch file")
      rospy.signal_shutdown("[Diagnostic Node] Incorrect message type, please revise launch file")
      sys.exit(1)

  # method blinking selfie lights for defined time
  def blinkLights(self,time):
    now = rospy.Time.now()
    while (now + rospy.Duration(time) > rospy.Time.now()):
      self.left_blink_.publish(Bool(data=True))
      self.right_blink_.publish(Bool(data=True))
    # rospy.sleep(time)
    now = rospy.Time.now()
    while (now + rospy.Duration(0.5) > rospy.Time.now()):
      self.left_blink_.publish(Bool(data=False))
      self.right_blink_.publish(Bool(data=False))

  # turning lights off when node interrupted
  def shutdownPerformance(self):
    self.left_blink_.publish(Bool(data=False))
    self.right_blink_.publish(Bool(data=False))
    print("[Diagnostic Node] DIAGNOSER SHUTDOWN")

  # reading parameters from server
  def getParameters(self):
    parameters = [] 
    try:
      for i in range(0,10000):
        device = {}
        device["name"] = rospy.get_param("~" + str(i) + "_sensor_name")
        device["directory"] = rospy.get_param("~" + str(i) + "_sensor_directory", None)
        device["topic"] = rospy.get_param("~" + str(i) + "_sensor_topic")
        device["type"] = self.getRosMsgType(rospy.get_param("~" + str(i) + "_sensor_datatype"))
        device["frequency"] = rospy.get_param("~" + str(i) + "_sensor_hz")
        rospy.loginfo("[Diagnostic Node] %s detected", device["name"])
        parameters.append(device)
    except:
      rospy.loginfo("[Diagnostic Node] SUMMARY: %d devices detected",len(parameters))
    return parameters

  def diagnose(self, debug):
    # checking overall status
    devices_status = self.checkDevicesStates()
    # defining diagnose
    if devices_status == State.FATAL or devices_status == State.ERROR:
      rospy.logfatal("[Diagnostic Node] FATAL ERROR")
    elif devices_status == State.WARNING:
      rospy.loginfo("[Diagnostic Node] WARNING")
      if not debug:
        diagnoser.blinkLights(6.0)
        # unregistering subscribers
        for device in self.devices_:
          device.sub_.unregister()
          rospy.signal_shutdown("[Diagnostic Node] Done")
          sys.exit(0)  
    elif devices_status == State.OK:
      rospy.loginfo("[Diagnostic Node] OK")
      if not debug:
        diagnoser.blinkLights(2.0)
        # unregistering subscribers
        for device in self.devices_:
          device.sub_.unregister()
          rospy.signal_shutdown("[Diagnostic Node] Done")
          sys.exit(0) 
    else:
      rospy.loginfo("[Diagnostic Node] WTF")


if __name__ =="__main__":
  # init
  rospy.init_node("diagnose", anonymous=True)
  diagnoser = Diagnose()
  diagnoser.setupDevices()
  # defining rate of performing diagnostics
  frequency = rospy.get_param("~frequency",1)
  rate = rospy.Rate(frequency)
  # getting delay when to stop programme
  delay = rospy.get_param("~delay",10)
  # debug value true -> programme spins to infinity
  # debug value false -> delay is taken into consideration during computation
  debug = rospy.get_param("~debug",True)
  # reading start time of node, needed for
  start_time = rospy.get_time()
  while not rospy.is_shutdown():
    # updating devices internal states
    diagnoser.checkDevices()
    # checking if diagnostics ended
    if rospy.get_time() - start_time > delay:
      diagnoser.diagnose(debug)          
    rate.sleep()
