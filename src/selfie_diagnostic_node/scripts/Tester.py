#!/usr/bin/env python3
import rospy
import os
from sensor_msgs.msg import LaserScan, Image, Imu
from std_msgs.msg import Bool, String, Float32 #/left_turn_indicator, /right_turn_indicator
from State import State

class Tester:

  def __init__(self, name, topic, directory, msg_type, desired_frequency):
    self.name = name
    self.directory_ = directory
    self.msg_type_ = msg_type
    if self.msg_type_ == Imu:
      self.is_data_valid = None
    self.sub_ = rospy.Subscriber(topic, msg_type, self.callback)
    self.pub_ = rospy.Publisher("~" + self.name, String, queue_size=10)
    self.desired_frequency_ = desired_frequency
    self.last_stamp_ = None
    self.frequency_ = None
    self.state_ = State.UNDEFINED
      
  def callback(self, data):
    try:
      if(self.last_stamp_ == None):
        self.last_stamp_ = data.header.stamp
      else:
        self.frequency_ = 1/(data.header.stamp.to_sec() - self.last_stamp_.to_sec()) # Hz
        self.last_stamp_ = data.header.stamp
    except:
      if(self.last_stamp_ == None):
        self.last_stamp_ = rospy.get_rostime()
      else:
        now = rospy.get_rostime()
        self.frequency_ = 1/(now.to_sec() - self.last_stamp_.to_sec()) # Hz
        self.last_stamp_ = now
    if self.msg_type_ == Imu:
      if data.orientation.x != 0.0 and data.orientation.y != 0.0 and data.orientation.z != 0.0:
        self.is_data_valid = True
      else:
        self.is_data_valid = False


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
          msg += '[Diagnostic Node] ' +  self.name + " is not plugged! "
          self.pub_.publish(msg)
          self.state_ = State.FATAL
          rospy.logerr(msg)
          return
      #check if Imu data exists
      if self.msg_type_ == Imu:
        if self.is_data_valid != True:
          msg += '[Diagnostic Node] ' +  self.name + " invalid data! "
          self.state = State.ERROR
          rospy.logerr(msg)
          return
      # node stopped publishing
      if self.last_stamp_ != None :
        if rospy.get_time() - self.last_stamp_.to_sec() > 3 :
          self.frequency_ = 0.0
          self.state = State.ERROR
      # checking msg rates
      if self.frequency_ == None:
        msg += '[Diagnostic Node] ' +  self.name + " hasn't published messages yet! "
        self.state_ = State.FATAL
      elif self.frequency_ == 0.0:
        msg += '[Diagnostic Node] ' +  self.name + " stopped publishing "
        self.state_ = State.ERROR
      elif self.frequency_ < self.desired_frequency_ - 0.5 or self.frequency_ > self.desired_frequency_ + 0.5:
        msg = '[Diagnostic Node] ' +  self.name + " rate: " + str(self.frequency_) + " " + msg
        self.state_ = State.WARNING
      else:
        msg = '[Diagnostic Node] ' +  self.name + " rate: " + str(self.frequency_) + " " + msg
      rospy.loginfo(msg)
      self.pub_.publish(msg)
    