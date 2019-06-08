#pragma once
#ifndef USB_HPP
#define USB_HPP

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <string>
#include <vector>
#include <stdint.h>
#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

struct data_container
{
  const uint8_t startbyte = 0xFF;
  const uint8_t code = 0x40;
  const uint8_t endbyte = 0xFE;
};

struct usb_command
{
  const uint8_t startbyte = 0xFF;
  const uint8_t data_enable = 0x01;
  const uint8_t data_disable = 0x02;
  const uint8_t recalibration = 0x10;
  const uint8_t idle = 0x20;
  const uint8_t ready = 0x30;
  const uint8_t bootloader = 0xDE;
  const uint8_t reset = 0xAD;
  const uint8_t endbyte = 0xFE;

};

class Ackermann_control
{
public:
  data_container commands;
  float steering_angle;
  float steering_angle_velocity;
  float speed;
  float acceleration;
  float jerk;
};

class Indicator_control
{
public:
  uint8_t left;
  uint8_t right;
};

class USB_STM
{
private:
  int fd; //file descriptor
public:
  Ackermann_control control;
  Indicator_control indicators;
  usb_command command;
  int init(int speed = B115200);
  void usb_read_buffer(int buf_size, uint32_t& timestamp, int32_t& distance, int16_t& velocity, int16_t& quaternion_x, int16_t& quaternion_y, int16_t& quaternion_z, int16_t& quaternion_w, uint16_t yaw, int16_t& ang_vel_x, int16_t& ang_vel_y, int16_t& ang_vel_z, int16_t& lin_acc_x, int16_t& lin_acc_y, int16_t& lin_acc_z, uint8_t& start_button1, uint8_t& start_button2, uint8_t& reset_vision, uint8_t& switch_state);
  void usb_send_buffer(uint32_t timestamp_ms, float steering_angle, float steering_angle_velocity, float speed, float acceleration, float jerk, uint8_t left_indicator, uint8_t right_indicator);
};

#endif // USB_HPP
