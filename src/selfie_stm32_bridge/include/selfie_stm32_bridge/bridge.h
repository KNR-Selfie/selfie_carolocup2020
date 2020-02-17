#ifndef SELFIE_STM32_BRIDGE_STM32_BRIDGE_H
#define SELFIE_STM32_BRIDGE_STM32_BRIDGE_H
#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"



class Time
{
    const ros::Time begin;
public:
    uint32_t get_ms_time();
    Time();
};

class Ackermann_control
{
public:
  float steering_angle_front;
  float steering_angle_back;
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

class Pub_messages
{
public:
    sensor_msgs::Imu imu_msg;
    std_msgs::Float32 velo_msg;
    std_msgs::Float32 dist_msg;
    std_msgs::UInt8 futaba_state;
    bool button_1;
    bool button_2;
};

class Sub_messages
{
public:
    Ackermann_control ackerman;
    Sub_messages();
};


#endif // SELFIE_STM32_BRIDGE_STM32_BRIDGE_H
