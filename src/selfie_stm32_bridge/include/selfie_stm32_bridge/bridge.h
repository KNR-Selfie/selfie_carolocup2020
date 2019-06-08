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

class Pub_messages
{
public:
    sensor_msgs::Imu imu_msg;
    std_msgs::Float32 velo_msg;
    std_msgs::Float32 dist_msg;
    std_msgs::Bool button1_msg;
    std_msgs::Bool button2_msg;

    std_msgs::Bool reset_vision_msg;
    std_msgs::UInt8 switch_state_msg;

};

class Sub_messages
{
public:
    Ackermann_control ackerman;
    Indicator_control indicator;
    Sub_messages();
};


#endif // SELFIE_STM32_BRIDGE_STM32_BRIDGE_H
