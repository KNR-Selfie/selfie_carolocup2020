#ifndef SELFIE_STM32_BRIDGE_USB_HPP
#define SELFIE_STM32_BRIDGE_USB_HPP
#pragma once

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
#include <selfie_stm32_bridge/bridge.h>

enum
{
  frame_startbyte = 0xFF,
  frame_code = 0x40,
  frame_endbyte = 0xFE,
}data_frame_control_bytes;

enum 
{
  cmd_data_enable = 0x01,
  cmd_data_disable = 0x02,
  cmd_recalibration = 0x10,
  cmd_idle = 0x20,
  cmd_ready = 0x30,
  cmd_bootloader = 0xDE,
  cmd_reset = 0xAD,

}usb_commands;

typedef struct
{
    uint8_t startbyte;
    uint8_t code;
    uint8_t length;

    uint32_t timecode;

    int32_t distance;
    int16_t velocity;
    int16_t w, x, y, z;
    uint16_t yaw;
    int16_t rates[3];
    int16_t acc[3];
    uint8_t start_button1;
    uint8_t start_button2;
    uint8_t reset_vision;
    uint8_t switch_state;

    uint8_t endByte;

} UsbReadFrame_s __attribute__((__packed__));

#define USB_RECEIVE_SIZE sizeof(UsbReadFrame_s)
typedef struct
{
    uint8_t startbyte;
    uint8_t code;
    uint8_t length;
    uint32_t timestamp_ms;
    int16_t steering_angle;
    int16_t steering_angle_velocity;
    int16_t speed;
    int16_t acceleration;
    int16_t jerk;
    uint8_t left_indicator;
    uint8_t right_indicator;
    uint8_t endbyte;

}UsbSendFrame_s __attribute__((__packed__));
#define USB_SEND_SIZE sizeof(UsbSendFrame_s)

class USB_STM
{
private:
    int fd; //file descriptor
    UsbReadFrame_s *read_frame;
    UsbSendFrame_s *send_frame;
    unsigned char read_buffer[512];
    unsigned char send_buffer[sizeof(UsbSendFrame_s)];

public:
  int init(int speed = B115200);
  bool read_from_STM();
  void send_to_STM(uint32_t timestamp_ms, Sub_messages to_send);
  void fill_publishers(Pub_messages& to_publish);
};

#endif // USB_HPP
