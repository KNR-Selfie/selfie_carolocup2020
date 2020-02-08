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
  headligths_on = 0x50,
  headligths_off = 0x51,
  headligths_set_low_beam = 0x52,
  headligths_set_high_beam = 0x53,


  left_indicator_on = 0x58,
  left_indicator_off = 0x59,
  rigth_indicator_on = 0x5A,
  rigth_indicator_off = 0x5B,
  reset_buttons = 0x60,

}usb_commands;

typedef struct __attribute__((__packed__))
{
    uint8_t start_code;
    uint8_t length;

    uint32_t timecode;

    int32_t distance;
    int16_t velocity;
    int16_t w, x, y, z;
    uint16_t yaw;
    int16_t rates[3];
    int16_t acc[3];
    uint8_t buttons;
    uint8_t lights;
    uint8_t futaba_state;
    uint16_t crc16;
    uint8_t end_code;

} UsbReadFrame_s;

#define USB_RECEIVE_SIZE sizeof(UsbReadFrame_s)
typedef struct __attribute__((__packed__))
{
    uint8_t start_code;
    uint8_t length;
    uint32_t timecode;
    int16_t steering_fi_front;
    int16_t steering_fi_back;
    int16_t speed;
    int16_t acceleration;
    int16_t jerk;
    uint16_t crc16;
    uint8_t end_code;

}UsbSendFrame_s;
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
  void send_frame_to_STM(uint32_t timestamp_ms, Sub_messages to_send);
  void fill_publishers(Pub_messages& to_publish);
  void send_cmd_to_STM(uint8_t cmd);
  ~USB_STM();
};

#endif // USB_HPP
