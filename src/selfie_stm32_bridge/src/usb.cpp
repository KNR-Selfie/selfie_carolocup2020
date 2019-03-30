#include "usb.hpp"

#define USB_SEND_SIZE 18+2
#define USB_RECEIVE_SIZE 36+4

int USB_STM::init(int speed)
{
  char port[] = "/dev/serial/by-id/usb-KNR_Selfie_F7_00000000001A-if00";
  //char port[] = "/dev/serial/by-id/STM32F407";
  fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0){
    ROS_ERROR("Could not open serial communication on port!.\n Make sure you did: chmod u +rw /dev/serial/by-id/usb-KNR_Selfie_F7_00000000001A-if00\n");
    return -1;
  }
  else
  {
    ROS_INFO("Opened serial communication on port\n");
  }

  // Get attributes of transmission
  struct termios tty;
  if (tcgetattr(fd, &tty) < 0)
  {
    ROS_ERROR("Error while getting attributes!");
    return -2;
  }

  // Set input and output speed
  cfsetospeed(&tty, B921600);
  cfsetispeed(&tty, B921600);

  tty.c_cflag |= (CLOCAL | CREAD);    // program will not become owner of port
  tty.c_cflag &= ~CSIZE;              // bit mask for data bits
  tty.c_cflag |= CS8;                 // 8 bit data lenght
  tty.c_cflag |= PARENB;              // enable parity
  tty.c_cflag &= ~PARODD;             // even parity
  tty.c_cflag &= ~CSTOPB;             // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;            // no hardware flowcontrol

  // non-canonical mode
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  // fetch bytes asap
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;

  // Set new parameters of transmission
  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    ROS_ERROR("Error while setting attributes!");
    return -3;
  }

  unsigned char data_enable[3] = {command.startbyte, command.data_enable, command.endbyte};
  write(fd, data_enable, 3);

  return 1;
}

void USB_STM::usb_read_buffer(int buf_size, uint32_t& timestamp, int32_t& distance, int16_t& velocity, int16_t& quaternion_x, int16_t& quaternion_y, int16_t& quaternion_z, int16_t& quaternion_w, uint16_t yaw, int16_t& ang_vel_x, int16_t& ang_vel_y, int16_t& ang_vel_z, int16_t& lin_acc_x, int16_t& lin_acc_y, int16_t& lin_acc_z, uint8_t& start_button1, uint8_t& start_button2, uint8_t& reset_vision, uint8_t& switch_state)
{

  struct UsbFrame_s
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
  } __attribute__((__packed__));
  union UsbFrame_u
  {
    unsigned char buffer[512];
    struct UsbFrame_s frame;
  } Data;

  int read_state = read(fd, &Data.buffer[0], 512) ;

  if (read_state == USB_RECEIVE_SIZE && Data.frame.startbyte == control.commands.startbyte
      && Data.frame.code == control.commands.code && Data.frame.length == USB_RECEIVE_SIZE - 4
      && Data.frame.endByte == control.commands.endbyte)

  {
    //timestamp
    timestamp = Data.frame.timecode;

    //distance from encoders
    distance = Data.frame.distance;

    //car velocity
    velocity = Data.frame.velocity;

    //imu data
    quaternion_x = Data.frame.x;
    quaternion_y = Data.frame.y;
    quaternion_z = Data.frame.z;
    quaternion_w = Data.frame.w;
    yaw = Data.frame.yaw;
    ang_vel_x = Data.frame.rates[0];
    ang_vel_y = Data.frame.rates[1];
    ang_vel_z = Data.frame.rates[2];
    lin_acc_x = Data.frame.acc[0];
    lin_acc_y = Data.frame.acc[1];
    lin_acc_z = Data.frame.acc[2];

    //start_button
    start_button1 = Data.frame.start_button1;
    start_button2 = Data.frame.start_button2;

    //stm32 status
    reset_vision = Data.frame.reset_vision;
    switch_state = Data.frame.switch_state;
  }
}

void USB_STM::usb_send_buffer(uint32_t timestamp_ms, float steering_angle, float steering_angle_velocity, float speed, float acceleration, float jerk, uint8_t left_indicator, uint8_t right_indicator)
{
  struct UsbFrame_s
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
  } __attribute__((__packed__));

  union UsbFrame_u
  {
    unsigned char bytes[USB_SEND_SIZE];
    struct UsbFrame_s frame;
  } Data;

  Data.frame.startbyte = control.commands.startbyte;
  Data.frame.code = control.commands.code;
  Data.frame.length = USB_SEND_SIZE - 4;
  Data.frame.timestamp_ms = timestamp_ms;
  Data.frame.steering_angle = (int16_t)(steering_angle * 10000);
  Data.frame.steering_angle_velocity = (int16_t)(steering_angle_velocity * 10000);
  Data.frame.speed = (int16_t)(speed * 1000);
  Data.frame.acceleration = (int16_t)(acceleration * 1000);
  Data.frame.jerk = (int16_t)(jerk * 1000);
  Data.frame.left_indicator = (uint8_t)(left_indicator);
  Data.frame.right_indicator = (uint8_t)(right_indicator);
  Data.frame.endbyte = control.commands.endbyte;
  write(fd, &Data.bytes, USB_SEND_SIZE);
}
