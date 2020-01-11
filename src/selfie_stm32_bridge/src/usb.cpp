#include <selfie_stm32_bridge/usb.hpp>

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

  //send ACK command
  unsigned char data_enable[3] = {frame_startbyte, cmd_data_enable, frame_endbyte};
  write(fd, data_enable, 3);

  return 1;
}
void USB_STM::send_cmd_to_STM(uint8_t cmd)
{ 
  unsigned char cmd_data[3] = {frame_startbyte, cmd, frame_endbyte};
  write(fd, cmd_data, 3);
}

bool USB_STM::read_from_STM()
{
    int read_state = read(fd, &read_buffer[0], 512) ;
    
    //cast read_buffer to frame
    read_frame = (UsbReadFrame_s *)read_buffer;
    
    if (read_state == USB_RECEIVE_SIZE && read_frame->start_code == frame_startbyte && read_frame->length == USB_RECEIVE_SIZE
      && read_frame->end_code == frame_endbyte)
    {   
        return true; //correct data
    }
    else
    {
        return false; //no new data 
    }
}

void USB_STM::send_frame_to_STM(uint32_t timestamp_ms, Sub_messages to_send)
{
    send_frame = (UsbSendFrame_s *)send_buffer;
    send_frame->start_code = frame_startbyte;
    send_frame->length = USB_SEND_SIZE;

    send_frame->timecode = timestamp_ms;
    send_frame->steering_fi_front = (int16_t)(to_send.ackerman.steering_angle_front * 10000);
    send_frame->steering_fi_back = (int16_t)(to_send.ackerman.steering_angle_back * 10000);
    send_frame->speed = (int16_t)(to_send.ackerman.speed * 1000);
    send_frame->acceleration = (int16_t)(to_send.ackerman.acceleration * 1000);
    send_frame->jerk = (int16_t)(to_send.ackerman.jerk * 1000);
    send_frame->end_code = frame_endbyte;

    write(fd, &send_buffer, USB_SEND_SIZE);
}

void USB_STM::fill_publishers(Pub_messages &pub_data)
{
    pub_data.imu_msg.header.frame_id = "imu";
    pub_data.imu_msg.header.stamp = ros::Time::now();

    //calculate imu quaternions
    pub_data.imu_msg.orientation.x = (float)(read_frame->x / 32767.f);
    pub_data.imu_msg.orientation.y = (float)(read_frame->y / 32767.f);
    pub_data.imu_msg.orientation.z = (float)(read_frame->z / 32767.f);
    pub_data.imu_msg.orientation.w = (float)(read_frame->w / 32767.f);

    pub_data.imu_msg.linear_acceleration.x = (float)(read_frame->acc[0] / 8192.f * 9.80655f);
    pub_data.imu_msg.linear_acceleration.y = (float)(read_frame->acc[1] / 8192.f * 9.80655f);
    pub_data.imu_msg.linear_acceleration.z = (float)(read_frame->acc[2] / 8192.f * 9.80655f);

    pub_data.imu_msg.angular_velocity.x = (float)(read_frame->rates[0] / 65.535f);
    pub_data.imu_msg.angular_velocity.y = (float)(read_frame->rates[1] / 65.535f);
    pub_data.imu_msg.angular_velocity.z = (float)(read_frame->rates[2] / 65.535f);

    //acctual car velocity
    pub_data.velo_msg.data = (float)read_frame->velocity / 1000;

    //distance from encoders
    pub_data.dist_msg.data = (float)read_frame->distance / 1000;

    //buttons
    uint8_t button1_state = (read_frame->buttons >> 0) & 1U;
    uint8_t button2_state = (read_frame->buttons >> 2) & 1U;

    if(button1_state)
      pub_data.button_1 = true;
    else 
      pub_data.button_1 = false;
    if(button2_state)
      pub_data.button_2 = true;
    else
      pub_data.button_2 = false;

    //reset states
    pub_data.futaba_state.data = read_frame->futaba_state;

    //unused
    // read_frame->timestamp
    // read_frame->yaw
}
USB_STM::~USB_STM()
{
    send_frame = (UsbSendFrame_s *)send_buffer;
    send_frame->start_code = frame_startbyte;
    send_frame->length = USB_SEND_SIZE;

    send_frame->timecode = 0;
    send_frame->steering_fi_back = 0;
    send_frame->steering_fi_front = 0;
    send_frame->speed = 0;
    send_frame->acceleration = 0;
    send_frame->jerk = 0;
    send_frame->end_code = frame_endbyte;

    write(fd, &send_buffer, USB_SEND_SIZE);
}
