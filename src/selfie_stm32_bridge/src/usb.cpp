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

bool USB_STM::read_from_STM()
{
    int read_state = read(fd, &read_buffer[0], 512) ;

    //cast read_buffer to frame
    read_frame = (UsbReadFrame_s *)read_buffer;

    if (read_state == USB_RECEIVE_SIZE && read_frame->startbyte == frame_startbyte
      && read_frame->code == frame_code && read_frame->length == USB_RECEIVE_SIZE - 4
      && read_frame->endByte == frame_endbyte)
    {
        return true; //correct data
    }
    else
    {
        return false; //no new data 
    }
}

void USB_STM::send_to_STM(uint32_t timestamp_ms, Sub_messages to_send)
{
    send_frame = (UsbSendFrame_s *)send_buffer;
    send_frame->startbyte = frame_startbyte;
    send_frame->code = frame_code;
    send_frame->length = USB_SEND_SIZE - 4;

    send_frame->timestamp_ms = timestamp_ms;
    send_frame->steering_angle = (int16_t)(to_send.ackerman.steering_angle * 10000);
    send_frame->steering_angle_velocity = (int16_t)(to_send.ackerman.steering_angle_velocity * 10000);
    send_frame->speed = (int16_t)(to_send.ackerman.speed * 1000);
    send_frame->acceleration = (int16_t)(to_send.ackerman.acceleration * 1000);
    send_frame->jerk = (int16_t)(to_send.ackerman.jerk * 1000);
    send_frame->left_indicator = (uint8_t)(to_send.indicator.left);
    send_frame->right_indicator = (uint8_t)(to_send.indicator.right);
    send_frame->endbyte = frame_endbyte;

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
    pub_data.button1_msg.data = read_frame->start_button1;
    pub_data.button2_msg.data = read_frame->start_button2;

    //reset states
    pub_data.reset_vision_msg.data = read_frame->reset_vision;
    pub_data.switch_state_msg.data = read_frame->switch_state;

    //unused
    // read_frame->timestamp
    // read_frame->yaw


}