#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "usb.hpp"
#include <sstream>

USB_STM Usb;

void ackermanCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);
void left_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg);
void right_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "selfie_stm32_bridge");
  ros::NodeHandle n;
  ros::Publisher imu_publisher = n.advertise<sensor_msgs::Imu>("imu", 100);
  ros::Publisher velo_publisher = n.advertise<std_msgs::Float32>("speed", 50);
  ros::Publisher dis_publisher = n.advertise<std_msgs::Float32>("distance", 50);
  ros::Publisher button1_publisher = n.advertise<std_msgs::Bool>("start_button1", 50);  
  ros::Publisher button2_publisher = n.advertise<std_msgs::Bool>("start_button2", 50); 
  ros::Publisher reset_vision_publisher = n.advertise<std_msgs::Bool>("reset_vision", 50);  
  ros::Publisher switch_state_publisher = n.advertise<std_msgs::UInt8>("switch_state", 50);  
 

  ros::Subscriber ackerman_subscriber = n.subscribe("drive", 1, ackermanCallback);
  ros::Subscriber left_turn_indicator_subscriber = n.subscribe("left_turn_indicator", 1, left_turn_indicatorCallback);
  ros::Subscriber right_turn_indicator_subscriber = n.subscribe("right_turn_indicator", 1, right_turn_indicatorCallback);
  
  //usb communication - read
  uint32_t timestamp = 1;

  int16_t velocity = 1;
  int32_t distance = 1;
  int16_t quaternion_x = 1;
  int16_t quaternion_y = 1;
  int16_t quaternion_z = 1;
  int16_t quaternion_w = 1;
  uint16_t yaw = 1;
  int16_t ang_vel_x = 1;
  int16_t ang_vel_y = 1;
  int16_t ang_vel_z = 1;
  int16_t lin_acc_x = 1;
  int16_t lin_acc_y = 1;
  int16_t lin_acc_z = 1;
  uint8_t start_button1 = 0;
  uint8_t start_button2 = 0;
  uint8_t reset_vision = 0;
  uint8_t switch_state = 0;

  Usb.init();

  ros::Time begin = ros::Time::now();

	Usb.indicators.left = 0;
  Usb.indicators.right = 0;
  Usb.control.steering_angle = 0;
  Usb.control.steering_angle_velocity = 0;
  Usb.control.speed = 0;
  Usb.control.acceleration = 0;
  Usb.control.jerk = 0;


  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    uint32_t send_ms = (now.sec - begin.sec) * 1000 + (now.nsec / 1000000);
    Usb.usb_read_buffer(128, timestamp, distance, velocity, quaternion_x, quaternion_y, quaternion_z, quaternion_w, yaw, ang_vel_x,  ang_vel_y, ang_vel_z, lin_acc_x, lin_acc_y, lin_acc_z, start_button1, start_button2, reset_vision, switch_state);
    Usb.usb_send_buffer(send_ms, Usb.control.steering_angle, Usb.control.steering_angle_velocity, Usb.control.speed, Usb.control.acceleration, Usb.control.jerk, Usb.indicators.left, Usb.indicators.right);

    //send imu to msg
    static sensor_msgs::Imu imu_msg;

    imu_msg.header.frame_id = "imu";
    imu_msg.header.stamp = ros::Time::now();

    imu_msg.orientation.x = (float)(quaternion_x / 32767.f);
    imu_msg.orientation.y = (float)(quaternion_y / 32767.f);
    imu_msg.orientation.z = (float)(quaternion_z / 32767.f);
    imu_msg.orientation.w = (float)(quaternion_w / 32767.f);

    imu_msg.linear_acceleration.x = (float)(lin_acc_x / 8192.f * 9.80655f);
    imu_msg.linear_acceleration.y = (float)(lin_acc_y / 8192.f * 9.80655f);
    imu_msg.linear_acceleration.z = (float)(lin_acc_z / 8192.f * 9.80655f);

    imu_msg.angular_velocity.x = (float)(ang_vel_x / 65.535f);
    imu_msg.angular_velocity.y = (float)(ang_vel_y / 65.535f);
    imu_msg.angular_velocity.z = (float)(ang_vel_z / 65.535f);

    //send velocity to msg
    static std_msgs::Float32 velo_msg;
    velo_msg.data = (float)velocity / 1000;

    //send distance to msg
    static std_msgs::Float32 dis_msg;
    dis_msg.data = (float)distance / 1000;
    
    //send status button status to msg
    static std_msgs::Bool button1_msg;
    button1_msg.data = start_button1;
    static std_msgs::Bool button2_msg;
    button2_msg.data = start_button2;

    //send status button status to msg
    static std_msgs::Bool reset_vision_msg;
    reset_vision_msg.data = reset_vision;
    static std_msgs::UInt8 switch_state_msg;
    switch_state_msg.data = switch_state;


    //publishing msg
    imu_publisher.publish(imu_msg);
    velo_publisher.publish(velo_msg);
    dis_publisher.publish(dis_msg);
    button1_publisher.publish(button1_msg);
    button2_publisher.publish(button2_msg);
    reset_vision_publisher.publish(reset_vision_msg);
    switch_state_publisher.publish(switch_state_msg);

    ros::spinOnce();
  }
}

void ackermanCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{
  Usb.control.steering_angle = -msg->drive.steering_angle;
  Usb.control.steering_angle_velocity = msg->drive.steering_angle_velocity;
  Usb.control.speed = msg->drive.speed;
  Usb.control.acceleration = msg->drive.acceleration;
  Usb.control.jerk = msg->drive.jerk;
}

void left_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg)
{
  Usb.indicators.left = (int8_t) msg->data;
}

void right_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg)
{
  Usb.indicators.right = (int8_t) msg->data;
}
