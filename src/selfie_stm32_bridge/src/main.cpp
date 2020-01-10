#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <selfie_stm32_bridge/usb.hpp>
#include <selfie_stm32_bridge/bridge.h>
#include <sstream>

void ackermanCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);
void left_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg);
void right_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg);


Pub_messages pub_messages;
Sub_messages sub_messages;

USB_STM Usb;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "selfie_stm32_bridge");
    ros::NodeHandle n;
    ros::Publisher imu_publisher = n.advertise<sensor_msgs::Imu>("imu", 100);
    ros::Publisher velo_publisher = n.advertise<std_msgs::Float32>("speed", 50);
    ros::Publisher dis_publisher = n.advertise<std_msgs::Float32>("distance", 50);
    ros::Publisher button1_publisher = n.advertise<std_msgs::Empty>("start_button1", 50);
    ros::Publisher button2_publisher = n.advertise<std_msgs::Empty>("start_button2", 50);
    ros::Publisher reset_vision_publisher = n.advertise<std_msgs::Bool>("reset_vision", 50);
    ros::Publisher switch_state_publisher = n.advertise<std_msgs::UInt8>("switch_state", 50);

    ros::Subscriber ackerman_subscriber = n.subscribe("drive", 1, ackermanCallback);
    ros::Subscriber left_turn_indicator_subscriber = n.subscribe("left_turn_indicator", 1, left_turn_indicatorCallback);
    ros::Subscriber right_turn_indicator_subscriber = n.subscribe("right_turn_indicator", 1, right_turn_indicatorCallback);

    Usb.init();
    Time time;

    while (ros::ok())
    {
        //if new data read - publish
        if(Usb.read_from_STM())
            Usb.fill_publishers(pub_messages);

        //send subscribed data
        Usb.send_to_STM(time.get_ms_time(), sub_messages);

        //publishing msg
        imu_publisher.publish(pub_messages.imu_msg);
        velo_publisher.publish(pub_messages.velo_msg);
        dis_publisher.publish(pub_messages.dist_msg);
        if(pub_messages.button1_msg.data) button1_publisher.publish(std_msgs::Empty());
        if(pub_messages.button2_msg.data) button2_publisher.publish(std_msgs::Empty());
        reset_vision_publisher.publish(pub_messages.reset_vision_msg);
        switch_state_publisher.publish(pub_messages.switch_state_msg);

        ros::spinOnce();
    }
}

void ackermanCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{   
    sub_messages.ackerman.steering_angle = -msg->drive.steering_angle;
    sub_messages.ackerman.steering_angle_velocity = msg->drive.steering_angle_velocity;
    sub_messages.ackerman.speed = msg->drive.speed;
    sub_messages.ackerman.acceleration = msg->drive.acceleration;
    sub_messages.ackerman.jerk = msg->drive.jerk;
}

void left_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg)
{
    sub_messages.indicator.left = (int8_t) msg->data;
}

void right_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg)
{   
    sub_messages.indicator.right = (int8_t) msg->data;
}
