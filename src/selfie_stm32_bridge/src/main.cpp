#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <selfie_stm32_bridge/usb.hpp>
#include <selfie_stm32_bridge/bridge.h>
#include <sstream>

void ackermanCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);
void left_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg);
void right_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg);
bool steeringAckermanCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool steeringParallelCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool steeringFrontAxisCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

Pub_messages pub_messages;
Sub_messages sub_messages;

USB_STM Usb;
int steering_mode = 0;
std_msgs::Empty empty_msg;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "selfie_stm32_bridge");
    ros::NodeHandle n;
    ros::Publisher imu_publisher = n.advertise<sensor_msgs::Imu>("imu", 100);
    ros::Publisher velo_publisher = n.advertise<std_msgs::Float32>("speed", 50);
    ros::Publisher dis_publisher = n.advertise<std_msgs::Float32>("distance", 50);
    ros::Publisher button1_publisher = n.advertise<std_msgs::Empty>("start_button1", 50);
    ros::Publisher button2_publisher = n.advertise<std_msgs::Empty>("start_button2", 50);
    ros::Publisher switch_state_publisher = n.advertise<std_msgs::UInt8>("switch_state", 50);

    ros::ServiceServer ackerman_steering_mode = n.advertiseService("steering_ackerman", steeringAckermanCallback);
    ros::ServiceServer parallel_steering_mode = n.advertiseService("steering_parallel", steeringParallelCallback);
    ros::ServiceServer front_axis_steering_mode = n.advertiseService("steering_front_axis", steeringFrontAxisCallback);

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
        Usb.send_frame_to_STM(time.get_ms_time(), sub_messages);

        //publishing msg
        imu_publisher.publish(pub_messages.imu_msg);
        velo_publisher.publish(pub_messages.velo_msg);
        dis_publisher.publish(pub_messages.dist_msg);
        switch_state_publisher.publish(pub_messages.futaba_state);

        if(pub_messages.button_1)
            button1_publisher.publish(empty_msg);
        if(pub_messages.button_2)
            button2_publisher.publish(empty_msg);

        ros::spinOnce();
    }
}

void ackermanCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{   
    sub_messages.ackerman.steering_angle_front = -msg->drive.steering_angle;
    if(steering_mode == 1)
    {
        sub_messages.ackerman.steering_angle_back = msg->drive.steering_angle;
    }
    else if(steering_mode == 0)
    {
        sub_messages.ackerman.steering_angle_back = -msg->drive.steering_angle;
    }
    else if(steering_mode == 2)
    {
        sub_messages.ackerman.steering_angle_back = 0;
    }
        
    sub_messages.ackerman.speed = msg->drive.speed;
    sub_messages.ackerman.acceleration = msg->drive.acceleration;
    sub_messages.ackerman.jerk = msg->drive.jerk;
}

bool steeringAckermanCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    steering_mode = 0;
    return true;
}

bool steeringParallelCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    steering_mode = 1;
    return true;
}

bool steeringFrontAxisCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    steering_mode = 2;
    return true;
}

void left_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
        Usb.send_cmd_to_STM(left_indicator_on);
    else
        Usb.send_cmd_to_STM(left_indicator_off);
}

void right_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg)
{   
    if(msg->data)
        Usb.send_cmd_to_STM(rigth_indicator_on);
    else
        Usb.send_cmd_to_STM(rigth_indicator_off);
}


