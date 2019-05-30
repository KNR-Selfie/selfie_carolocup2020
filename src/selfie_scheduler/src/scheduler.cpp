#include <ros/ros.h>
#include <selfie_scheduler/starting_action_client.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scheduler");
    ros::NodeHandle nh;

    ROS_INFO("Hello world!");
}
