#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <selfie_park/parkAction.h>
#include <geometry_msgs/Polygon.h>

class ParkClient
{
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<selfie_park::parkAction> ac_;
    public:
    void send_goal();
    ParkClient(const ros::NodeHandle &nh);

};
