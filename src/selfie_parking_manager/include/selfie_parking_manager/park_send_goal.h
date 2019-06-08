#pragma once

#include<stdlib.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <selfie_msgs/searchAction.h>
#include <geometry_msgs/Polygon.h>

class park_send_goal
{
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<selfie_msgs::searchAction> ac_;
    public:
    void send_goal(const float);
    park_send_goal(const ros::NodeHandle &nh);

};

park_send_goal::park_send_goal(const ros::NodeHandle &nh):
nh_(nh),
ac_("search", true)
{}

void park_send_goal::send_goal(const float spot_length=0.5){
    selfie_msgs::searchGoal msg;
    msg.min_spot_lenght=spot_length;//Czy nie za mało/dużo
    ac_.sendGoal(msg);

}