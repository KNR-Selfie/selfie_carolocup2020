#pragma once

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Polygon.h>
#include <ros/ros.h>
#include <selfie_msgs/searchAction.h>
#include <stdlib.h>

class Search_client_mock {
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<selfie_msgs::searchAction> ac_;

public:
  void send_goal(const float);
  Search_client_mock(const ros::NodeHandle &nh);
};

Search_client_mock::Search_client_mock(const ros::NodeHandle &nh)
    : nh_(nh), ac_("search", true) {}

void Search_client_mock::send_goal(const float spot_length = 0.5) {
  selfie_msgs::searchGoal msg;
  msg.min_spot_lenght = spot_length; // Czy nie za mało/dużo
  ac_.sendGoal(msg);
}