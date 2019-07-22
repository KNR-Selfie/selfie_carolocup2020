#pragma once

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <selfie_msgs/intersectionAction.h>
#include <selfie_scheduler/scheduler_enums.h>

class IntersectionServer
{
public:
  IntersectionServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~IntersectionServer() {}

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obstacles_sub_;
  ros::Publisher visualize_free_place_;
  ros::Publisher speed_publisher_;

  float point_min_x_;//Area of interest
  float point_max_x_;
  float point_min_y_;
  float point_max_y_;

  actionlib::SimpleActionServer<selfie_msgs::intersectionAction> intersectionServer_;
};