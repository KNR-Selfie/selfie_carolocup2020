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
  ros::Subscriber obstacles_sub;
  ros::Publisher visualize_free_place;
  ros::Publisher speed_publisher;

  actionlib::SimpleActionServer<selfie_msgs::intersectionAction> intersectionServer_;
};