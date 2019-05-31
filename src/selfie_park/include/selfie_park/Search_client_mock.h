#pragma once

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Polygon.h>
#include <ros/ros.h>
#include <selfie_msgs/searchAction.h>
#include <selfie_scheduler/scheduler_enums.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>


void topicCallback(const std_msgs::Float64ConstPtr &msg);
void feedbackCb(const selfie_msgs::searchFeedbackConstPtr &feedback);

class Search_client_mock {
public:
  static float last_speed_;
  void send_goal(const float);
  Search_client_mock(const ros::NodeHandle &nh);void activeCb() { ac_.waitForServer(); }
  void doneCb(const actionlib::SimpleClientGoalState& state,const selfie_msgs::searchResultConstPtr& result){}
  actionlib::SimpleActionClient<selfie_msgs::searchAction> ac_;

private:
  ros::NodeHandle nh_;
  ros::Subscriber speed_subscriber_;
  
  
  
};
