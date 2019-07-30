#pragma once

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <selfie_msgs/PolygonArray.h>
#include <selfie_msgs/intersectionAction.h>

#include <selfie_park/shapes.h>
#include <selfie_scheduler/scheduler_enums.h>

/*
TODO
dokumentacja
*/

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
  ros::Publisher visualize_intersection_;

  float point_min_x_; // Area of interest
  float point_max_x_;
  float point_min_y_;
  float point_max_y_;
  bool visualization_;

  std::list<Box> filtered_boxes_;
  selfie_msgs::intersectionFeedback action_status_;
  selfie_msgs::intersectionGoal goal_;

  actionlib::SimpleActionServer<selfie_msgs::intersectionAction> intersectionServer_;
  void init();
  void manager(const selfie_msgs::PolygonArray &);
  void filter_boxes(const selfie_msgs::PolygonArray &);
  void publishFeedback(program_states newStatus);
  void send_goal();
  void visualizeBoxes(std::list<Box> boxes,float r,float g, float b);
};