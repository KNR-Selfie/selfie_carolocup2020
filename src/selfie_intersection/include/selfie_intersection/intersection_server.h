#pragma once

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <selfie_msgs/PolygonArray.h>
#include <selfie_msgs/intersectionAction.h>
#include <std_msgs/Float32.h>

#include <dynamic_reconfigure/server.h>
#include <selfie_intersection/IntersectionServerConfig.h>
#include <selfie_park/shapes.h>
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
  ros::Subscriber intersection_subscriber_;
  ros::Subscriber distance_subscriber_;
  ros::Publisher speed_publisher_;
  ros::Publisher visualize_intersection_;

  double beginning_time_;
  double current_time_;
  double difftime_;
  bool time_started_;
  float stop_time_; // How long car should stop when approached intersection
  float speed_default_;

  float point_min_x_; // Area of interest
  float point_max_x_;
  float point_min_y_;
  float point_max_y_;
  float road_width_;
  float max_distance_to_intersection_; // Describrs how far before intersection
                                       // car should stop
  float distance_of_blind_approaching_;
  float distance_to_stop_blind_approaching_;
  float current_distance_;
  bool approached_blindly_;
  float distance_to_intersection_when_started_;
  bool is_distance_to_intersection_saved_;
  float distance_when_started_;
  bool is_distance_saved_;

  int num_corners_to_detect_;
  bool visualization_;

  std::list<Box> filtered_boxes_;
  selfie_msgs::intersectionFeedback action_status_;
  selfie_msgs::intersectionGoal goal_;
  std_msgs::Float64 speed_;

  actionlib::SimpleActionServer<selfie_msgs::intersectionAction> intersectionServer_;

  dynamic_reconfigure::Server<selfie_intersection::IntersectionServerConfig> dr_server_;
  dynamic_reconfigure::Server<selfie_intersection::IntersectionServerConfig>::CallbackType dr_server_CB_;

  void init();
  void preemptCb();
  void manager(const selfie_msgs::PolygonArray &);
  void intersection_callback(const std_msgs::Float32 &);
  void distance_callback(const std_msgs::Float32 &);
  void filter_boxes(const selfie_msgs::PolygonArray &);
  void publishFeedback(program_state newStatus);
  void send_goal();
  void reconfigureCB(selfie_intersection::IntersectionServerConfig &config, uint32_t level);

  void visualizeBoxes(std::list<Box> boxes, float r, float g, float b);
};