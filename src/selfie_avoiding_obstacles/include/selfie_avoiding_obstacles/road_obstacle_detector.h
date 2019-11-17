#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <list>
#include <ros/ros.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <selfie_msgs/PolygonArray.h>
#include <selfie_msgs/RoadMarkings.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <ros/console.h>

#include <selfie_park/shapes.h>

using namespace std;

class Road_obstacle_detector
{
public:
  Road_obstacle_detector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~Road_obstacle_detector();

private:
  enum status
  {
    CLEAR,
    OVERTAKING,
    PASSIVE,
  };
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obstacles_sub_;
  ros::Subscriber markings_sub_;
  ros::Subscriber distance_sub_;
  ros::Publisher speed_pub_;
  ros::Publisher visualizer_;
  ros::Publisher setpoint_pub_;
  // Two services as switches activating active/passive mode
  ros::ServiceServer passive_mode_service_;
  ros::ServiceServer active_mode_service_;
  ros::ServiceServer reset_node_service_;
  ros::Timer timer_;
  // Polymonial coefficients describing road markings
  float left_line_[4];
  float center_line_[4];
  float right_line_[4];

  // area of interest (camera's field of view)
  float ROI_min_x_;
  float ROI_max_x_;
  float ROI_min_y_;
  float ROI_max_y_;
  // Setpoints for lanes
  float right_lane_;
  float left_lane_;

  float max_speed_;
  float safe_speed_;
  std_msgs::Float64 speed_message_;

  float maximum_distance_to_obstacle_; // to avoid changing lane too early
  float maximum_length_of_obstacle_;

  float safety_margin_; // safety margin considering inaccurations in measuring distance etc..
  float current_distance_;
  float return_distance_; // after passing this distance car returns on right lane

  std::list<Box> filtered_boxes_; // boxes are sorted by x valule
  // ascendend (near->far)
  std::list<Box>::iterator nearest_box_in_front_of_car_;
  std_msgs::Float64 setpoint_value_;

  bool visualization_;
  visualization_msgs::Marker empty_marker_;
  Box area_of_interest_box_;
  bool received_road_markings_;
  status status_;

  void filter_boxes(const selfie_msgs::PolygonArray &);           // filters boxes and saves in filtered_boxes_
  void road_markings_callback(const selfie_msgs::RoadMarkings &); // checks if boxes from filtered_boxes_ are on right lane
  void obstacle_callback(const selfie_msgs::PolygonArray &);
  void distanceCallback(const std_msgs::Float32 &);
  void calculate_return_distance();

  bool switchToActive(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
  bool switchToPassive(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
  bool reset_node(std_srvs::Empty::Request &, std_srvs::Empty::Response &);

  void change_lane(float lane);

  bool is_on_right_lane(const Point &);
  void passive_timer_cb(const ros::TimerEvent &);

  void visualizeBoxes();
};

/*
//TODO
update README

*/