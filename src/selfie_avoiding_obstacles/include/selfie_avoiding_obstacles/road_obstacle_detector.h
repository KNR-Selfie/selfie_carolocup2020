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
#include <visualization_msgs/Marker.h>

#include <ros/console.h>

#include <selfie_park/shapes.h>

#define LEFT 0.2
#define RIGHT -0.2

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
    OVERTAKING
  };
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obstacles_sub_;
  ros::Subscriber markings_sub_;
  ros::Subscriber speed_sub_;
  ros::Publisher visualizer_;
  ros::Publisher setpoint_pub_;
  // Polymonial coefficients describing road markings
  float left_line_[4];
  float center_line_[4];
  float right_line_[4];

  // area of interest (camera's field of view)
  float point_min_x_;
  float point_max_x_;
  float point_min_y_;
  float point_max_y_;
  

  float maximum_distance_to_obstacle_; // to avoid changing lane too early
  float maximum_length_of_obstacle_;
  float time_left_; // after passing this time car resturns on right lane
  float timer_duration_;
  ros::Timer timer_;

  std::list<Box> filtered_boxes_; // boxes are sorted by x valule
  // ascendend (near->far)
  std::list<Box>::iterator nearest_box_in_front_of_car_;
  std_msgs::Float32 setpoint_value_;

  bool visualization_;
  bool received_road_markings_;
  bool is_time_calculated_for_overtake_;
  status status_;

  void filter_boxes(const selfie_msgs::PolygonArray &);           // filters boxes and saves in filtered_boxes_
  void road_markings_callback(const selfie_msgs::RoadMarkings &); // checks if boxes from filtered_boxes_ are on right lane
  void obstacle_callback(const selfie_msgs::PolygonArray &);
  void calculate_overtake_time(const std_msgs::Float32 &);
  void calculate_time(const ros::TimerEvent &);

  void change_lane(float lane);

  bool is_on_right_lane(const Point &);

  void visualizeBoxes();
};

/*
//TODO
update README

*/