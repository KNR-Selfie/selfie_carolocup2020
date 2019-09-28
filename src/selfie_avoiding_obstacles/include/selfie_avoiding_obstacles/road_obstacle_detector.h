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
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obstacles_sub_;
  ros::Subscriber markings_sub_;
  ros::Publisher visualizer_;
  // Polymonial coefficients describing road markings
  float left_line_[3];
  float center_line_[3];
  float right_line_[3];

  std::list<Box> filtered_boxes_; // boxes are sorted by x valule
                                  // ascendend (near->far)

  bool visualization_;
  bool received_road_markings_;

  void filter_boxes(const selfie_msgs::PolygonArray &);           // filters boxes and saves in filtered_boxes_
  void road_markings_callback(const selfie_msgs::RoadMarkings &); // checks if boxes from filtered_boxes_ are on right lane

  bool is_on_right_lane(const Point &);
  /*
    void display_places(std::vector<Box> &, const std::string &);
    void display_place(Box &, const std::string &);
    */
};
