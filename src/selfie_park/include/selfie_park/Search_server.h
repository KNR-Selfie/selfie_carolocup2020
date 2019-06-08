#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <vector>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <selfie_msgs/PolygonArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>

#include <actionlib/server/simple_action_server.h>
#include <selfie_msgs/searchAction.h>
#include <selfie_scheduler/scheduler_enums.h>

#include <ros/console.h>

#include "shapes.h"

using namespace std;

class Search_server {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obstacles_sub;
  ros::Publisher visualize_lines_pub;
  ros::Publisher visualize_free_place;
  ros::Publisher point_pub;
  ros::Publisher speed_publisher;

  actionlib::SimpleActionServer<selfie_msgs::searchAction> search_server_;

  std::vector<Box> boxes_on_the_right_side; // boxes are sorted by x valule
                                            // ascendind (near->far)
  std::vector<Box> potential_free_places;
  Box first_free_place;

  float min_spot_lenght;
  bool visualization;

  float default_speed_in_parking_zone;
  std_msgs::Float64 speed_current;

  selfie_msgs::searchFeedback action_status;
  void publishFeedback(unsigned int);
  selfie_msgs::searchResult result;

  //area of interest (used unit- meter)
  float point_min_x;
  float point_max_x;

  float point_min_y;
  float point_max_y;

public:
  Search_server(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~Search_server();

  bool init();
  void preemptCB();
  void manager(const selfie_msgs::PolygonArray &);
  void filter_boxes(const selfie_msgs::PolygonArray &); // odfiltrowywuje boxy,
                                                        // pozostawia tylko te
                                                        // po prawej
  bool find_free_places();
  void send_goal();

  void display_places(std::vector<Box> &, const std::string &);
  void display_place(Box &, const std::string &);
};
