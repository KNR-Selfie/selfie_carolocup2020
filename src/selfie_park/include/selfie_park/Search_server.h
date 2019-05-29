#pragma once
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <chrono>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>
#include <selfie_msgs/PolygonArray.h>

#include <actionlib/server/simple_action_server.h>
#include <selfie_msgs/searchAction.h>
#include <selfie_scheduler/scheduler_enums.h>



#include <ros/console.h>

#include "shapes.h"

using namespace std;


class Search_server{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obstacles_sub;
  ros::Publisher visualize_lines_pub;
  ros::Publisher visualize_free_place;
  ros::Publisher point_pub;
  ros::Publisher parking_state_pub;
  ros::Publisher parking_place_pub;
  ros::Publisher speed_publisher;

  actionlib::SimpleActionServer<selfie_msgs::searchAction> search_server_;

  std::vector<Box> boxes_on_the_right_side;//boxy są posortowane wg wartości x rosnąco (bliskie->dlasze)
  std::vector<Box> potential_free_places;
  // for now only this is used
  std::vector<Box> for_planning;
  Box first_free_place;
  float distance_to_stop;
  float min_spot_lenght;
  int visualization_type;
  int scans_ignored;
  int scans_taken;
  bool debug_mode;


  float default_speed_in_parking_zone;
  std_msgs::Float64 speed_current;

  selfie_msgs::searchFeedback feedback_msg;
  selfie_msgs::searchResult result;



  double planning_scan_counter=0;

  // jednostki w metrach
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
  void filter_boxes(const selfie_msgs::PolygonArray &);//odfiltrowywuje boxy, pozostawia tylko te po prawej
  bool find_free_places();
  void send_goal();

  void display_places(std::vector<Box> &, const std::string &);
  void display_place(Box &,const std::string &);
};
