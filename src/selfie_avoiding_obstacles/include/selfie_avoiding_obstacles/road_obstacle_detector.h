#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <list>
#include <ros/ros.h>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pid/PidConfig.h>
#include <pid/pid.h>
#include <selfie_avoiding_obstacles/LaneControllerConfig.h>
#include <selfie_msgs/PolygonArray.h>
#include <selfie_msgs/RoadMarkings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <ros/console.h>

#include <selfie_park/shapes.h>

using namespace std;
using namespace pid;
using namespace pid_ns;
using namespace dynamic_reconfigure;

class Road_obstacle_detector
{
public:
  Road_obstacle_detector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~Road_obstacle_detector();

private:
  enum status
  {
    ON_RIGHT,
    OVERTAKE,
    ON_LEFT,
    RETURN,
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
  ros::Publisher right_indicator_pub_;
  ros::Publisher left_indicator_pub_;
  // Two services as switches activating active/passive mode
  ros::ServiceServer passive_mode_service_;
  ros::ServiceServer active_mode_service_;
  ros::ServiceServer reset_node_service_;
  ros::Timer timer_;
  // services used for switching ackermann mode
  ros::ServiceClient ackerman_steering_service_;
  ros::ServiceClient front_axis_steering_service_;
  // Polymonial coefficients describing road markings
  float left_line_[4];
  float center_line_[4];
  float right_line_[4];

  // area of interest (camera's field of view)
  float ROI_min_x_;
  float ROI_max_x_;
  float ROI_min_y_;
  float ROI_max_y_;

  // area of interest for checking if there is any car on right (while overtaking)
  float right_obst_area_min_x_;
  float right_obst_area_max_x_;
  float right_obst_area_min_y_;
  float right_obst_area_max_y_;
  // Setpoints for lanes
  float right_lane_;
  float left_lane_;

  float max_speed_;
  float slowdown_speed_;
  float lane_change_speed_;
  std_msgs::Float64 speed_message_;

  float max_distance_to_obstacle_; // to avoid changing lane too early
  float max_length_of_obstacle_;

  float safety_margin_; // safety margin considering inaccurations in measuring distance etc..
  float current_distance_;
  float current_offset_;
  float return_distance_; // after passing this distance car returns on right lane

  float lane_change_distance_;                // returning from left lane should be gradual, and it should take about
                                              // "lane_change_distance_" meters
  float distance_when_started_changing_lane_; // saved when we begin changing lane

  int proof_slowdown_;
  int num_proof_to_slowdown_;
  int proof_return_;
  int num_proof_to_return_;
  int num_corners_to_detect_;

  std::list<Box> filtered_boxes_; // boxes are sorted by x valule
  // ascendend (near->far)
  std::list<Box>::iterator nearest_box_in_front_of_car_;
  std_msgs::Float64 setpoint_value_;

  bool visualization_;
  bool ackermann_mode_;
  visualization_msgs::Marker empty_marker_;
  Box area_of_interest_box_;
  Box right_obst_area_box_;
  bool received_road_markings_;
  bool return_distance_calculated_;
  status status_;

  dynamic_reconfigure::Server<selfie_avoiding_obstacles::LaneControllerConfig> dr_server_;
  dynamic_reconfigure::Server<selfie_avoiding_obstacles::LaneControllerConfig>::CallbackType dr_server_CB_;
  // variables used for changing settings of PID
  dynamic_reconfigure::ReconfigureRequest srv_req_;
  dynamic_reconfigure::ReconfigureResponse srv_resp_;
  dynamic_reconfigure::DoubleParameter double_param_;
  dynamic_reconfigure::Config conf_;
  Client<PidConfig> df_pid_client_;
  double old_Kp_;
  double old_kp_scale_;
  double lane_change_kp_;
  bool old_pid_saved_;

  ros::ServiceClient turn_on_speed_tuner_;
  ros::ServiceClient turn_off_speed_tuner_;

  void reconfigureCB(selfie_avoiding_obstacles::LaneControllerConfig &config, uint32_t level);

  void filter_boxes(const selfie_msgs::PolygonArray &);           // filters boxes and saves in filtered_boxes_
  void road_markings_callback(const selfie_msgs::RoadMarkings &); // checks if boxes from filtered_boxes_ are on right lane
  void obstacle_callback(const selfie_msgs::PolygonArray &);
  void distanceCallback(const std_msgs::Float32 &);
  void calculate_return_distance();

  void changePidSettings(float);
  void restorePidSettings();
  void pidDynamicReconfigureCb(const PidConfig &);

  bool switchToActive(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
  bool switchToPassive(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
  bool reset_node(std_srvs::Empty::Request &, std_srvs::Empty::Response &);

  void blinkLeft(bool on);
  void blinkRight(bool on);

  bool is_on_right_lane(const Point &);
  bool is_obstacle_next_to_car(const selfie_msgs::PolygonArray &);
  void passive_timer_cb(const ros::TimerEvent &);

  void visualizeBoxes();
};

/*
//TODO
update README

*/
