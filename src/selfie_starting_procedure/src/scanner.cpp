/***Copyright ( c) 2019, KNR Selfie*
 * This code is licensed under BSD license (see LICENSE for details)
 **/
#include "selfie_starting_procedure/gate_scanner.h"

GateScanner::GateScanner(const ros::NodeHandle &nh, const ros::NodeHandle &pnh):
nh_(nh), pnh_(pnh)
{
  pnh_.param<float>("distance_threshold", distance_threshold_, 1.0);
  pnh_.param<float>("gate_width", gate_width_, 0.2);
  pnh_.param<int>("number_of_saved_measurements", number_of_saved_measurements_, 10);
  pnh_.param<int>("no_obstacles_limit", no_obstacles_limit_, 10);
  
  start_serv_ = nh_.advertiseService("startGateScan", &GateScanner::startSearching, this);
  stop_serv_ = nh_.advertiseService("stopGateScan", &GateScanner::stopSearching, this);
  gate_open_pub_ = nh_.advertise<std_msgs::Empty>("scan_gate_open", 1);

  // publishing tresholded pointcloud
  debug_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("gate_scanner_debug",10);
}

void GateScanner::appendMeasurement(const float &value)
{
  // too little measurements
  if (measurements_.size() > number_of_saved_measurements_)
  {
    measurements_.erase(measurements_.begin());
  }
  measurements_.push_back(value);
}

void GateScanner::checkDistances()
{
  // if mean of all measurements is greater than threshold
  if ((std::accumulate(measurements_.begin(), measurements_.end(), 0.0) /
                      measurements_.size()) > distance_threshold_ &&
       measurements_.back() > distance_threshold_)
  {
    no_obstacles_counter_++;
  }
  else 
  {
    no_obstacles_counter_ = 0;
  }
}

bool GateScanner::checkValidity(){
  return (no_obstacles_counter_ >= no_obstacles_limit_) ? true : false;
}

void GateScanner::laserScanCB(const sensor_msgs::LaserScan &msg)
{
  std::vector<float> valid_measurement;
  ROS_INFO("msg size: %d", static_cast<int>(msg.ranges.size()));
  sensor_msgs::LaserScan debug_msg = msg;
  debug_msg.ranges.clear();
  float angle = msg.angle_min;
  for (std::vector<float>::const_iterator it = msg.ranges.begin(); it < msg.ranges.end();
   ++it, angle += msg.angle_increment)
  {
    // out of bounds
    if (*it > msg.range_max || *it < msg.range_min) continue;
    // converting to cartesian plane
    Point point(angle, *it);
    {
      // checking if measurement is in gate range
      if (std::abs(point.y) < (gate_width_ / 2.f))
      {
        // finding measurement inside gate range
        valid_measurement.push_back(point.x);
        // publishing debug pointcloud
        debug_msg.ranges.push_back(*it);
        ROS_INFO("Vector data: %f", *it);
      }
    }
  }
  // if lidar doesnt produce any data in this direction it means 
  //gate is opened
  if (valid_measurement.size() == 0) 
  {
    no_obstacles_counter_++;
    ROS_INFO("Gate Scanner: counter state %d", no_obstacles_counter_);
    return;
  }
  float mean = std::accumulate(valid_measurement.begin(), valid_measurement.end(), 0.0) / 
                               valid_measurement.size();
  ROS_INFO("Measurement mean: %f", mean);
  debug_scan_pub_.publish(debug_msg);
  appendMeasurement(mean);
  // check if measurements are greater than treshold distance
  checkDistances();
  // check if gate is open
  if (checkValidity()) 
  {
    gate_open_pub_.publish(std_msgs::Empty());
    ROS_INFO("GateScanner: gate open");
    no_obstacles_counter_ = 0;
    scan_sub_.shutdown(); 
  }
  ROS_INFO("Gate Scanner: counter state %d", no_obstacles_counter_);
}

bool GateScanner::startSearching(std_srvs::Empty::Request &rq, std_srvs::Empty::Response &rp)
{
  scan_sub_ = nh_.subscribe("scan", 10, &GateScanner::laserScanCB, this);
  ROS_INFO("GateScanner: start searching");
  return true;
}

bool GateScanner::stopSearching(std_srvs::Empty::Request &rq, std_srvs::Empty::Response &rp)
{
  scan_sub_.shutdown();
  ROS_INFO("GateScanner: stopped searching");
  return true;
}

GateScanner::Point::Point(float ang, float range):x(cos(ang)*range), y(sin(ang)*range){}
