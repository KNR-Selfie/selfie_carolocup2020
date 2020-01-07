/***Copyright ( c) 2019, KNR Selfie*
 * This code is licensed under BSD license (see LICENSE for details)
 **/
#include "selfie_starting_procedure/gate_scanner.h"

GateScanner::GateScanner(const ros::NodeHandle &nh, const ros::NodeHandle &pnh):
nh_(nh), pnh_(pnh)
{
  pnh_.param<float>("no_obstacle_time_thresh", no_obstacle_time_thresh_, 1.5f);
  pnh_.param<float>("min_distance", min_distance_, 0.1);
  pnh_.param<float>("max_distance", max_distance_, 0.5);
  pnh_.param<float>("min_width", min_width_, 0.2);
  pnh_.param<int>("min_gate_seen_count", min_gate_seen_count_, 5);
  timer_ = nh_.createTimer(ros::Duration(no_obstacle_time_thresh_), &GateScanner::timerCallback, this, true, false);
  start_serv_ = nh_.advertiseService("startGateScan", &GateScanner::startSearching, this);
  gate_open_pub_ = nh_.advertise<std_msgs::Empty>("scan_gate_open", 1);
  if(min_gate_seen_count_ < 1) min_gate_seen_count_ = 1;
  gate_seen_count_ = 0;
}

void GateScanner::laserScanCB(const sensor_msgs::LaserScan &msg)
{
  float angle = msg.angle_min;
  for (std::vector<float>::const_iterator it = msg.ranges.begin(); it < msg.ranges.end();
   ++it, angle += msg.angle_increment)
  {
    if (*it > msg.range_max || *it < msg.range_min) continue;
    Point point(angle, *it);
    {
      if (point.x > min_distance_ && point.x < max_distance_ && std::abs(point.y) < min_width_ / 2.f)
      {
        if(++gate_seen_count_ == min_gate_seen_count_)
        {
          timer_.start();
        }
        else if(gate_seen_count_ > min_gate_seen_count_)
        {
          resetTimer();
        }
        return;
        
      }
    }
  }
}

void GateScanner::timerCallback(const ros::TimerEvent &e)
{
  gate_open_pub_.publish(std_msgs::Empty());
  ROS_INFO("timer callback");
  scan_sub_.shutdown();
}

void GateScanner::resetTimer()
{
  timer_.setPeriod(ros::Duration(no_obstacle_time_thresh_), true);  // reset
  ROS_INFO("timer reset");
}

bool GateScanner::startSearching(std_srvs::Empty::Request &rq, std_srvs::Empty::Response &rp)
{
  scan_sub_ = nh_.subscribe("scan", 10, &GateScanner::laserScanCB, this);
  return true;
}

GateScanner::Point::Point(float ang, float range):x(cos(ang)*range), y(sin(ang)*range){}
