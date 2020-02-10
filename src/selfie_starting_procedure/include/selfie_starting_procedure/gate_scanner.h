#ifndef GATE_SCANNER_H 
#define GATE_SCANNER_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <stdlib.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Empty.h>

class GateScanner
{


  ros::NodeHandle nh_,pnh_;
  ros::Subscriber scan_sub_; 
  ros::ServiceServer start_serv_, stop_serv_;
  ros::Publisher gate_open_pub_, debug_scan_pub_;

  std::vector<float> measurements_;

  // float no_obstacle_time_thresh_;
  // float min_distance_;
  // int min_gate_seen_count_;
  // int gate_seen_count_;
  float gate_width_;
  float distance_threshold_;

  int number_of_saved_measurements_;
  int no_obstacles_limit_;
  int no_obstacles_counter_{0};

  ros::Timer timer_;

  bool startSearching(std_srvs::Empty::Request &rq, std_srvs::Empty::Response &rp);
  bool stopSearching(std_srvs::Empty::Request &rq, std_srvs::Empty::Response &rp);
  void checkDistances();
  bool checkValidity();
  void laserScanCB(const sensor_msgs::LaserScan &msg);
  void appendMeasurement(const float &value);
  // void timerCallback(const ros::TimerEvent &e);
  // void resetTimer();

  struct Point
  {
    float x;
    float y;
    Point(float ang, float range);
  };
public:
  GateScanner(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

};
#endif