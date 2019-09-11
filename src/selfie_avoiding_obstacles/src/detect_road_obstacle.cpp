/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/

#include <ros/ros.h>
#include <selfie_avoiding_obstacles/road_obstacle_detector.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_road_obstacle");
  ros::NodeHandle nh;


  ros::spin();
  return 0;
}