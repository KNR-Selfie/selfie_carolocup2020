/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/

#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "intersection_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
//IntersectionServer

  ros::spin();
  return 0;
}