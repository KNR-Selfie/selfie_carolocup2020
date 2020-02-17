/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/ 

#include <ros/ros.h>
#include <selfie_free_drive/free_drive_server.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "free_drive");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  FreeDriveServer free_drive(nh, pnh);

  ros::spin();
  return 0;
}
