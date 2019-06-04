#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <selfie_msgs/RoadMarkings.h>

std_msgs::Float64 _max_speed;

void maxSpeedCallback(const std_msgs::Float64 &msg)
{
  _max_speed = msg;
}

void roadMarkingsCallback(const selfie_msgs::RoadMarkings &msg)
{
  ;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planner");

  ros::NodeHandle n("~");

  ros::Publisher speed_pub = n.advertise<std_msgs::Float64>("/speed", 50);
  ros::Publisher postion_offset_pub = n.advertise<std_msgs::Float64>("/postion_offset", 50);
  ros::Publisher heading_offset_pub = n.advertise<std_msgs::Float64>("/heading_offset", 50);
  ros::Subscriber sub_road_markings = n.subscribe("/road_markings", 50, roadMarkingsCallback);
  ros::Subscriber sub_max_speed = n.subscribe("/max_speed", 50, maxSpeedCallback);

  _max_speed.data = 1;

  while (ros::ok())
  {
    ros::spin();
  }
}
