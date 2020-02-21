#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <selfie_fuzzy_logic/fuzzycontroller.h>
#include <selfie_fuzzy_logic/membership.h>
#include <selfie_fuzzy_logic/rule.h>

ros::Publisher steering_relationship_pub;
ros::Publisher steering_angle_pub;

float curvature = 0;
float offset = 0;

void curvatureCallback(const std_msgs::Float64 &msg)
{
  curvature = msg.data;
}

void combinedOffsetCallback(const std_msgs::Float64 &msg)
{
  offset = msg.data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "selfie_fuzzy_controller");

  ros::NodeHandle n("~");

  ros::Publisher steering_relationship_pub = n.advertise<std_msgs::Float32>("/steering_relationship", 50);
  ros::Publisher steering_angle_pub = n.advertise<std_msgs::Float64>("/steering_angle", 50);

  ros::Subscriber sub_curvature = n.subscribe("/curvature", 50, curvatureCallback);
  ros::Subscriber sub_combined_offset = n.subscribe("/combined_offset", 50, combinedOffsetCallback);

  ros::Rate loop_rate(120);

  while (n.ok())
  {

    // check for incoming messages
    ros::spinOnce();

    loop_rate.sleep();
  }
}
