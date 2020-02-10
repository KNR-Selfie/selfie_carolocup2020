#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_srvs/Empty.h>

std_msgs::Float64 _speed;
std_msgs::Float64 _steering_angle;
bool _pub = false;

void steeringCallback(const std_msgs::Float64 &msg)
{
  _steering_angle = msg;
}

void speedCallback(const std_msgs::Float64 &msg)
{
  _speed = msg;
}

bool cmdPubCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  _pub = true;
  ROS_INFO("cmd_creator start publishing");
  return true;
}

bool cmdNotPubCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  _pub = false;
  ROS_INFO("cmd_creator stop publishing");
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_creator");

  ros::NodeHandle n("~");

  ros::Publisher drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 50);
  ros::Subscriber sub_steering_angle = n.subscribe("/steering_angle", 50, steeringCallback);
  ros::Subscriber sub_speed = n.subscribe("/speed", 50, speedCallback);
  ros::ServiceServer pub_service = n.advertiseService("/cmd_start_pub", cmdPubCallback);
  ros::ServiceServer not_pub_service = n.advertiseService("/cmd_stop_pub", cmdNotPubCallback);

  ros::Rate loop_rate(120);
  ackermann_msgs::AckermannDriveStamped drive_msg;

  _speed.data = 0;
  _steering_angle.data = 0;

  while (n.ok())
  {
    // check for incoming messages
    ros::spinOnce();

    if(_pub)
    {
      drive_msg.header.stamp = ros::Time::now();
      drive_msg.drive.speed = _speed.data;
      drive_msg.drive.steering_angle = _steering_angle.data;
      drive_pub.publish(drive_msg);
    }

    loop_rate.sleep();
  }
}
