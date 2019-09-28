#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

#define ODOM_FRAME "odom"
#define IMU_FRAME "imu"

double speed = 0;

double roll = 0;
double pitch = 0;
double yaw = 0;

double base_yaw = 0;
bool yaw_initialized = false;

double x = 0;
double y = 0;

double vx = 0;
double vy = 0;
double vyaw = 0;

double currentDistance = 0;
double lastDistance = 0;
double deltaDistance = 0;
double dt = 0;
double dx = 0;
double dy = 0;

ros::Time current_time, last_distance_update;
bool distance_initialized = false;

ros::Publisher odom_pub;
geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

std::string rear_axis_frame;
tf::StampedTransform imu_transform;

void distanceCallback(const std_msgs::Float32 &msg)
{

  current_time = ros::Time::now();
  currentDistance = msg.data;

  if (distance_initialized)
  {
    deltaDistance = currentDistance - lastDistance;
    dt = (current_time - last_distance_update).toSec();
    if (dt != 0)
    {
      speed = deltaDistance / dt;
      vx = speed * cos(yaw);
      vy = speed * sin(yaw);
    }

    dx = deltaDistance * cos(yaw);
    dy = deltaDistance * sin(yaw);

    x += dx;
    y += dy;
  }

  distance_initialized = true;
  last_distance_update = current_time;
  lastDistance = currentDistance;
}

void imuCallback(const sensor_msgs::Imu &msg)
{
  static tf::TransformListener listener;

  tf::Quaternion q(
    msg.orientation.x,
    msg.orientation.y,
    msg.orientation.z,
    msg.orientation.w
  );

  q = imu_transform * q;

  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  if (!yaw_initialized) {
    base_yaw = yaw;
    yaw_initialized = true;
  }

  yaw -= base_yaw;
  vyaw = msg.angular_velocity.z;

  // quaternion created from yaw
  odom_quat = tf::createQuaternionMsgFromYaw(yaw);
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "selfie_odometry");

  ros::NodeHandle n("~");

  odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
  ros::Subscriber sub_distance = n.subscribe("/distance", 50, distanceCallback);
  ros::Subscriber sub_imu = n.subscribe("/imu", 50, imuCallback);

  rear_axis_frame = n.param<std::string>("rear_axis_frame", "base_link");
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time now = ros::Time::now();

  tf::TransformListener listener;
  listener.waitForTransform(
    rear_axis_frame,
    IMU_FRAME,
    now,
    ros::Duration(3.0)
  );
  listener.lookupTransform(
    rear_axis_frame,
    IMU_FRAME,
    ros::Time(0),
    imu_transform
  );

  ros::Rate loop_rate(10);

  while (n.ok())
  {

    // check for incoming messages
    ros::spinOnce();
    current_time = ros::Time::now();

    // publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = ODOM_FRAME;
    odom_trans.child_frame_id = rear_axis_frame;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = ODOM_FRAME;

    // set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = rear_axis_frame;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vyaw;

    // publish the message
    odom_pub.publish(odom);

    loop_rate.sleep();
  }
}
