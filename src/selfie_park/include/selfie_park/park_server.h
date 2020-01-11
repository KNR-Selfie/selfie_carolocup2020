#pragma once


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <selfie_msgs/parkAction.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <tf/tf.h>
#include <vector>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <string>
#include <vector>
#include <selfie_scheduler/scheduler_enums.h>
#include <selfie_park/ParkServerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float32.h>
#include <algorithm>


class ParkService
{
public:
  ParkService(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

private:

  ros::Subscriber dist_sub_;
  ros::NodeHandle nh_, pnh_;
  actionlib::SimpleActionServer <selfie_msgs::parkAction> as_;
  ros::Publisher ackermann_pub_;
  ros::Publisher right_indicator_pub_;
  ros::Publisher left_indicator_pub_;

  dynamic_reconfigure::Server<selfie_park::ParkServerConfig> dr_server_;
  dynamic_reconfigure::Server<selfie_park::ParkServerConfig>::CallbackType dr_server_CB_;
  void reconfigureCB(selfie_park::ParkServerConfig& config, uint32_t level);

  void distanceCallback(const std_msgs::Float32 &msg);
  void goalCB();
  void preemptCB();

  void drive(float speed, float steering_angle);
  bool toParkingSpot();
  bool park();
  bool leave();
  void initParkingSpot(const geometry_msgs::Polygon &msg);
  void blinkLeft(bool on);
  void blinkRight(bool on);


  enum Parking_State
  {
    not_parking = 0,
    go_to_parking_spot = 1,
    going_in = 2,
    parked = 3,
    get_straight = 7,
    going_out = 4,
    out = 5,
    go_back = 6
  } parking_state_;

  feedback_variable action_status_;

  float front_wall_;
  float back_wall_;
  float middle_of_parking_spot_y_;
  float middle_of_parking_spot_x_;
  float parking_spot_width_;
  float leaving_target_;
  float parking_speed_;
  float mid_y_;

  float actual_dist_;
  float prev_dist_;

  float park_spot_dist_;
  float park_spot_dist_ini_;
  

  float front_target_;
  float back_target_;


  enum Move_State
  {
    first_phase = 0,
    straight = 1,
    second_phase = 2,
    end = 3
  } move_state_;

  //params
  std::string ackermann_topic_;
  float minimal_start_parking_x_;
  bool state_msgs_;
  float max_rot_;
  float max_distance_to_wall_;
  float dist_turn_;
  float odom_to_front_;
  float odom_to_back_;
  float odom_to_laser_;
  float max_turn_;
  float idle_time_;
  float iter_distance_;
  float car_length_;
  std::string odom_topic_;
};
