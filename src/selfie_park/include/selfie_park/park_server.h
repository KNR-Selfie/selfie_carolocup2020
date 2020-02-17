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
#include <std_srvs/Empty.h>
#include <selfie_msgs/RoadMarkings.h>


class ParkService
{
public:
  ParkService(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

private:
  const float PARK_SPOT_WIDTH = 0.3;

  ros::Subscriber dist_sub_;
  ros::NodeHandle nh_, pnh_;
  actionlib::SimpleActionServer <selfie_msgs::parkAction> as_;
  ros::Publisher ackermann_pub_;
  ros::Publisher right_indicator_pub_;
  ros::Publisher left_indicator_pub_;
  ros::Subscriber markings_sub_;
  ros::ServiceClient steering_mode_set_parallel_;
  ros::ServiceClient steering_mode_set_front_axis_;

  dynamic_reconfigure::Server<selfie_park::ParkServerConfig> dr_server_;
  dynamic_reconfigure::Server<selfie_park::ParkServerConfig>::CallbackType dr_server_CB_;
  void reconfigureCB(selfie_park::ParkServerConfig& config, uint32_t level);

  void distanceCallback(const std_msgs::Float32 &msg);
  void markingsCallback(const selfie_msgs::RoadMarkings &msg);
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

  float parking_speed_;

  float actual_dist_;
  float prev_dist_;

  float park_spot_dist_;
  float park_spot_dist_ini_;
  

  float front_target_;
  float back_target_;
  float park_spot_middle_;
  std::vector<float> right_line_;
  ros::Time delay_end_;
  float out_target_;



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
  float max_distance_to_wall_;
  float max_turn_;
  float idle_time_;
  float iter_distance_;
  float back_to_mid_;
  std::string odom_topic_;
  float angle_coeff_;
  float turn_delay_;
  float line_dist_end_;
  float start_parking_speed_;
};
