/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/ 

#ifndef FREE_DRIVE_ACTION_H
#define FREE_DRIVE_ACTION_H

#include <ros/ros.h>
#include <selfie_msgs/drivingAction.h> // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <string>
#include <chrono>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <selfie_scheduler/scheduler_enums.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class FreeDriveAction
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  actionlib::SimpleActionServer<selfie_msgs::drivingAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;

  // create messages that are used to published feedback/result
  selfie_msgs::drivingFeedback feedback_;
  selfie_msgs::drivingResult result_;
  selfie_msgs::drivingGoal goal_;

  //subscribers
  ros::Subscriber starting_line_sub_;
  ros::Subscriber intersection_sub_;

  //publishers
  ros::Publisher max_speed_pub_;

  float distance_to_event_;
  bool event_detected_;
  float event_distance_to_end_;
  std::chrono::steady_clock::time_point last_event_time_;

  float starting_line_distance_to_end_;
  float intersection_distance_to_end_;

  float max_speed_;
  int last_feedback_ {AUTONOMOUS_DRIVE};

public:
  FreeDriveAction(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~FreeDriveAction(void);

  inline void publishFeedback(feedback_variable program_state);
  inline void maxSpeedPub();

  void registerGoal();
  void executeLoop();
  void preemptCB();
  void startingLineCB(const std_msgs::Float32 &msg);
  void intersectionCB(const std_msgs::Float32 &msg);
};
#endif // FREE_DRIVE_ACTION_H
