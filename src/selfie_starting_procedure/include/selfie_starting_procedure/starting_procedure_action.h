#ifndef STARTING_PROCEDURE_ACTION_H
#define STARTING_PROCEDURE_ACTION_H

#include <ros/ros.h>
#include <selfie_msgs/startingAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <selfie_scheduler/scheduler_enums.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class StartingProcedureAction
{
protected:

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  actionlib::SimpleActionServer<selfie_msgs::startingAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.

  //params
  float starting_speed_;

  //create messages that are used to published feedback/result
  selfie_msgs::startingGoal goal_;
  selfie_msgs::startingFeedback feedback_;
  selfie_msgs::startingResult result_;

  //subscribers
  ros::Subscriber button1_sub_;
  ros::Subscriber button2_sub_;
  ros::Subscriber distance_sub_;

  //publishers
  ros::Publisher drive_pub_;

  int button_status_{SELFIE_IDLE};
  bool base_distance_initialized{false};
  float covered_distance_{0.0};
  float base_distance_{0.0};

public:

  StartingProcedureAction(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~StartingProcedureAction(void);

  void publishFeedback(feedback_variable program_state);

  void registerGoal();
  void executeLoop();
  void preemptCB();
  void driveBoxOut(float speed);
  void button1CB(const std_msgs::BoolConstPtr &msg);
  void button2CB(const std_msgs::BoolConstPtr &msg);
  void distanceCB(const std_msgs::Float32ConstPtr &msg);
};
#endif // STARTING_PROCEDURE_ACTION_H
