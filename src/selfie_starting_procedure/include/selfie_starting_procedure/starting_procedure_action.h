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
  actionlib::SimpleActionServer<selfie_msgs::startingAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;

  // create messages that are used to published feedback/result
  selfie_msgs::startingFeedback feedback_;
  selfie_msgs::startingResult result_;

  // subscribers
  ros::Subscriber button_sub_;
  ros::Subscriber distance_sub_;

  // publishers
  ros::Publisher drive_pub_;

  int button_status_ = 0;
  float covered_distance_ = 0.0;
  float distance_base_;

public:

  StartingProcedureAction(std::string name);
  ~StartingProcedureAction(void);

  void publishFeedback(feedback_variable program_state);

  void driveBoxOut(float speed);
  void executeCB(const selfie_msgs::startingGoalConstPtr &goal);
  void buttonCB(const std_msgs::BoolConstPtr &msg);
  void distanceCB(const std_msgs::Float32ConstPtr &msg);
};
#endif // STARTING_PROCEDURE_ACTION_H
