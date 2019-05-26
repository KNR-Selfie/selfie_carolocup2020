#ifndef STARTING_PROCEDURE_ACTION_H
#define STARTING_PROCEDURE_ACTION_H

#include <ros/ros.h>
#include <selfie_msgs/startingAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

class StartingProcedureAction
{
protected:

  ros::NodeHandle nh;
  actionlib::SimpleActionServer<selfie_msgs::startingAction> as; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name;

  // create messages that are used to published feedback/result
  selfie_msgs::startingFeedback feedback;
  selfie_msgs::startingResult result;

  //subscribers
  ros::Subscriber button_sub;
  ros::Subscriber distance_sub;

public:

  StartingProcedureAction(std::string name);
  ~StartingProcedureAction(void);
  void executeCB(const selfie_msgs::startingGoalConstPtr &goal);
  void buttonCB(const std_msgs::BoolConstPtr &msg);
  void distanceCB(const std_msgs::Float32ConstPtr &msg);
};
#endif // STARTING_PROCEDURE_ACTION_H
