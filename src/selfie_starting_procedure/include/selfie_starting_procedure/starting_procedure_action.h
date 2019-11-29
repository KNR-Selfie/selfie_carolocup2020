#ifndef STARTING_PROCEDURE_ACTION_H
#define STARTING_PROCEDURE_ACTION_H

#include <ros/ros.h>
#include <selfie_msgs/startingAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

#include <selfie_scheduler/scheduler_enums.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class StartingProcedureAction
{
protected:

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  actionlib::SimpleActionServer<selfie_msgs::startingAction> as_;

  //params
  float startingSpeed_;
  bool useScan_;
  bool useQr_;

  //create messages that are used to published feedback/result
  selfie_msgs::startingGoal goal_;
  selfie_msgs::startingFeedback feedback_;
  selfie_msgs::startingResult result_;

  //subscribers
  ros::Subscriber parkingButtonSub_;
  ros::Subscriber obstacleButtonSub_;
  ros::Subscriber distanceSub_;
  ros::Subscriber qrSub_;
  ros::Subscriber gateScanSub_;
  ros::ServiceClient qrClient_;
  ros::ServiceClient scanClient_;
  //publishers
  ros::Publisher drivePub_;

  feedback_variable buttonStatus_;
private:
  enum class State
  {
    IDLE,
    WAIT_BUTTON,
    WAIT_START,
    START_MOVE,
    END_MOVE
  } state_;

  float distanceGoal_;
  float startingDistance_;
  float distanceRead_;

  ros::Time minSecondPressTime_;
  ros::Duration debounceDuration_;

  void publishFeedback(feedback_variable program_state);

  void executeCB();
  void preemptCB();
  void driveBoxOut(float speed);
  void parkingButtonCB(const std_msgs::Empty &msg);
  void obstacleButtonCB(const std_msgs::Empty &msg);
  void distanceCB(const std_msgs::Float32ConstPtr &msg);
  void gateOpenCB(const std_msgs::Empty &msg);


public:

  StartingProcedureAction(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

};
#endif // STARTING_PROCEDURE_ACTION_H
