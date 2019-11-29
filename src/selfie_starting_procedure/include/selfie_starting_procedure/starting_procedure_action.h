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
  float starting_speed_;
  bool use_scan_;
  bool use_qr_;

  //create messages that are used to published feedback/result
  selfie_msgs::startingGoal goal_;
  selfie_msgs::startingFeedback feedback_;
  selfie_msgs::startingResult result_;

  //subscribers
  ros::Subscriber parking_button_sub_;
  ros::Subscriber obstacle_button_sub_;
  ros::Subscriber distance_sub_;
  ros::Subscriber qr_sub_;
  ros::Subscriber gate_scan_sub_;
  ros::ServiceClient qr_client_;
  ros::ServiceClient scan_client_;
  //publishers
  ros::Publisher drive_pub_;

  feedback_variable button_status_;
private:
  enum class State
  {
    IDLE,
    WAIT_BUTTON,
    WAIT_START,
    START_MOVE,
    END_MOVE
  } state_;

  float distance_goal_;
  float starting_distance_;
  float distance_read_;

  ros::Time min_second_press_time_;
  ros::Duration debounce_duration_;

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
