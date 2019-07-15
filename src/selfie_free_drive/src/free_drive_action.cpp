/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/ 

#include <selfie_free_drive/free_drive_action.h>

FreeDriveAction::FreeDriveAction(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : 
  nh_(nh),
  pnh_(pnh),
  as_(nh_, "free_drive", boost::bind(&FreeDriveAction::executeCB, this, _1), false),
  d_to_starting_line_(100),
  starting_line_distance_to_end(0.45),
  starting_line_detected_(false),
  max_speed_(2)
{
  as_.registerPreemptCallback(boost::bind(&FreeDriveAction::preemptCB, this));
  as_.start();
  starting_line_sub_ = nh_.subscribe("starting_line", 1000, &FreeDriveAction::startingLineCB, this);
  max_speed_pub_ = nh_.advertise<std_msgs::Float64>("max_speed", 1);
  pnh_.getParam("max_speed", max_speed_);
  pnh_.getParam("starting_line_distance_to_end", starting_line_distance_to_end);

  ROS_INFO("max_speed_: %.3f", max_speed_);
  ROS_INFO("starting_line_distance_to_end: %.3f\n", starting_line_distance_to_end);
  ROS_INFO("Free drive initialized initialized");
}
FreeDriveAction::~FreeDriveAction(void)
{
  ROS_INFO("free_drive dead");
}

void FreeDriveAction::executeCB(const selfie_msgs::drivingGoalConstPtr &goal)
{
  ROS_INFO("mode 0 - no obstacles");
  ROS_INFO("mode 1 - obstacles");
  ROS_INFO("received goal: mode %d", goal->mode);
  publishFeedback(AUTONOMOUS_DRIVE);

  d_to_starting_line_ = 100;

  ros::Rate loop_rate(50);
  while (!(d_to_starting_line_ < starting_line_distance_to_end))
  {
    if (starting_line_detected_)
    {
      if (last_feedback_ != DETECT_START_LINE)
      {
        publishFeedback(DETECT_START_LINE);
        last_feedback_ = DETECT_START_LINE;
      }
      starting_line_detected_ = false;
    }
    else
    {
      if (last_feedback_ != AUTONOMOUS_DRIVE)
      {
        publishFeedback(AUTONOMOUS_DRIVE);
        last_feedback_ = AUTONOMOUS_DRIVE;
      }
    }
    else
      publishFeedback(AUTONOMOUS_DRIVE);
    maxSpeedPub();
    loop_rate.sleep();
  }

    maxSpeedPub();
    loop_rate.sleep();

    if(!as_.isActive())
        return;
  }

  //publish result
  ROS_INFO("PARKING AREA");
  d_to_starting_line_ = 100;
  result_.parking_area = true;
  as_.setSucceeded(result_);
}

void FreeDriveAction::startingLineCB(const std_msgs::Float32ConstPtr &msg)
{
  starting_line_detected_ = true;
  d_to_starting_line_ = msg->data;
}

void FreeDriveAction::publishFeedback(feedback_variable program_state)
{
  feedback_.action_status = program_state;
  as_.publishFeedback(feedback_);
}

void FreeDriveAction::maxSpeedPub()
{
  std_msgs::Float64 msg;
  msg.data = max_speed_;
  max_speed_pub_.publish(msg);
}
void FreeDriveAction::preemptCB()
{
    ROS_INFO("Preempted");
    as_.setAborted();
}
