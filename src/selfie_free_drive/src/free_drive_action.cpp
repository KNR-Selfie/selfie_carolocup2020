/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/ 

#include <selfie_free_drive/free_drive_action.h>

FreeDriveAction::FreeDriveAction(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : 
  nh_(nh),
  pnh_(pnh),
  as_(nh_, "free_drive", false),
  d_to_starting_line_(100),
  d_to_intersection_(100),
  starting_line_distance_to_end_(0.45),
  intersection_distance_to_end_(0.8),
  starting_line_detected_(false),
  intersection_detected_(false),
  max_speed_(2)
{
  as_.registerGoalCallback(boost::bind(&FreeDriveAction::executeCB, this));
  as_.registerPreemptCallback(boost::bind(&FreeDriveAction::preemptCB, this));
  as_.start();
  max_speed_pub_ = nh_.advertise<std_msgs::Float64>("max_speed", 1);
  pnh_.getParam("max_speed", max_speed_);
  pnh_.getParam("starting_line_distance_to_end", starting_line_distance_to_end_);
  pnh_.getParam("intersection_distance_to_end", intersection_distance_to_end_);

  ROS_INFO("max_speed_: %.3f", max_speed_);
  ROS_INFO("starting_line_distance_to_end: %.3f", starting_line_distance_to_end_);
  ROS_INFO("intersection_distance_to_end: %.3f\n", intersection_distance_to_end_);
  ROS_INFO("Free drive initialized initialized");
}
FreeDriveAction::~FreeDriveAction(void)
{
  ROS_INFO("free_drive dead");
}

void FreeDriveAction::executeCB()
{
  goal_ = *(as_.acceptNewGoal());
  ROS_INFO("mode 0 - no obstacles");
  ROS_INFO("mode 1 - obstacles");
  ROS_INFO("received goal: mode %d", goal_.mode);
  publishFeedback(AUTONOMOUS_DRIVE);

  d_to_starting_line_ = 100;
  d_to_intersection_ = 100;
  ros::Rate loop_rate(50);

  if (goal_.mode == 0)
  {
    starting_line_sub_ = nh_.subscribe("/starting_line", 100, &FreeDriveAction::startingLineCB, this);
    ROS_INFO("starting_line_sub_");
    while (!(d_to_starting_line_ < starting_line_distance_to_end_))
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

      maxSpeedPub();
      loop_rate.sleep();

      if(!as_.isActive())
          return;
    }
    starting_line_sub_.shutdown();

    //publish result
    ROS_INFO("PARKING AREA");
    d_to_starting_line_ = 100;
    result_.event = false;
    as_.setSucceeded(result_);
  }
  else
  {
    ROS_INFO("intersection_sub_");
    intersection_sub_ = nh_.subscribe("/intersection_distance", 100, &FreeDriveAction::intersectionCB, this);
    while (!(d_to_intersection_ < intersection_distance_to_end_))
    {
      if (starting_line_detected_)
      {
        if (last_feedback_ != DETECT_CROSSROAD)
        {
          publishFeedback(DETECT_CROSSROAD);
          last_feedback_ = DETECT_CROSSROAD;
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

      //maxSpeedPub();
      loop_rate.sleep();

      if(!as_.isActive())
          return;
    }
    intersection_sub_.shutdown();

    //publish result
    ROS_INFO("INTERSECTION");
    d_to_intersection_ = 100;
    result_.event = true;
    as_.setSucceeded(result_);
  }
}

void FreeDriveAction::startingLineCB(const std_msgs::Float32ConstPtr &msg)
{
  ROS_INFO("ELO");
  starting_line_detected_ = true;
  d_to_starting_line_ = msg->data;
  ROS_INFO("ELO");
}

void FreeDriveAction::intersectionCB(const std_msgs::Float32 &msg)
{
    ROS_INFO("ELO2");
  intersection_detected_ = true;
  d_to_intersection_ = msg.data;
  ROS_INFO("ELO2");
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
