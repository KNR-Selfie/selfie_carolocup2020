/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/ 

#include <selfie_free_drive/free_drive_server.h>

FreeDriveServer::FreeDriveServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : 
  nh_(nh),
  pnh_(pnh),
  as_(nh_, "free_drive", false),
  distance_to_event_(100),
  starting_line_distance_to_end_(0.45),
  intersection_distance_to_end_(0.8),
  event_detected_(false),
  max_speed_(2),
  dr_server_CB_(boost::bind(&FreeDriveServer::reconfigureCB, this, _1, _2))
{
  as_.registerGoalCallback(boost::bind(&FreeDriveServer::registerGoal, this));
  as_.registerPreemptCallback(boost::bind(&FreeDriveServer::preemptCB, this));
  as_.start();
  max_speed_pub_ = nh_.advertise<std_msgs::Float64>("max_speed", 1);
  dr_server_.setCallback(dr_server_CB_);
  pnh_.getParam("max_speed", max_speed_);
  pnh_.getParam("starting_line_distance_to_end", starting_line_distance_to_end_);
  pnh_.getParam("intersection_distance_to_end", intersection_distance_to_end_);
  pnh_.getParam("distance_to_verify_event", distance_to_verify_event_);

  ROS_INFO("max_speed_: %.3f", max_speed_);
  ROS_INFO("starting_line_distance_to_end: %.3f", starting_line_distance_to_end_);
  ROS_INFO("intersection_distance_to_end: %.3f", intersection_distance_to_end_);
  ROS_INFO("distance_to_verify_event: %.3f\n", distance_to_verify_event_);
  ROS_INFO("Free drive initialized initialized");
  last_event_time_ = std::chrono::steady_clock::now();
}
FreeDriveServer::~FreeDriveServer(void)
{
  ROS_INFO("free_drive dead");
}

void FreeDriveServer::registerGoal()
{
  goal_ = *(as_.acceptNewGoal());
  ROS_INFO("mode 0 - no obstacles");
  ROS_INFO("mode 1 - obstacles");
  ROS_INFO("received goal: mode %d", goal_.mode);
  publishFeedback(AUTONOMOUS_DRIVE);

  distance_to_event_ = 100;
  if (!event_verified_)
    distance_sub_ = nh_.subscribe("/distance", 50, &FreeDriveServer::distanceCB, this);

  if (goal_.mode == 0)
  {
    event_distance_to_end_ = starting_line_distance_to_end_;
    starting_line_sub_ = nh_.subscribe("/starting_line", 100, &FreeDriveServer::startingLineCB, this);
  }
  else
  {
    event_distance_to_end_ = intersection_distance_to_end_;
    intersection_sub_ = nh_.subscribe("/intersection_distance", 100, &FreeDriveServer::intersectionCB, this);
  }
  executeLoop();
}

void FreeDriveServer::executeLoop()
{
  ros::Rate loop_rate(50);

  while (!(distance_to_event_ < event_distance_to_end_) && ros::ok())
  {
    if (event_detected_ && last_feedback_ == AUTONOMOUS_DRIVE)
    {
      if (goal_.mode == 0)
      {
        publishFeedback(DETECT_START_LINE);
        last_feedback_ = DETECT_START_LINE;
      }
      else
      {
        publishFeedback(DETECT_CROSSROAD);
        last_feedback_ = DETECT_CROSSROAD;
      }
      event_detected_ = false;
    }
    else
    {
      if (last_feedback_ != AUTONOMOUS_DRIVE &&
          std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - last_event_time_) > std::chrono::seconds(1))
      {
        publishFeedback(AUTONOMOUS_DRIVE);
        last_feedback_ = AUTONOMOUS_DRIVE;
        event_detected_ = false;
      }
    }

    if (goal_.mode == 0)
    {
      maxSpeedPub();
    }

    if(!as_.isActive())
    {
      starting_line_sub_.shutdown();
      intersection_sub_.shutdown();
      return;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  //publish result
  if (goal_.mode == 0)
  {
    ROS_INFO("PARKING AREA DETECTED");
    result_.event = false;
    as_.setSucceeded(result_);
    starting_line_sub_.shutdown();
  }
  else
  {
    ROS_INFO("INTERSECTION DETECTED");
    result_.event = true;
    as_.setSucceeded(result_);
    intersection_sub_.shutdown();
  }
  distance_sub_ = nh_.subscribe("/distance", 50, &FreeDriveServer::distanceCB, this);
}

void FreeDriveServer::startingLineCB(const std_msgs::Float32 &msg)
{
  event_detected_ = true;
  if (event_verified_)
    distance_to_event_ = msg.data;
  last_event_time_ = std::chrono::steady_clock::now();
}

void FreeDriveServer::intersectionCB(const std_msgs::Float32 &msg)
{
  event_detected_ = true;
  if (event_verified_)
    distance_to_event_ = msg.data;
  last_event_time_ = std::chrono::steady_clock::now();
}

inline void FreeDriveServer::publishFeedback(feedback_variable program_state)
{
  feedback_.action_status = program_state;
  as_.publishFeedback(feedback_);
}

inline void FreeDriveServer::maxSpeedPub()
{
  std_msgs::Float64 msg;
  msg.data = max_speed_;
  max_speed_pub_.publish(msg);
}

void FreeDriveServer::preemptCB()
{
  ROS_INFO("free_drive: Preempted");
  distance_sub_.shutdown();
  starting_line_sub_.shutdown();
  intersection_sub_.shutdown();
  event_verified_ = true;
  as_.setAborted();
}

void FreeDriveServer::reconfigureCB(selfie_free_drive::FreeDriveConfig& config, uint32_t level)
{
  if(intersection_distance_to_end_ != config.intersection_distance_to_end)
  {
    intersection_distance_to_end_ = config.intersection_distance_to_end;
    ROS_INFO("Intersection distance to end new value: %f", intersection_distance_to_end_);
  }
  if(intersection_distance_to_end_ != config.max_speed)
  {
    max_speed_ = config.max_speed;
    ROS_INFO("Max speed new value: %f", max_speed_);
  }
  if(starting_line_distance_to_end_ != config.starting_line_distance_to_end)
  {
    starting_line_distance_to_end_ = config.starting_line_distance_to_end;
    ROS_INFO("Starting line distance to end new value %f", starting_line_distance_to_end_);
  }
  if(distance_to_verify_event_ != config.distance_to_verify_event_)
  {
    distance_to_verify_event_ = config.distance_to_verify_event_;
    ROS_INFO("Distance to verify event new value %f", distance_to_verify_event_);
  }
}

void FreeDriveServer::distanceCB(const std_msgs::Float32 &msg)
{
  if (event_verified_)
  {
    distance_on_last_event_ = msg.data;
    event_verified_ = false;
    distance_to_event_ = 100;
    ROS_INFO("free_drive: last event distance saved");
    distance_sub_.shutdown();
  }
  else
  {
    if (msg.data - distance_on_last_event_ > distance_to_verify_event_)
    {
      event_verified_ = true;
      distance_to_event_ = 100;
      ROS_INFO("free_drive: event verified");
      distance_sub_.shutdown();
    }
  }
}
