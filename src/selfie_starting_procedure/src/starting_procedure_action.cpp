#include <selfie_starting_procedure/starting_procedure_action.h>

StartingProcedureAction::StartingProcedureAction(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
  nh_(nh),
  pnh_(pnh),
  as_(nh_, "starting_procedure", false),
  starting_speed_(2.0)
{
  as_.registerGoalCallback(boost::bind(&StartingProcedureAction::registerGoal, this));
  as_.registerPreemptCallback(boost::bind(&StartingProcedureAction::preemptCB, this));

  pnh_.getParam("starting_speed", starting_speed_);
  ROS_INFO("starting_speed: %.3f", starting_speed_);

  drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 1);

  as_.start();
  
  ROS_INFO("Starting procedure object created");
}
StartingProcedureAction::~StartingProcedureAction(void)
{ 
  button1_sub_.shutdown();
  button2_sub_.shutdown();
  distance_sub_.shutdown();
}

void StartingProcedureAction::registerGoal()
{
  goal_ = *(as_.acceptNewGoal());
  button_status_ = SELFIE_READY;
  ROS_INFO("GOAL RECEIVED");
  ROS_INFO("goal_distance: %f", goal_.distance);
  publishFeedback(SELFIE_READY);

  // reset variables
  base_distance_initialized = false;
  covered_distance_ = 0.0;
  base_distance_ = 0.0;

  button1_sub_ = nh_.subscribe("start_button1", 100, &StartingProcedureAction::button1CB, this);
  button2_sub_ = nh_.subscribe("start_button2", 100, &StartingProcedureAction::button2CB, this);
  distance_sub_ = nh_.subscribe("distance", 100, &StartingProcedureAction::distanceCB, this);

  executeLoop();
}

void StartingProcedureAction::executeLoop()
{
  ros::Rate loop_rate(50);
  publishFeedback(SELFIE_READY);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

    if (!(button_status_ == BUTTON_PARKING_DRIVE_PRESSED || button_status_ == BUTTON_OBSTACLE_DRIVE_PRESSED))
    {
      continue;
    }
    
    if (!base_distance_initialized)
    {
      continue;
    }

    if (covered_distance_ < goal_.distance)
    {
      driveBoxOut(starting_speed_);
    }
    else
    {
      break;
    }
  }
  distance_sub_.shutdown();
  driveBoxOut(0.0);
  ROS_INFO("DISTANCE COVERED");
  publishFeedback(END_DRIVE);
  if (button_status_ == BUTTON_PARKING_DRIVE_PRESSED)
  {
    result_.drive_mode = false;
  }
  else
  {
    result_.drive_mode = true;
  }
  
  // publish result
  as_.setSucceeded(result_);
}
void StartingProcedureAction::button1CB(const std_msgs::BoolConstPtr &msg)
{
  if (msg->data == true)
  {
    button_status_ = BUTTON_PARKING_DRIVE_PRESSED;
    ROS_INFO("Button1 pressed");
    publishFeedback(BUTTON_PARKING_DRIVE_PRESSED);
    button1_sub_.shutdown();
    button2_sub_.shutdown();
  }
}

void StartingProcedureAction::button2CB(const std_msgs::BoolConstPtr &msg)
{
  if (msg->data == true)
  {
    button_status_ = BUTTON_OBSTACLE_DRIVE_PRESSED;
    ROS_INFO("Button2 pressed");
    publishFeedback(BUTTON_OBSTACLE_DRIVE_PRESSED);
    button1_sub_.shutdown();
    button2_sub_.shutdown();
  }
}

void StartingProcedureAction::preemptCB()
{
  ROS_INFO("STARTING PROCEDURE ABORTED");
  button1_sub_.shutdown();
  button2_sub_.shutdown();
  distance_sub_.shutdown();
  as_.setAborted();
}

void StartingProcedureAction::distanceCB(const std_msgs::Float32ConstPtr &msg)
{
  if (!base_distance_initialized)
  {
    if (button_status_ == BUTTON_PARKING_DRIVE_PRESSED || button_status_ == BUTTON_OBSTACLE_DRIVE_PRESSED)
    {
      base_distance_ = msg->data;
      base_distance_initialized = true;
      ROS_INFO("CAR START MOVE");
      publishFeedback(START_DRIVE);
    }
  }
  else
  {
    covered_distance_ = msg->data - base_distance_;
  }
}

void StartingProcedureAction::publishFeedback(feedback_variable program_state)
{
  feedback_.action_status = program_state;
  as_.publishFeedback(feedback_);
}

void StartingProcedureAction::driveBoxOut(float speed)
{
  ackermann_msgs::AckermannDriveStamped cmd;
  cmd.drive.speed = speed;
  cmd.drive.steering_angle = 0;
  cmd.drive.steering_angle_velocity = 15;
  drive_pub_.publish(cmd);
}
