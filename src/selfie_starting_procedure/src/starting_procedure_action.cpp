#include <selfie_starting_procedure/starting_procedure_action.h>

StartingProcedureAction::StartingProcedureAction(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
  nh_(nh), pnh_(pnh), as_(nh_, "starting_procedure",  false),
  min_second_press_time_(ros::Time(0)), debounce_duration_(ros::Duration(2)),
  distance_goal_(0.f), distance_read_(0.f)
{
  as_.registerPreemptCallback(boost::bind(&StartingProcedureAction::preemptCB, this));
  as_.registerGoalCallback(boost::bind(&StartingProcedureAction::executeCB, this));

  parking_button_sub_ = nh_.subscribe("start_button1", 10, &StartingProcedureAction::parkingButtonCB, this);
  obstacle_button_sub_ = nh_.subscribe("start_button2", 10, &StartingProcedureAction::obstacleButtonCB, this);
  qr_client_ = nh_.serviceClient<std_srvs::Empty>("startQrSearch");
  scan_client_ = nh_.serviceClient<std_srvs::Empty>("startGateScan");
  distance_sub_ = nh_.subscribe("distance", 10, &StartingProcedureAction::distanceCB, this);
  drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 1);
  pnh_.param<float>("starting_speed", starting_speed_, 2.f);
  pnh_.param<bool>("use_scan", use_scan_, false);
  pnh_.param<bool>("use_qr", use_qr_, true);
  if (use_qr_) {qr_sub_ = nh_.subscribe("qr_gate_open", 1, &StartingProcedureAction::gateOpenCB, this);}
  if (use_scan_) {gate_scan_sub_ = nh_.subscribe("scan_gate_open", 1, &StartingProcedureAction::gateOpenCB, this);}

  ROS_INFO("Starting_speed: %.3f", starting_speed_);
  as_.start();
  ROS_INFO("Starting procedure object created");
}

void StartingProcedureAction::executeCB()
{
  selfie_msgs::startingGoal goal = *as_.acceptNewGoal();
  distance_goal_ = goal.distance;
  ROS_INFO("received goal %f", goal.distance);
  publishFeedback(SELFIE_READY);
  state_ = State::WAIT_BUTTON;
}

void StartingProcedureAction::parkingButtonCB(const std_msgs::Empty &msg)
{
  if (state_ == State::WAIT_BUTTON)
  {
    min_second_press_time_ = ros::Time::now() + debounce_duration_;
    button_status_ = BUTTON_PARKING_DRIVE_PRESSED;
    ROS_INFO("parking button pressed 1");
    if (use_qr_)
    {
      ROS_INFO("start qr search");
      std_srvs::Empty call = std_srvs::Empty();
      qr_client_.call(call);
    }
    if (use_scan_)
    {
      ROS_INFO("start scan search");
      std_srvs::Empty call = std_srvs::Empty();
      scan_client_.call(call);
    }
    state_ = State::WAIT_START;
  }
  else if (state_ == State::WAIT_START)
  {
    if( ros::Time::now() > min_second_press_time_)
    {
        publishFeedback(BUTTON_PARKING_DRIVE_PRESSED);
        distance_goal_ = distance_read_ + distance_goal_;
        starting_distance_ = distance_read_;
        ROS_INFO("start parking");
        state_ = State::START_MOVE;
        publishFeedback(START_DRIVE);
    }
  }
}

void StartingProcedureAction::obstacleButtonCB(const std_msgs::Empty &msg)
{
  if (state_ == State::WAIT_BUTTON)
  {
    min_second_press_time_ = ros::Time::now() + debounce_duration_;
    button_status_ = BUTTON_OBSTACLE_DRIVE_PRESSED;
    ROS_INFO("obstacle button pressed 1");
    if (use_qr_)
    {
      ROS_INFO("start qr search");
      std_srvs::Empty call = std_srvs::Empty();
      qr_client_.call(call);
    }
    if (use_scan_)
    {
      ROS_INFO("start scan search");
      std_srvs::Empty call = std_srvs::Empty();
      scan_client_.call(call);
    }
    state_ = State::WAIT_START;
  }
  else if (state_ == State::WAIT_START)
  {
    if (ros::Time::now() > min_second_press_time_)
    {
        publishFeedback(BUTTON_OBSTACLE_DRIVE_PRESSED);
        distance_goal_ = distance_read_ + distance_goal_;
        starting_distance_ = distance_read_;
        ROS_INFO("start obstacle");
        state_ = State::START_MOVE;
        publishFeedback(START_DRIVE);
    }
  }
}

void StartingProcedureAction::gateOpenCB(const std_msgs::Empty &msg)
{
  if (state_ == State::WAIT_START)
  {
    state_ = State::START_MOVE;
    publishFeedback(button_status_);
    distance_goal_ = distance_read_ + distance_goal_;
    ROS_INFO("gate was opened");
    starting_distance_ = distance_read_;
    publishFeedback(START_DRIVE);
  }
}

void StartingProcedureAction::preemptCB()
{
  as_.setAborted();
  state_ = State::IDLE;
  ROS_INFO("STARTING PROCEDURE ABORTED");
  parking_button_sub_.shutdown();
  obstacle_button_sub_.shutdown();
  distance_sub_.shutdown();
  as_.setAborted();
}

void StartingProcedureAction::distanceCB(const std_msgs::Float32ConstPtr &msg)
{  
  distance_read_ = msg->data;
  if (state_ == State::START_MOVE )
  {
      driveBoxOut(starting_speed_);
      if (distance_read_ > distance_goal_)
      {
        state_ = State::IDLE;
        ROS_INFO("end start procedure");
        publishFeedback(END_DRIVE);
        as_.setSucceeded(result_);
      }
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
    drive_pub_.publish(cmd);
}
