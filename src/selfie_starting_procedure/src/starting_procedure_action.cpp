#include <selfie_starting_procedure/starting_procedure_action.h>

StartingProcedureAction::StartingProcedureAction(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
  nh_(nh), pnh_(pnh), as_(nh_, "starting_procedure",  false),
  minSecondPressTime_(ros::Time(0)), debounceDuration_(ros::Duration(2)),
  distanceGoal_(0.f), distanceRead_(0.f)
{
  as_.registerPreemptCallback(boost::bind(&StartingProcedureAction::preemptCB, this));
  as_.registerGoalCallback(boost::bind(&StartingProcedureAction::executeCB, this));

  parkingButtonSub_ = nh_.subscribe("start_button1", 10, &StartingProcedureAction::parkingButtonCB, this);
  obstacleButtonSub_ = nh_.subscribe("start_button2", 10, &StartingProcedureAction::obstacleButtonCB, this);
  qrClient_ = nh_.serviceClient<std_srvs::Empty>("startQrSearch");
  scanClient_ = nh_.serviceClient<std_srvs::Empty>("startGateScan");
  distanceSub_ = nh_.subscribe("distance", 10, &StartingProcedureAction::distanceCB, this);
  drivePub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 1);
  pnh_.param<float>("starting_speed", startingSpeed_, 2.f);
  pnh_.param<bool>("use_scan", useScan_, false);
  pnh_.param<bool>("use_qr", useQr_, true);
  if (useQr_) {qrSub_ = nh_.subscribe("qr_gate_open", 1, &StartingProcedureAction::gateOpenCB, this);}
  if (useScan_) {gateScanSub_ = nh_.subscribe("scan_gate_open", 1, &StartingProcedureAction::gateOpenCB, this);}

  ROS_INFO("Starting_speed: %.3f", startingSpeed_);
  as_.start();
  ROS_INFO("Starting procedure object created");
}

void StartingProcedureAction::executeCB()
{
  selfie_msgs::startingGoal goal = *as_.acceptNewGoal();
  distanceGoal_ = goal.distance;
  ROS_INFO("received goal %f", goal.distance);
  publishFeedback(SELFIE_READY);
  state_ = State::WAIT_BUTTON;
}

void StartingProcedureAction::parkingButtonCB(const std_msgs::Empty &msg)
{
  if (state_ == State::WAIT_BUTTON)
  {
    minSecondPressTime_ = ros::Time::now() + debounceDuration_;
    buttonStatus_ = BUTTON_PARKING_DRIVE_PRESSED;
    ROS_INFO("parking button pressed 1");
    if (useQr_)
    {
      ROS_INFO("start qr search");
      std_srvs::Empty call = std_srvs::Empty();
      qrClient_.call(call);
    }
    if (useScan_)
    {
      ROS_INFO("start scan search");
      std_srvs::Empty call = std_srvs::Empty();
      scanClient_.call(call);
    }
    state_ = State::WAIT_START;
  }
  else if (state_ == State::WAIT_START)
  {
    if( ros::Time::now() > minSecondPressTime_)
    {
        publishFeedback(BUTTON_PARKING_DRIVE_PRESSED);
        distanceGoal_ = distanceRead_ + distanceGoal_;
        startingDistance_ = distanceRead_;
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
    minSecondPressTime_ = ros::Time::now() + debounceDuration_;
    buttonStatus_ = BUTTON_OBSTACLE_DRIVE_PRESSED;
    ROS_INFO("obstacle button pressed 1");
    if (useQr_)
    {
      ROS_INFO("start qr search");
      std_srvs::Empty call = std_srvs::Empty();
      qrClient_.call(call);
    }
    if (useScan_)
    {
      ROS_INFO("start scan search");
      std_srvs::Empty call = std_srvs::Empty();
      scanClient_.call(call);
    }
    state_ = State::WAIT_START;
  }
  else if (state_ == State::WAIT_START)
  {
    if (ros::Time::now() > minSecondPressTime_)
    {
        publishFeedback(BUTTON_OBSTACLE_DRIVE_PRESSED);
        distanceGoal_ = distanceRead_ + distanceGoal_;
        startingDistance_ = distanceRead_;
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
    publishFeedback(buttonStatus_);
    distanceGoal_ = distanceRead_ + distanceGoal_;
    ROS_INFO("gate was opened");
    startingDistance_ = distanceRead_;
    publishFeedback(START_DRIVE);
  }
}

void StartingProcedureAction::preemptCB()
{
  as_.setAborted();
  state_ = State::IDLE;
  ROS_INFO("STARTING PROCEDURE ABORTED");
  parkingButtonSub_.shutdown();
  obstacleButtonSub_.shutdown();
  distanceSub_.shutdown();
  as_.setAborted();
}

void StartingProcedureAction::distanceCB(const std_msgs::Float32ConstPtr &msg)
{  
  distanceRead_ = msg->data;
  if (state_ == State::START_MOVE )
  {
      driveBoxOut(startingSpeed_);
      if (distanceRead_ > distanceGoal_)
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
    drivePub_.publish(cmd);
}
