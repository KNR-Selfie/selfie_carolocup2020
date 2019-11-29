#include <selfie_starting_procedure/starting_procedure_action.h>

StartingProcedureAction::StartingProcedureAction(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
  nh_(nh),pnh_(pnh), as_(nh_, "starting_procedure",  false),
  minSecondPressTime_(ros::Time(0)),debounceDuration_(ros::Duration(2)),
  distanceGoal_(0.f),distanceRead_(0.f)
{
  as_.registerPreemptCallback(boost::bind(&StartingProcedureAction::preemptCB, this));
  as_.registerGoalCallback(boost::bind(&StartingProcedureAction::executeCB, this));

  parking_button_sub_ = nh_.subscribe("start_button1", 10, &StartingProcedureAction::parking_buttonCB, this);
  obstacle_button_sub_ = nh_.subscribe("start_button2", 10, &StartingProcedureAction::obstacle_buttonCB, this);
  qrClient_ = nh_.serviceClient<std_srvs::Empty>("startQrSearch");
  distance_sub_ = nh_.subscribe("distance",10,&StartingProcedureAction::distanceCB, this);
  drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1);
  pnh_.param<float>("starting_speed", starting_speed_,2.f);
  pnh_.param("use_scan",use_scan_,false);
  pnh_.param("use_qr",use_qr_,true);
  if(use_qr_) {qrSub_ = nh_.subscribe("qr_gate_open",1,&StartingProcedureAction::gateOpenCB,this);}
  ROS_INFO("starting_speed: %.3f", starting_speed_);

  as_.start();
  ROS_INFO("Starting procedure object created");
}
StartingProcedureAction::~StartingProcedureAction(void)
{ 
  obstacle_button_sub_.shutdown();
  parking_button_sub_.shutdown();
  distance_sub_.shutdown();
}

void StartingProcedureAction::executeCB()
{
  selfie_msgs::startingGoal goal = *as_.acceptNewGoal();
  distanceGoal_ = goal.distance;
  ROS_INFO("received goal %f",goal.distance);
  publishFeedback(SELFIE_READY);
  state_ = State::WAIT_BUTTON;
}
void StartingProcedureAction::parking_buttonCB(const std_msgs::Empty &msg)
{
  if(state_ == State::WAIT_BUTTON)
  {
    minSecondPressTime_ = ros::Time::now() + debounceDuration_;
    button_status_ = BUTTON_PARKING_DRIVE_PRESSED;
    ROS_INFO("parking button pressed 1");
    if(use_qr_)
    {
      ROS_INFO("start qr search");
      std_srvs::Empty call = std_srvs::Empty();
      qrClient_.call(call);
    }
    state_ = State::WAIT_START;
  }
  else if(state_ == State::WAIT_START)
  {
    if(ros::Time::now() > minSecondPressTime_)
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

void StartingProcedureAction::obstacle_buttonCB(const std_msgs::Empty &msg)
{
  if(state_ == State::WAIT_BUTTON)
  {
    minSecondPressTime_ = ros::Time::now() + debounceDuration_;
    button_status_ = BUTTON_OBSTACLE_DRIVE_PRESSED;
    ROS_INFO("obstacle button pressed 1");
    if(use_qr_)
    {
      ROS_INFO("start qr search");
      std_srvs::Empty call = std_srvs::Empty();
      qrClient_.call(call);
    }
    state_ = State::WAIT_START;
  }
  else if(state_ == State::WAIT_START)
  {
    if(ros::Time::now() > minSecondPressTime_)
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
  if(state_ == State::WAIT_START)
  {
    publishFeedback(button_status_);
    distanceGoal_ = distanceRead_ + distanceGoal_;
    ROS_INFO("gate was opened");
    startingDistance_ = distanceRead_;
    state_ = State::START_MOVE;
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
    distanceRead_ = msg->data;
    if(state_ == State::START_MOVE )
    {
        driveBoxOut(starting_speed_);
        if(distanceRead_ > distanceGoal_)
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
