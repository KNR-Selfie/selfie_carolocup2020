#include <selfie_scheduler/scheduler.h>
#include <selfie_scheduler/starting_action_client.h>
#include <selfie_scheduler/drive_action_client.h>
#include <selfie_scheduler/search_action_client.h>
#include <selfie_scheduler/park_action_client.h>
#include <selfie_scheduler/intersection_action_client.h>
#include <std_srvs/Empty.h>

Scheduler::Scheduler() :
    pnh_("~"),
    begin_action_(STARTING),
    start_distance_(1.0),
    parking_spot_(0.6),
    parking_steering_mode_(ACKERMANN),
    drive_steering_mode_(ACKERMANN)
{
    pnh_.getParam("begin_action", begin_action_);
    pnh_.getParam("starting_distance", start_distance_);
    pnh_.getParam("parking_spot", parking_spot_);
    pnh_.getParam("parking_steering_mode", parking_steering_mode_);
    pnh_.getParam("drive_steering_mode", drive_steering_mode_);

    ROS_INFO("Created scheduler with params: BA: %d, SD: %f, PS: %f", begin_action_, start_distance_, parking_spot_);

    visionReset_ = nh_.serviceClient<std_srvs::Empty>("resetVision");
    cmdCreatorStartPub_ = nh_.serviceClient<std_srvs::Empty>("cmd_start_pub");
    cmdCreatorStopPub_ = nh_.serviceClient<std_srvs::Empty>("cmd_stop_pub");
    avoidingObstSetPassive_ = nh_.serviceClient<std_srvs::Empty>("avoiding_obst_set_passive");
    avoidingObstSetActive_ = nh_.serviceClient<std_srvs::Empty>("avoiding_obst_set_active");
    resetLaneController_ = nh_.serviceClient<std_srvs::Empty>("resetLaneControl");
    steeringModeSetAckermann_ = nh_.serviceClient<std_srvs::Empty>("steering_ackerman");
    steeringModeSetParallel_ = nh_.serviceClient<std_srvs::Empty>("steering_parallel");
    steeringModeSetFrontAxis_ = nh_.serviceClient<std_srvs::Empty>("steering_front_axis");
    switchState_ = nh_.subscribe("switch_state", 10, &Scheduler::switchStateCallback, this);

    clients_[STARTING] = new StartingProcedureClient("starting_procedure");
    action_args_[STARTING] = start_distance_;

    clients_[DRIVING] = new DriveClient("free_drive");
    action_args_[DRIVING] = [](bool x){return x;}(false);

    previousRcState_ = RC_UNINTIALIZED;
    currentRcState_ = RC_UNINTIALIZED;
    previous_car_state_= SELFIE_IDLE;
    current_car_state_ = SELFIE_READY;

    ROS_INFO("Clients created successfully");
}
Scheduler::~Scheduler()
{
    pnh_.deleteParam("begin_action");
}
void Scheduler::waitForStart()
{
    while(ros::ok)
    {
        if(checkIfActionFinished() == SUCCESS)
        {
            boost::any result;
            current_client_ptr_->getActionResult(result);

            DriveClient* drive_client = dynamic_cast<DriveClient*>(clients_[DRIVING]);
            bool drive_mode = boost::any_cast<bool>(result);

            drive_client->setDriveMode(drive_mode);
            setupActionClients(drive_mode);
            break;
        }
        else if(checkIfActionFinished()==ABORTED)
            break;
    }
}
void Scheduler::setupActionClients(bool button_pressed)
{
    if(button_pressed == 0) // parking mode
    {
        clients_[PARKING_SEARCH] = new SearchClient("search");
        action_args_[PARKING_SEARCH] = parking_spot_;

        clients_[PARK] = new ParkClient("park");
        action_args_[PARK] = [](geometry_msgs::Polygon x){return x;};
    }
    else // intersection mode
    {
        clients_[INTERSECTION] = new IntersectionClient("intersection");
        action_args_[INTERSECTION] = (int)0; // empty goal
        setAvoidingObstActive();
    }
}
void Scheduler::init()
{
    startAction((action)begin_action_);

    switch(begin_action_)
    {
        case(DRIVING):
            startCmdCreator();
            break;
        case(PARK):
            stopCmdCreator();
            break;
    }
}
void Scheduler::startAction(action action_to_set)
{
    current_client_ptr_ = clients_[action_to_set];
    current_client_ptr_->waitForServer(200);
    current_client_ptr_->setGoal(action_args_[action_to_set]);
}
void Scheduler::startNextAction()
{
    action next_action = current_client_ptr_->getNextAction();
    current_client_ptr_ = clients_[next_action];
    current_client_ptr_->waitForServer(200);
    current_client_ptr_->setGoal(action_args_[next_action]);
}
int Scheduler::checkIfActionFinished()
{
    return current_client_ptr_->getClientGoalState();
}
void Scheduler::loop()
{
    if (checkIfActionFinished() == SUCCESS)
    {
        current_client_ptr_->getActionResult(action_args_[current_client_ptr_->getNextAction()]);
        shiftAction();
    }
    else if(checkIfActionFinished() == ABORTED)
    {
        // abort caused by RC
        if(currentRcState_ == RC_MANUAL)
        {
            // empty state
        }
        else // abort caused by server
        {
            stopAction();
            resetVision();
            startAction(DRIVING);
            startCmdCreator();
        }
    }
    stateMachine();
}
void Scheduler::setAvoidingObstActive()
{
    std_srvs::Empty empty_msg;
    avoidingObstSetActive_.call(empty_msg);
}
void Scheduler::setAvoidingObstPassive()
{
    std_srvs::Empty empty_msg;
    avoidingObstSetPassive_.call(empty_msg);
}
void Scheduler::resetLaneControl()
{
    std_srvs::Empty empty_msg;
    resetLaneController_.call(empty_msg);
}
void Scheduler::resetVision()
{
    std_srvs::Empty empty_msg;
    visionReset_.call(empty_msg);
}
void Scheduler::startCmdCreator()
{
    std_srvs::Empty empty_msg;
    cmdCreatorStartPub_.call(empty_msg);
}
void Scheduler::stopCmdCreator()
{
    std_srvs::Empty empty_msg;
    cmdCreatorStopPub_.call(empty_msg);
}
void Scheduler::setParkSteeringMode()
{   
    std_srvs::Empty empty_msg;
    if(parking_steering_mode_)
        steeringModeSetParallel_.call(empty_msg);
    else
        steeringModeSetAckermann_.call(empty_msg);
}
void Scheduler::setDriveSteeringMode()
{   
    std_srvs::Empty empty_msg;
    if(drive_steering_mode_ == PARALLEL)
        steeringModeSetParallel_.call(empty_msg);
    else if(drive_steering_mode_ == ACKERMANN)
        steeringModeSetAckermann_.call(empty_msg);
    else if(drive_steering_mode_ == FRONT_AXIS)
        steeringModeSetFrontAxis_.call(empty_msg);
}
template <typename T>
bool Scheduler::checkCurrentClientType()
{
    ClientInterface *check = dynamic_cast<T> (current_client_ptr_);
    if (check)
    {
        return true;
    }
    return false;
}

void Scheduler::shiftAction()
{
    if (checkCurrentClientType<StartingProcedureClient*>())
    {
        resetVision();
        startAction(DRIVING);
        startCmdCreator();
        setDriveSteeringMode();
    }
    else if (checkCurrentClientType<DriveClient*>())
    {
        setAvoidingObstPassive();
        startNextAction();
    }
    else if (checkCurrentClientType<SearchClient*>())
    {
        stopCmdCreator();
        startAction(PARK);
        setParkSteeringMode();
    }
    else if (checkCurrentClientType<ParkClient*>())
    {
        resetVision();
        startAction(DRIVING);
        startCmdCreator();
        setDriveSteeringMode();
    }
    else if (checkCurrentClientType<IntersectionClient*>())
    {
        setAvoidingObstActive();
        startNextAction();
        setDriveSteeringMode();
    }
    else
    {
        ROS_WARN("No such action client!!");
    }
}
void Scheduler::stateMachine()
{
    current_car_state_ = current_client_ptr_->getActionState();

    if (current_car_state_ == previous_car_state_)
    {
        return;
    }
    switch (current_car_state_)
    {
        case SELFIE_READY:
            ROS_INFO("STATE_SELFIE_READY");
            previous_car_state_ = SELFIE_READY;
            break;
        case BUTTON_PARKING_DRIVE_PRESSED:
            ROS_INFO("BUTTON_PARKING_DRIVE_PRESSED");
            previous_car_state_ = BUTTON_PARKING_DRIVE_PRESSED;
            break;
        case BUTTON_OBSTACLE_DRIVE_PRESSED:
            ROS_INFO("BUTTON_OBSTACLE_DRIVE_PRESSED");
            previous_car_state_ = BUTTON_OBSTACLE_DRIVE_PRESSED;
            break;
        case START_DRIVE:
            ROS_INFO("START_DRIVE");
            previous_car_state_ = START_DRIVE;
            break;
        case END_DRIVE:
            ROS_INFO("END DRIVE");
            previous_car_state_ = END_DRIVE;
            break;
    }
}
void Scheduler::stopAction()
{
    current_client_ptr_->cancelAction();
    ROS_WARN("STOP current action");
}
void Scheduler::switchStateCallback(const std_msgs::UInt8ConstPtr &msg)
{
    // prevent from execution on the beginning
    if (current_car_state_ > SELFIE_READY)
    {
        currentRcState_ = (rc_state)msg->data;

        if (previousRcState_ != currentRcState_)
        {
            if (currentRcState_ == RC_MANUAL && (previousRcState_ == RC_AUTONOMOUS || previousRcState_ == RC_HALF_AUTONOMOUS))
            {
                stopAction();
            }
            else if(currentRcState_ == RC_AUTONOMOUS && previousRcState_ == RC_MANUAL)
            {
                resetVision();
                startAction(DRIVING);
                setDriveSteeringMode();
                startCmdCreator();
                resetLaneControl();
            }
            else if(currentRcState_ == RC_HALF_AUTONOMOUS && previousRcState_ == RC_MANUAL)
            {
                resetVision();
                startAction(DRIVING);
                setDriveSteeringMode();
                startCmdCreator();
                resetLaneControl();
            }
            previousRcState_ = currentRcState_;
        } 
    }
}

