#include <selfie_scheduler/scheduler.h>
#include <selfie_scheduler/starting_action_client.h>
#include <selfie_scheduler/drive_action_client.h>
#include <selfie_scheduler/search_action_client.h>
#include <selfie_scheduler/park_action_client.h>
#include <std_srvs/Empty.h>

Scheduler::Scheduler() :
    pnh_("~"),
    begin_action_(STARTING),
    start_distance_(1.0),
    parking_spot_(70.0)
{

    //ovveride parameters
    pnh_.getParam("begin_action", begin_action_);
    pnh_.getParam("starting_distance", start_distance_);
    pnh_.getParam("parking_spot", parking_spot_);

    ROS_INFO("Created scheduler with params: BA: %d, SD: %f, PS: %f",begin_action_,start_distance_,parking_spot_);

    visionReset_ = nh_.serviceClient<std_srvs::Empty>("resetVision");
    cmdCreatorStartPub_ = nh_.serviceClient<std_srvs::Empty>("cmd_start_pub");
    cmdCreatorStopPub_ = nh_.serviceClient<std_srvs::Empty>("cmd_stop_pub");

    clients_[STARTING] = new StartingProcedureClient("starting_procedure");
    action_args_[STARTING] = start_distance_;

    clients_[DRIVING] = new DriveClient("free_drive");
    action_args_[DRIVING] = [](bool x){return x;}(false);

    clients_[PARKING_SEARCH] = new SearchClient("search");
    action_args_[PARKING_SEARCH] = parking_spot_;

    clients_[PARK] = new ParkClient("park");
    action_args_[PARK] = [](geometry_msgs::Polygon x){return x;};


    ROS_INFO("Clients created successfully");
}
Scheduler::~Scheduler()
{
    pnh_.deleteParam("begin_action");
}
void Scheduler::init()
{
    startAction((action)begin_action_);
}
void Scheduler::startAction(action action_to_set)
{
    current_client_ptr_ = clients_[action_to_set];
    current_client_ptr_->waitForServer(200);
    current_client_ptr_->setGoal(action_args_[action_to_set]);
}
bool Scheduler::checkIfActionFinished()
{
    return current_client_ptr_->isActionFinished();
}
void Scheduler::loop()
{
    if (checkIfActionFinished())
    {
        current_client_ptr_->getActionResult(action_args_[current_client_ptr_->getNextAction()]);
        shiftAction();
    }
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
        startAction(DRIVING);
    }
    else if (checkCurrentClientType<DriveClient*>())
    {
        startAction(PARKING_SEARCH);
    }
    else if (checkCurrentClientType<SearchClient*>())
    {
        startAction(PARK);
    }
    else if (checkCurrentClientType<ParkClient*>())
    {
        startAction(DRIVING);
    }
    else
    {
        ROS_WARN("No such action client!!");
    }
}
void Scheduler::stateMachine()
{
    current_car_state_ = current_client_ptr_->getActionState();

    if(current_car_state_ == previous_car_state_)
    {
        return;
    }
    switch(current_car_state_)
    {
        case SELFIE_READY:
            ROS_INFO("STATE_SELFIE_READY");
            previous_car_state_ = SELFIE_READY;
            break;
        case BUTTON_FREE_DRIVE_PRESSED:
            ROS_INFO("BUTTON_FREE_DRIVE_PRESSED");
            previous_car_state_ = BUTTON_FREE_DRIVE_PRESSED;
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
