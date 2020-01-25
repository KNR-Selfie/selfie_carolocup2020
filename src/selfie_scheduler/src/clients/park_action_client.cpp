#include <selfie_scheduler/park_action_client.h>
#include <std_srvs/Empty.h>

ParkClient::ParkClient(std::string name, const ros::NodeHandle &pnh):
    ac_(name, true),
    pnh_(pnh),
    result_(0),
    parking_steering_mode_(ACKERMANN),
    sucessful_park_counter_(0),
    park_atttempts_counter_(0)
{
    next_action_ = DRIVING;
    result_flag_ = EMPTY;
    action_state_ = SELFIE_IDLE;
    cmdCreatorStopPub_ = nh_.serviceClient<std_srvs::Empty>("cmd_stop_pub");
    steeringModeSetAckermann_ = nh_.serviceClient<std_srvs::Empty>("steering_ackerman");
    steeringModeSetParallel_ = nh_.serviceClient<std_srvs::Empty>("steering_parallel");
    
    pnh_.getParam("parking_steering_mode", parking_steering_mode_);
}

ParkClient::~ParkClient()
{
}
void ParkClient::setGoal(boost::any goal)
{
    geometry_msgs::Polygon parking_spot;

    try
    {
        parking_spot = boost::any_cast<geometry_msgs::Polygon>(goal);
    }
    catch (boost::bad_any_cast &e)
    {
        ROS_ERROR("bad casting %s", e.what());
        return;
    }
    goal_.parking_spot = parking_spot;
    ac_.sendGoal(goal_, boost::bind(&ParkClient::doneCb, this, _1, _2),
                boost::bind(&ParkClient::activeCb, this),
                boost::bind(&ParkClient::feedbackCb, this, _1));
    park_atttempts_counter_++;
}
bool ParkClient::waitForResult(float timeout)
{
    return ac_.waitForResult(ros::Duration(timeout));
}
bool ParkClient::waitForServer(float timeout)
{
    result_flag_ = EMPTY;
    ROS_INFO("Wait for park action server");
    return ac_.waitForServer(ros::Duration(timeout));
}
void ParkClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const selfie_msgs::parkResultConstPtr& result)
{
    ROS_INFO("Finished park in state [%s]", state.toString().c_str());
    if(state == actionlib::SimpleClientGoalState::StateEnum::ABORTED)
    {
        result_flag_ = ABORTED;
    }
    else
    {
        result_flag_ = SUCCESS;
        sucessful_park_counter_++;
    }

    //todo implement more logic 
    if(sucessful_park_counter_ == 2)
    {
        result_ = PARKING_COMPLETE;
    }

}
void ParkClient::activeCb()
{
    ROS_INFO("Park server active");
}
void ParkClient::feedbackCb(const selfie_msgs::parkFeedbackConstPtr& feedback)
{
  ROS_INFO("Park action feedback %d", feedback->action_status);
  action_state_ = (program_state)feedback->action_status;
}
void ParkClient::cancelAction()
{
  ac_.cancelAllGoals();
}
void ParkClient::getActionResult(boost::any &result)
{
    result = result_;
}
void ParkClient::setParkSteeringMode()
{   
    std_srvs::Empty empty_msg;
    if(parking_steering_mode_)
        steeringModeSetParallel_.call(empty_msg);
    else
        steeringModeSetAckermann_.call(empty_msg);
}
void ParkClient::prepareAction()
{
    std_srvs::Empty empty_msg;
    cmdCreatorStopPub_.call(empty_msg);
    setParkSteeringMode();

    ROS_INFO("Prepare park -  cmd creator stop pub, set steering mode to %d", parking_steering_mode_);
}

