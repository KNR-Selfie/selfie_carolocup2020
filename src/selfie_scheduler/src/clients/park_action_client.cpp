#include <selfie_scheduler/park_action_client.h>

ParkClient::ParkClient(std::string name):
    ac_(name, true)
{
    next_action_ = DRIVING;
    result_flag_ = false;
    action_state_ = SELFIE_IDLE;
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
}
bool ParkClient::waitForResult(float timeout)
{
    return ac_.waitForResult(ros::Duration(timeout));
}
bool ParkClient::waitForServer(float timeout)
{
    result_flag_ = false;
    ROS_INFO("Wait for park action server");
    return ac_.waitForServer(ros::Duration(timeout));
}
void ParkClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const selfie_msgs::parkResultConstPtr& result)
{
    ROS_INFO("Finished park in state [%s]", state.toString().c_str());
    if(state == actionlib::SimpleClientGoalState::StateEnum::ABORTED)
    {
        ROS_INFO("ABORTED!!");
        result_flag_ = 2;
    }
    else
    {
        result_ = result->done;
        result_flag_ = 1;
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
program_state ParkClient::getActionState()
{
    if(action_state_ != SELFIE_IDLE)
        return action_state_;
}
int ParkClient::isActionFinished()
{
    return result_flag_;
}
void ParkClient::getActionResult(boost::any &result)
{
    result = result_;
}
action ParkClient::getNextAction()
{
    return next_action_;
}