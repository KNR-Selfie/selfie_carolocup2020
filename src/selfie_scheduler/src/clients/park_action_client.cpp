#include <selfie_scheduler/park_action_client.h>

ParkClient::ParkClient(std::string name):
    ac_(name, true)
{
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
        ROS_ERROR("bad casting %s",e.what());
        return;
    }

    ROS_INFO("Good goal cast");
    goal_.parking_spot = parking_spot;
    ac_.sendGoal(goal_,boost::bind(&ParkClient::doneCb, this, _1,_2),
                boost::bind(&ParkClient::activeCb,this),
                boost::bind(&ParkClient::feedbackCb,this,_1));
}

bool ParkClient::waitForResult(float timeout)
{
    return ac_.waitForResult(ros::Duration(timeout));
}
bool ParkClient::waitForServer(float timeout)
{
    ROS_INFO("Wait for driving action server");
    return ac_.waitForServer(ros::Duration(timeout));
}
void ParkClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const selfie_msgs::parkResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

void ParkClient::activeCb()
{
    ROS_INFO("Starting procedure server active");
}
void ParkClient::feedbackCb(const selfie_msgs::parkFeedbackConstPtr& feedback)
{
  ROS_INFO("Starting procedure feedback %d", feedback->action_status);
  action_state_ = (program_state)feedback->action_status;
}

void ParkClient::cancelAction()
{
  ac_.cancelAllGoals();
}

program_state ParkClient::getActionState()
{
    return action_state_;
}
