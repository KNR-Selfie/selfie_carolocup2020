#include <selfie_scheduler/search_action_client.h>

SearchClient::SearchClient(std::string name):
    ac_(name, true)
{
}

SearchClient::~SearchClient()
{

}


void SearchClient::setGoal(boost::any goal)
{
    float parking_spot;

    try
    {
        parking_spot = boost::any_cast<float>(goal);
    }
    catch (boost::bad_any_cast &e)
    {
        ROS_ERROR("bad casting %s",e.what());
        return;
    }

    ROS_INFO("Good goal cast");
    goal_.min_spot_lenght= parking_spot;
    ac_.sendGoal(goal_,boost::bind(&SearchClient::doneCb, this, _1,_2),
                boost::bind(&SearchClient::activeCb,this),
                boost::bind(&SearchClient::feedbackCb,this,_1));
}

bool SearchClient::waitForResult(float timeout)
{
    return ac_.waitForResult(ros::Duration(timeout));
}
bool SearchClient::waitForServer(float timeout)
{
    ROS_INFO("Wait for driving action server");
    return ac_.waitForServer(ros::Duration(timeout));
}
void SearchClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const selfie_msgs::searchResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    //result_ = result;
}

void SearchClient::activeCb()
{
    ROS_INFO("Starting procedure server active");
}
void SearchClient::feedbackCb(const selfie_msgs::searchFeedbackConstPtr& feedback)
{
  ROS_INFO("Starting procedure feedback %d", feedback->action_status);
  action_state_ = (program_state)feedback->action_status;
}

void SearchClient::cancelAction()
{
  ac_.cancelAllGoals();
}

program_state SearchClient::getActionState()
{
    return action_state_;
}
