#include <selfie_scheduler/starting_action_client.h>

StartingProcedureClient::StartingProcedureClient(std::string name):
    ac_(name, true)
{
    result_flag_ = false;
}

StartingProcedureClient::~StartingProcedureClient()
{

}


void StartingProcedureClient::setGoal(boost::any goal)
{
    float distance;

    try
    {
        distance = boost::any_cast<float>(goal);
    }
    catch (boost::bad_any_cast &e)
    {
        ROS_ERROR("bad casting %s",e.what());
        return;
    }

    ROS_INFO("Good goal cast");
    goal_.distance = distance;
    ac_.sendGoal(goal_,boost::bind(&StartingProcedureClient::doneCb, this, _1,_2),
                boost::bind(&StartingProcedureClient::activeCb,this),
                boost::bind(&StartingProcedureClient::feedbackCb,this,_1));
}

bool StartingProcedureClient::waitForResult(float timeout)
{
    return ac_.waitForResult(ros::Duration(timeout));
}
bool StartingProcedureClient::waitForServer(float timeout)
{
    ROS_INFO("Wait for starting action server");
    return ac_.waitForServer(ros::Duration(timeout));
}
void StartingProcedureClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const selfie_msgs::startingResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("result: %i", result->drive_mode);
  result_ = result->drive_mode;
  result_flag_ = true;
}

void StartingProcedureClient::activeCb()
{
  ROS_INFO("Starting procedure server active");
  //STARTING
}
void StartingProcedureClient::feedbackCb(const selfie_msgs::startingFeedbackConstPtr& feedback)
{
  //ROS_INFO("Starting procedure feedback %d", feedback->action_status);
  action_state_ = (program_state)feedback->action_status;
}

void StartingProcedureClient::cancelAction()
{
  ac_.cancelAllGoals();
}

program_state StartingProcedureClient::getActionState()
{
    return action_state_;
}

bool StartingProcedureClient::getResult()
{
    return result_;
}
bool StartingProcedureClient::isActionFinished()
{
    return result_flag_;
}
