#include <selfie_scheduler/starting_action_client.h>

StartingProcedureClient::StartingProcedureClient(std::string name):
    ac_(name, true)
{
    ROS_INFO("Wait for server");
     ac_.waitForServer();
}

StartingProcedureClient::~StartingProcedureClient()
{

}


void StartingProcedureClient::setGoal(boost::any goal)
{
    float distance = boost::any_cast<float>(goal);
    if(distance)
    {
        ROS_INFO("Good goal cast");
        goal_.distance = distance;
        ac_.sendGoal(goal_,boost::bind(&StartingProcedureClient::doneCb, this, _1,_2),
                    boost::bind(&StartingProcedureClient::activeCb,this),
                    boost::bind(&StartingProcedureClient::feedbackCb,this,_1));
    }
    else
    {
        ROS_INFO("Parameter invalid");
    }
}

bool StartingProcedureClient::waitForResult(float timeout)
{
    return ac_.waitForResult(ros::Duration(timeout));
}

void StartingProcedureClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const selfie_msgs::startingResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("result: %i", result->drive_mode);
}

void StartingProcedureClient::activeCb()
{
  ROS_INFO("Starting procedure server active");
  //STARTING
}
void StartingProcedureClient::feedbackCb(const selfie_msgs::startingFeedbackConstPtr& feedback)
{
  ROS_INFO("Starting procedure feedback %d", feedback->action_status);
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
