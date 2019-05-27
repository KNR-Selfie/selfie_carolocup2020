#include <selfie_starting_procedure/starting_procedure_action.h>

StartingProcedureAction::StartingProcedureAction(std::string name) :
    as(nh, name, boost::bind(&StartingProcedureAction::executeCB, this, _1), false),
  action_name(name)
{
    as.start();
    button_sub = nh.subscribe("start_button1", 1000, &StartingProcedureAction::buttonCB, this);
    distance_sub = nh.subscribe("distance",10,&StartingProcedureAction::distanceCB, this);
}
StartingProcedureAction::~StartingProcedureAction(void)
{

}

void StartingProcedureAction::executeCB(const selfie_msgs::startingGoalConstPtr &goal)
{
    ROS_INFO("goal %f",goal->distance);
    feedback.action_status = START_SIGN;//ButtonPressed;

    for(int i = 0;i<10;i++)
    {
       as.publishFeedback(feedback);
       ros::Duration(1).sleep();

    }
    result.drive_mode = true;
    as.setSucceeded(result);
}
void StartingProcedureAction::buttonCB(const std_msgs::BoolConstPtr &msg)
{

}

void StartingProcedureAction::distanceCB(const std_msgs::Float32ConstPtr &msg)
{

}
