#include <ros/ros.h>
#include <selfie_starting_procedure/starting_procedure_action.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "starting_procedure_action");
    StartingProcedureAction StartingProcedure("starting_procedure_task");
}
