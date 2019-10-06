/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include <ros/ros.h>
#include <selfie_starting_procedure/starting_procedure_action.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "starting_procedure_action");
    StartingProcedureAction StartingProcedure("starting_procedure");
    ros::spin();
}
