#include <ros/ros.h>
#include <selfie_scheduler/search_action_client.h>
#include <selfie_scheduler/drive_action_client.h>
#include <selfie_scheduler/starting_action_client.h>
#include <selfie_scheduler/park_action_client.h>
#include <selfie_scheduler/scheduler_enums.h>
#include <selfie_scheduler/scheduler.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scheduler");
    Scheduler selfie_scheduler;  // action initialization
    selfie_scheduler.init();
    ros::Rate sleep_rate(100);

// wait for key press
    selfie_scheduler.waitForStart();
    while (ros::ok())
    {
        ros::spinOnce();
        selfie_scheduler.loop();
        sleep_rate.sleep();
    }
}
