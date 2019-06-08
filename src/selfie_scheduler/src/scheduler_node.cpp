#include <ros/ros.h>
#include <selfie_scheduler/starting_action_client.h>
#include <selfie_scheduler/scheduler_enums.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scheduler");
    ros::NodeHandle nh;

    feedback_variable current_car_state = SELFIE_IDLE;
    feedback_variable previous_car_state = SELFIE_IDLE;

    action_variable current_action = IDLE;

    StartingProcedureClient startingAction("starting_procedure");
    startingAction.setGoal(float(30.23));


    while(ros::ok())
    {
        ros::spinOnce();
        current_car_state = startingAction.getActionState();

        //compare states
        if(current_car_state == previous_car_state)
        {
            continue;
        }

        switch(current_car_state)
        {
            case SELFIE_READY:
                ROS_INFO("STATE_SELFIE_READY");
                previous_car_state = SELFIE_READY;
                break;
            case BUTTON_FREE_DRIVE_PRESSED:
                ROS_INFO("BUTTON_FREE_DRIVE_PRESSED");
                previous_car_state = BUTTON_FREE_DRIVE_PRESSED;
                break;
            case BUTTON_OBSTACLE_DRIVE_PRESSED:
                ROS_INFO("BUTTON_OBSTACLE_DRIVE_PRESSED");
                previous_car_state = BUTTON_OBSTACLE_DRIVE_PRESSED;
                break;
            case START_DRIVE:
                ROS_INFO("START_DRIVE");
                previous_car_state = START_DRIVE;
                break;
                //goal reached 
            case END_DRIVE:
                ROS_INFO("END DRIVE");
                previous_car_state = END_DRIVE;
                break;
        }

    }
}
