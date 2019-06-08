#include <ros/ros.h>
#include <selfie_scheduler/starting_action_client.h>
#include <selfie_scheduler/drive_action_client.h>
#include <selfie_scheduler/scheduler_enums.h>




int main(int argc, char **argv)
{
    ros::init(argc, argv, "scheduler");
    ros::NodeHandle nh;

    feedback_variable current_car_state = SELFIE_IDLE;
    feedback_variable previous_car_state = SELFIE_IDLE;

    action_variable current_action = IDLE;
    action_variable previous_action = IDLE;

    StartingProcedureClient startingAction("starting_procedure");
    DriveClient driveAction("free_drive");

    current_action = STARTING; //dummy - set if all systems launched succesfuly

    while(ros::ok())
    {
        ros::spinOnce();
        current_car_state = startingAction.getActionState();

        if(current_action != previous_action)
        {
            switch(current_action)
            {
                case STARTING:
                    previous_action = STARTING;
                    startingAction.waitForServer(200);
                    startingAction.setGoal(float(1.0));
                    break;
                case DRIVING:
                    previous_action = DRIVING;
                    driveAction.waitForServer(200);
                    driveAction.setGoal(startingAction.getResult());
                    break;
                case PARKING_SEARCH:
                    break;
                case PARK:
                    break;
            }
        }

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
                //get goal
                current_action = DRIVING;
                break;
        }

    }
}
