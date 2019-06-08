#include <ros/ros.h>
#include <selfie_scheduler/search_action_client.h>
#include <selfie_scheduler/drive_action_client.h>
#include <selfie_scheduler/starting_action_client.h>
#include <selfie_scheduler/park_action_client.h>

#include <std_srvs/Empty.h>



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
    SearchClient searchAction("search");
    ParkClient parkAction("park");

    current_action = STARTING; //dummy - set if all systems launched succesfuly

    ros::ServiceClient resetVision = nh.serviceClient<std_srvs::Empty>("resetVision");
    ros::ServiceClient cmdStartPub = nh.serviceClient<std_srvs::Empty>("cmd_start_pub");
    ros::ServiceClient cmdStopPub = nh.serviceClient<std_srvs::Empty>("cmd_stop_pub");

    while(ros::ok())
    {
        ros::spinOnce();
        current_car_state = startingAction.getActionState();

        if(current_action != previous_action)
        {
            switch(current_action)
            {
                case STARTING:
                {
                    previous_action = STARTING;
                    startingAction.waitForServer(200);
                    startingAction.setGoal(float(1.0));
                    break;
                }
                case DRIVING:
                {
                    previous_action = DRIVING;
                    driveAction.waitForServer(200);
                    driveAction.setGoal(startingAction.getResult());

                    std_srvs::Empty reset_msg;
                    resetVision.call(reset_msg);

                    std_srvs::Empty start_pub;
                    cmdStartPub.call(start_pub);
                    break;
                }
                case PARKING_SEARCH:
                {
                    previous_action = PARKING_SEARCH;
                    searchAction.waitForServer(200);
                    searchAction.setGoal(float(70.0));
                    break;
                }
                case PARK:
                {
                    previous_action = PARK;
                    parkAction.waitForServer(200);
                    parkAction.setGoal(333);

                    std_srvs::Empty stop_pub;
                    cmdStopPub.call(stop_pub);

                    break;
                }
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
