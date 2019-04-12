#include <iostream>
#include <vector>
#include "ros/ros.h"

#include "../../selfie_parking_manager/include/selfie_parking_manager/Parking_manager.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "parking_manager");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    Parking_manager parking_node(nh,pnh);

    ros::spin();//TODO while rosok
    return 0;
}