#include <iostream>
#include <vector>
#include "ros/ros.h"

#include "../../selfie_park/include/selfie_park/Search_server.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_parking_spot");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    Search_server parking_node(nh,pnh);

    ros::spin();//TODO while rosok
    return 0;
}