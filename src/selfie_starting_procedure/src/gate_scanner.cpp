#include <ros/ros.h>
#include "selfie_starting_procedure/gate_scanner.h"



int main(int argc, char** argv)
{
    ros::init(argc,argv,"qr_decoder");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    GateScanner gateScanner(nh,pnh);
    ros::spin();
    return 0;
}