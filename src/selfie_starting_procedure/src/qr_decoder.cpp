#include <ros/ros.h>
#include "selfie_starting_procedure/qr_decoder.h"



int main(int argc, char** argv)
{
    ros::init(argc, argv, "qr_decoder");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    QrDecoder qr_decoder(nh, pnh);
    ros::spin();
    return 0;
}