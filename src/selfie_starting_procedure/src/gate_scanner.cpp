/***Copyright ( c) 2019, KNR Selfie*
 * This code is licensed under BSD license (see LICENSE for details)
 **/
#include <ros/ros.h>
#include "selfie_starting_procedure/gate_scanner.h"



int main(int argc, char** argv)
{
  ros::init(argc, argv, "qr_decoder");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  GateScanner gate_scanner(nh, pnh);
  ros::spin();
  return 0;
}
