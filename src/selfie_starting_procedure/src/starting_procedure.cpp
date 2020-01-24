/***Copyright ( c) 2019, KNR Selfie*
 * This code is licensed under BSD license (see LICENSE for details)
 **/
#include <ros/ros.h>
#include <selfie_starting_procedure/starting_procedure_action.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "starting_procedure");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  StartingProcedureAction s_p(nh, pnh);

  ros::spin();
  return 0;
}
