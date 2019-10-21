/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/

#include <selfie_park/Search_client_mock.h>

ros::Publisher Search_client_mock::obstacles_pub_;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_parking_spot_mock_client");
  ros::NodeHandle nh;
  Search_client_mock client(nh);

  ros::Duration(1).sleep();

  float len = 0.5;
  ros::Timer timer;

  if (argc >= 2)
  {
    if (std::strcmp(argv[1], "-o")==0)
    {
      std::cout << "mock obstacles active\n";
      Search_client_mock::obstacles_pub_ = nh.advertise<selfie_msgs::PolygonArray>("obstacles", 1);
      timer = nh.createTimer(ros::Duration(0.4), sendMockObstacles);
      if (argc == 3)
        len = atof(argv[2]);
    } else
    {
      std::cout << "mock obstacles disabled ( -o to activate)\n";
      len = atof(argv[1]);
    }
  }

  client.send_goal(len);
  std::cout << "sent goal min length of spot: " << len << std::endl;

  ros::spin();
  return 0;
}