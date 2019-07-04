#include <selfie_park/Search_client_mock.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "detect_parking_spot_mock_client");
  ros::NodeHandle nh;
  Search_client_mock client(nh);

  ros::Duration(1).sleep();

  float len = 0.5;
  std::cout << argc;
  if (argc == 2) {
    len = atof(argv[1]);
    std::cout << argv[1];
  }
  client.send_goal(len);
  std::cout << "sent goal min length of spot: " << len << std::endl;

  ros::spin();
  return 0;
}