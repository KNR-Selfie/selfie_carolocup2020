#include <selfie_perception/obstacles_generator.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_obstacles");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ObstaclesGenerator obstaclesGenerator(nh, pnh);
    if (obstaclesGenerator.init())
    {
        ROS_INFO("detect_obstacles initialized");
    }
    else
    {
        ROS_INFO("detect_obstacles doesn't work");
        return 1;
    }

    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}