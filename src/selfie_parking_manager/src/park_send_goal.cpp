#include <../include/selfie_parking_manager/park_send_goal.h>






int main(int argc, char** argv)
{
    ros::init(argc, argv, "park_send_goal");
    ros::NodeHandle nh;
    park_send_goal client(nh);

    ros::Duration(1).sleep();

    float len=0.5;
    std::cout<<argc;
    if(argc==2){
        len=atof(argv[1]);
        std::cout<<argv[1];
    }
    client.send_goal(len);
    std::cout<<"sent goal min length of spot: "<<len<<std::endl;


    return 0;

}