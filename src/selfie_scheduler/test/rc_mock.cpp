#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <selfie_scheduler/scheduler_enums.h>

class Rc_mock
{
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher rc_switch_state_pub_ = nh_.advertise<std_msgs::UInt8>("switch_state",1);
    ros::Timer timer_autonomous_pub_ = nh_.createTimer(ros::Duration(1.0 / 10.0),
                                                       std::bind(&Rc_mock::set_switch_state, this, RC_AUTONOMOUS));
    ros::Timer timer_rc_reset_;

    int rc_reset_time_;
public:
    Rc_mock():pnh_("~")
    {
        pnh_.getParam("rc_reset_time",rc_reset_time_);
        if(rc_reset_time_>0)
        {
            timer_rc_reset_= nh_.createTimer(ros::Duration(rc_reset_time_),
                           std::bind(&Rc_mock::rc_reset, this));
        }


    }
    void rc_reset()
    {
        ROS_INFO("Reset ");
        timer_autonomous_pub_.stop();

        std_msgs::UInt8 msg;
        msg.data = RC_MANUAL;

        for(int i = 0;i<5;i++)
        {
            rc_switch_state_pub_.publish(msg);
            ros::spinOnce();
            rc_switch_state_pub_.publish(msg);
            ros::spinOnce();
        }

        timer_autonomous_pub_.start();
    }
    void set_switch_state(rc_state state)
    {
        std_msgs::UInt8 msg;
        msg.data = state;
        rc_switch_state_pub_.publish(msg);
    }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rc_mock");
    Rc_mock rc_mock;

    while (ros::ok())
    {
        ros::spinOnce();
    }
}
