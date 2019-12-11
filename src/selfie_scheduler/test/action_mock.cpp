#include <ros/ros.h>
#include <selfie_scheduler/search_action_client.h>
#include <selfie_scheduler/drive_action_client.h>
#include <selfie_scheduler/starting_action_client.h>
#include <selfie_scheduler/park_action_client.h>
#include <selfie_scheduler/scheduler_enums.h>
#include <selfie_scheduler/intersection_action_client.h>
#include <selfie_scheduler/scheduler.h>
#include <actionlib/server/simple_action_server.h>
#include <selfie_msgs/PolygonArray.h>

class ActionMock
{
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    actionlib::SimpleActionServer <selfie_msgs::parkAction> park_action_;
    actionlib::SimpleActionServer <selfie_msgs::drivingAction> drive_action_;
    actionlib::SimpleActionServer <selfie_msgs::startingAction> starting_action_;
    actionlib::SimpleActionServer <selfie_msgs::searchAction> search_action_;
    actionlib::SimpleActionServer <selfie_msgs::intersectionAction> intersection_action_;

    // params
    bool button_;
    bool search_abort_;
    int action_time_;
public:
    ActionMock():
        pnh_("~"),
        park_action_(nh_,"park",boost::bind(&ActionMock::park_action_goalCB,this,_1),false),
        drive_action_(nh_,"free_drive",boost::bind(&ActionMock::drive_action_goalCB,this,_1),false),
        starting_action_(nh_,"starting_procedure",boost::bind(&ActionMock::starting_action_goalCB,this,_1),false),
        search_action_(nh_,"search",boost::bind(&ActionMock::search_action_goalCB,this,_1),false),
        intersection_action_(nh_,"intersection",boost::bind(&ActionMock::intersection_action_goalCB,this,_1),false)
    {

        pnh_.getParam("button", button_);
        pnh_.getParam("search_abort",search_abort_);
        pnh_.getParam("action_time",action_time_);

        starting_action_.registerPreemptCallback(boost::bind(&ActionMock::starting_action_preemptCB, this));
        drive_action_.registerPreemptCallback(boost::bind(&ActionMock::drive_action_preemptCB, this));
        search_action_.registerPreemptCallback(boost::bind(&ActionMock::search_action_preemptCB, this));
        park_action_.registerPreemptCallback(boost::bind(&ActionMock::park_action_preemptCB, this));
        intersection_action_.registerPreemptCallback(boost::bind(&ActionMock::intersection_action_preemptCB, this));

        starting_action_.start();
        drive_action_.start();
        search_action_.start();
        park_action_.start();
        intersection_action_.start();

    }
    void waitGivenTime(float sec)
    {
        ros::Time begin = ros::Time::now();
        while((ros::Time::now() - begin) < ros::Duration(sec))
        {

        }
    }
    void starting_action_goalCB(const selfie_msgs::startingGoalConstPtr &goal)
    {
        ROS_INFO("Starting procedure goal is %f",goal->distance);

        waitGivenTime(1);
        selfie_msgs::startingFeedback starting_feedback;
        starting_feedback.action_status = BUTTON_PARKING_DRIVE_PRESSED;
        waitGivenTime(action_time_);

        starting_action_.publishFeedback(starting_feedback);

        if (starting_action_.isActive())
            starting_action_.setSucceeded([](bool result){selfie_msgs::startingResult starting_result; starting_result.drive_mode = result; return starting_result;}(button_));
    }

    void drive_action_goalCB(const selfie_msgs::drivingGoalConstPtr &goal)
    {
        ROS_INFO("Drive goal is %d",goal->mode);
        selfie_msgs::drivingFeedback drive_feedback;
        drive_feedback.action_status = AUTONOMOUS_DRIVE;
        drive_action_.publishFeedback(drive_feedback);
        waitGivenTime(action_time_);
        if (drive_action_.isActive())
            drive_action_.setSucceeded([](bool result){selfie_msgs::drivingResult drive_result; drive_result.event = result; return drive_result;}(true));
    }
    void park_action_goalCB(const selfie_msgs::parkGoalConstPtr &goal)
    {
        ROS_INFO("Park goal is");
        selfie_msgs::parkFeedback park_feedback;
        park_feedback.action_status = START_PARK;
        park_action_.publishFeedback(park_feedback);
        waitGivenTime(action_time_);
        if (park_action_.isActive())
            park_action_.setSucceeded([](bool result){selfie_msgs::parkResult park_result; park_result.done = result; return park_result;}(true));
    }
    void search_action_goalCB(const selfie_msgs::searchGoalConstPtr &goal)
    {
        ROS_INFO("Search goal is %f",goal->min_spot_lenght);
        selfie_msgs::searchFeedback search_feedback;
        search_feedback.action_status = START_SEARCHING_PLACE;
        search_action_.publishFeedback(search_feedback);
        waitGivenTime(action_time_);

        if(search_abort_ && (search_action_.isPreemptRequested() || search_action_.isActive()))
            search_action_.setAborted();
        else if (search_action_.isActive())
            search_action_.setSucceeded(this->getMockObstacle());
    }
    void intersection_action_goalCB(const selfie_msgs::intersectionGoalConstPtr &goal)
    {
        ROS_INFO("Intersection goal is empty");
        waitGivenTime(action_time_);
        if(intersection_action_.isActive())
            intersection_action_.setSucceeded([](bool result){selfie_msgs::intersectionResult intersection_result; intersection_result.done = result; return intersection_result;}(true));
    }

    selfie_msgs::searchResult getMockObstacle()
    {

        selfie_msgs::searchResult result;
        geometry_msgs::Point32 p;
        p.x = 1;
        p.y = 2;
        result.parking_spot.points.push_back(p);
        p.x = 1;
        p.y = 2;
        result.parking_spot.points.push_back(p);
        p.x = 1;
        p.y = 2;
        result.parking_spot.points.push_back(p);
        p.x = 1;
        p.y = 2;
        result.parking_spot.points.push_back(p);
        return result;
    }
    void park_action_preemptCB()
    {
        ROS_INFO("Park action aborted");
        park_action_.setAborted();
    }
    void search_action_preemptCB()
    {
        ROS_INFO("Search action aborted");
        search_action_.setAborted();
    }
    void drive_action_preemptCB()
    {
        ROS_INFO("Drive action aborted");
        drive_action_.setAborted();
    }
    void starting_action_preemptCB()
    {
        ROS_INFO("Starting action aborted");
        starting_action_.setAborted();
    }
    void intersection_action_preemptCB()
    {
        ROS_INFO("Starting action aborted");
        intersection_action_.setAborted();
    }

};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_mock");

    ActionMock action_mock;

    while (ros::ok())
    {
        ros::spinOnce();
    }
}
