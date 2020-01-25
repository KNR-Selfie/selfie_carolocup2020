/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/

#include "../include/selfie_park/park_server.h"

ParkService::ParkService(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
nh_(nh),
pnh_(pnh),
as_(nh_, "park", false),
dr_server_CB_(boost::bind(&ParkService::reconfigureCB, this, _1, _2))
{
  pnh_.param<std::string>("odom_topic", odom_topic_, "/odom");
  pnh_.param<std::string>("ackermann_topic", ackermann_topic_, "/drive");
  pnh_.param<float>("minimal_start_parking_x_", minimal_start_parking_x_, -0.16);
  pnh_.param<bool>("state_msgs", state_msgs_, false);
  pnh_.param<float>("parking_speed", parking_speed_, 0.4);
  pnh_.param<float>("max_turn", max_turn_, 0.8);
  pnh_.param<float>("idle_time", idle_time_, 2.);
  pnh_.param<float>("iter_distance",iter_distance_,0.2);
  pnh_.param<float>("angle_coeff", angle_coeff_, 1./2.);
  pnh_.param<float>("back_to_base_", back_to_base_, 0.25);

  front_target_ = 0.;
  back_target_ = 0.;
  dr_server_.setCallback(dr_server_CB_);
  move_state_ = first_phase;
  parking_state_ = not_parking;
  action_status_ = READY_TO_DRIVE;
  as_.registerGoalCallback(boost::bind(&ParkService::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&ParkService::preemptCB, this));
  as_.start();
  dist_sub_ = nh_.subscribe("/distance", 1, &ParkService::distanceCallback, this);
  ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(ackermann_topic_, 10);
  right_indicator_pub_ = nh_.advertise<std_msgs::Bool>("right_turn_indicator", 20);
  left_indicator_pub_ = nh_.advertise<std_msgs::Bool>("left_turn_indicator", 20);
}

void ParkService::distanceCallback(const std_msgs::Float32 &msg)
{

    actual_dist_ = msg.data;

    switch (parking_state_)
    {
      case go_to_parking_spot:
        action_status_ = START_PARK;
        if (toParkingSpot())
        {
            prev_dist_ = actual_dist_;
            parking_state_ = going_in;

        }
        if (state_msgs_) ROS_INFO_THROTTLE(5, "go_to_parking_spot");
        blinkRight(true);
        blinkLeft(false);
        break;

      case going_in:
        if (state_msgs_) ROS_INFO_THROTTLE(5, "get_in");
        if (park()) parking_state_ = parked;
        blinkRight(true);
        blinkLeft(false);
        break;

      case parked:
        action_status_ = IN_PLACE;
        if (state_msgs_) ROS_INFO_THROTTLE(5, "parked");
        drive(0., 0.);
        blinkLeft(true);
        blinkRight(true);
        ros::Duration(idle_time_).sleep();
        parking_state_ = going_out;
        break;

      case going_out:
        blinkLeft(true);
        blinkRight(false);
        if (state_msgs_) ROS_INFO_THROTTLE(5, "get_out");
        if (leave()) parking_state_ = out;
        break;

      case out:
        action_status_ = READY_TO_DRIVE;
        blinkLeft(false);
        blinkRight(false);
        if (state_msgs_) ROS_INFO_THROTTLE(5, "out");
        drive(parking_speed_, 0);
        selfie_msgs::parkFeedback feedback;
        feedback.action_status = READY_TO_DRIVE;
        as_.publishFeedback(feedback);
        selfie_msgs::parkResult result;
        result.done = true;
        as_.setSucceeded(result);
        parking_state_ = not_parking;
        break;
    }
}

void ParkService::goalCB()
{
  selfie_msgs::parkGoal goal = *as_.acceptNewGoal();
  initParkingSpot(goal.parking_spot);
  selfie_msgs::parkFeedback feedback;
  feedback.action_status = START_PARK;
  as_.publishFeedback(feedback);
  parking_state_ = go_to_parking_spot;
}

void ParkService::preemptCB()
{
  ROS_INFO("parkService preempted");
  blinkLeft(false);
  blinkRight(false);
  parking_state_ = not_parking;
  move_state_ = first_phase;
  as_.setAborted();
}


void ParkService::initParkingSpot(const geometry_msgs::Polygon &msg)
{
    float min_point_dist = 10000.f;
    float sum = 0.;
    for(auto it = msg.points.begin();it<msg.points.end();++it)
    {
        if(it->x < min_point_dist) min_point_dist = it->x;
        sum+= it->y;
    }
    park_spot_dist_ini_ = std::abs(sum/msg.points.size());
    park_spot_dist_ = park_spot_dist_ini_;

    back_target_ = actual_dist_ + min_point_dist + back_to_base_;
    front_target_ = back_target_ + iter_distance_;
}


void ParkService::drive(float speed, float steering_angle)
{
  ackermann_msgs::AckermannDriveStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.drive.speed = speed;
  msg.drive.steering_angle = steering_angle;
  ackermann_pub_.publish(msg);
}


bool ParkService::toParkingSpot()
{
    if(actual_dist_ > back_target_)
    {
        return true;
    }
    drive(parking_speed_, 0.);
    return false;
}

bool ParkService::park()
{
    park_spot_dist_ -= angle_coeff_*std::sin(max_turn_)*std::abs(actual_dist_ - prev_dist_); 
    bool in_pos = park_spot_dist_ > 0.f;

    if(move_state_ == first_phase)
    {
        if(in_pos)
        {
            if(actual_dist_ > front_target_)
            {
                move_state_ = second_phase;
                drive(0., max_turn_);
            }
            else
            {
                drive(parking_speed_, -max_turn_);
            }
            
        }
        else
        {
            move_state_ = second_phase;
            return true;
        }
        

    }
    else
    {
        if(in_pos)
        {
            if(actual_dist_ < back_target_)
            {
                move_state_ = first_phase;
                drive(0., -max_turn_);
            }
            else
            {
                drive(-parking_speed_, max_turn_);
            }
            
        }
        else
        {
            move_state_ = first_phase;
            return true;
        }
        
    }
    prev_dist_ = actual_dist_;
    return false;
}

bool ParkService::leave()
{

    park_spot_dist_ += angle_coeff_*std::sin(max_turn_)*std::abs(actual_dist_ - prev_dist_);
    bool in_pos = park_spot_dist_ < park_spot_dist_ini_;
    if(move_state_ == first_phase)
    {
        if(in_pos)
        {
            if(actual_dist_ > front_target_)
            {
                move_state_ = second_phase;
                drive(0., -max_turn_);
            }
            else
            {
                drive(parking_speed_, max_turn_);
            }
            
        }
        else
        {
            move_state_ = second_phase;
            return true;
        }
        

    }
    else
    {
        if(in_pos)
        {
            if(actual_dist_ < back_target_)
            {
                move_state_ = first_phase;
                drive(0., max_turn_);
            }
            else
            {
                drive(-parking_speed_, -max_turn_);
            }
            
        }
        else
        {
            move_state_ = first_phase;
            return true;
        }
        
    }
    prev_dist_ = actual_dist_;
    return false;

}


void ParkService::blinkLeft(bool on)
{
  std_msgs::Bool msg;
  msg.data = on;
  left_indicator_pub_.publish(msg);
  return;
}

void ParkService::blinkRight(bool on)
{
  std_msgs::Bool msg;
  msg.data = on;
  right_indicator_pub_.publish(msg);
  return;
}

void ParkService::reconfigureCB(selfie_park::ParkServerConfig& config, uint32_t level)
{
    if(idle_time_ != (float)config.idle_time)
    {
        idle_time_ = config.idle_time;
        ROS_INFO("idle_time_ new value: %f",idle_time_);
    }
    if(max_turn_ != (float)config.max_turn)
    {
        max_turn_ = config.max_turn;
        ROS_INFO("max_turn new value: %f",max_turn_);
    }
    if(minimal_start_parking_x_ != (float)config.minimal_start_parking_x)
    {
        minimal_start_parking_x_ = config.minimal_start_parking_x;
        ROS_INFO("minimal_start_parking_x new value: %f",minimal_start_parking_x_);
    }
    if(parking_speed_ != (float)config.parking_speed)
    {
        parking_speed_ = config.parking_speed;
        ROS_INFO("parking_speed_ new value: %f",parking_speed_);
    }
    if(angle_coeff_ != (float)config.angle_coeff)
    {
        angle_coeff_ = config.angle_coeff;
        ROS_INFO("angle coeff new value: %f", angle_coeff_);
    }
    if(iter_distance_!= (float)config.iter_distance)
    {
        iter_distance_ = config.iter_distance;
        ROS_INFO("iter distance new value: %f", iter_distance_);
    }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "park_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ParkService park(nh, pnh);
  ros::spin();

  return 0;
}
