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
  pnh_.param<float>("max_rot", max_rot_, 0.8);
  pnh_.param<float>("dist_turn", dist_turn_, 0.17);
  pnh_.param<float>("parking_speed", parking_speed_, 0.4);
  pnh_.param<float>("odom_to_laser", odom_to_laser_, 0.2);
  pnh_.param<float>("odom_to_front", odom_to_front_, 0.18);
  pnh_.param<float>("odom_to_back", odom_to_back_, -0.33);
  pnh_.param<float>("max_turn", max_turn_, 0.8);
  pnh_.param<float>("idle_time", idle_time_, 2.);

  dr_server_.setCallback(dr_server_CB_);
  move_state_ = first_phase;
  parking_state_ = not_parking;
  action_status_ = READY_TO_DRIVE;
  as_.registerGoalCallback(boost::bind(&ParkService::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&ParkService::preemptCB, this));
  as_.start();
  odom_sub_ = nh_.subscribe(odom_topic_, 10, &ParkService::odomCallback, this);
  ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(ackermann_topic_, 10);
  right_indicator_pub_ = nh_.advertise<std_msgs::Bool>("right_turn_indicator", 20);
  left_indicator_pub_ = nh_.advertise<std_msgs::Bool>("left_turn_indicator", 20);
}

void ParkService::odomCallback(const nav_msgs::Odometry &msg)
{
  actual_odom_position_ = Position(msg);
  actual_laser_odom_position_ = Position(actual_odom_position_, odom_to_laser_);

  if (parking_state_ > not_parking)
  {
    actual_parking_position_ = Position(parking_spot_position_.transform_.inverse() * actual_odom_position_.transform_);
    actual_back_parking_position_ = Position(actual_parking_position_, odom_to_back_);
    actual_front_parking_position_ = Position(actual_parking_position_, odom_to_front_);

    switch (parking_state_)
    {
      case go_to_parking_spot:
        action_status_ = START_PARK;
        if (toParkingSpot())
        {
          parking_state_ = going_in;
          leaving_target_ = actual_parking_position_.y_;
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
        drive(0, -max_turn_);
        blinkLeft(true);
        blinkRight(true);
        ros::Duration(idle_time_).sleep();
        parking_state_ = get_straight;
        break;

      case get_straight:
        action_status_ = OUT_PLACE;
        blinkLeft(false);
        blinkRight(false);
        if (state_msgs_) ROS_INFO_THROTTLE(5, "get_straight");
        if (actual_parking_position_.rot_ < 0.0)
        {
          drive(-parking_speed_, -max_turn_);
        }
        else parking_state_ = go_back;
        break;

      case go_back:
        if (state_msgs_) ROS_INFO_THROTTLE(5, "go_back");
        blinkLeft(false);
        blinkRight(false);
        drive(-parking_speed_, 0);
        if (actual_back_parking_position_.x_ < back_wall_ + max_distance_to_wall_)
        {
          drive(0, 0);
          parking_state_ = going_out;
        }
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
        odom_sub_.shutdown();
        break;
    }
  }
}

void ParkService::initParkingSpot(const geometry_msgs::Polygon &msg)
{
  std::vector <tf::Vector3> odom_parking_spot;

  for (std::vector<geometry_msgs::Point32>::const_iterator it = msg.points.begin(); it < msg.points.end(); it++)
  {
    tf::Vector3 vec(it->x, it->y, 0);
    odom_parking_spot.push_back(actual_laser_odom_position_.transform_ * vec);
  }
  tf::Vector3 tl = odom_parking_spot[0];
  tf::Vector3 bl = odom_parking_spot[1];
  tf::Vector3 br = odom_parking_spot[2];
  tf::Vector3 tr = odom_parking_spot[3];

  parking_spot_position_ = Position(bl.x(), bl.y(), atan2(br.y() - bl.y(), br.x() - bl.x()));
  actual_parking_position_ = Position(parking_spot_position_.transform_.inverse() * actual_odom_position_.transform_);
  Position actual_laser_parking_position = Position(actual_parking_position_, odom_to_laser_);
  std::vector <tf::Vector3> parking_parking_spot;
  for (std::vector<geometry_msgs::Point32>::const_iterator it = msg.points.begin(); it < msg.points.end(); it++)
  {
    tf::Vector3 vec(it->x, it->y, 0);
    parking_parking_spot.push_back(actual_laser_parking_position.transform_ * vec);
  }
  tl = parking_parking_spot[0];
  bl = parking_parking_spot[1];
  br = parking_parking_spot[2];
  tr = parking_parking_spot[3];

  parking_spot_width_ = tl.y() < tr.y() ? tl.y() : tr.y();
  middle_of_parking_spot_y_ = parking_spot_width_ / 2.0;
  back_wall_ = tl.x() > bl.x() ? tl.x() : bl.x();
  front_wall_ = tr.x() < br.x() ? tr.x() : br.x();
  middle_of_parking_spot_x_ = (front_wall_ - back_wall_) / 2.0;
  mid_y_ = middle_of_parking_spot_y_ + (leaving_target_ - middle_of_parking_spot_y_) / 2.0;
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
  if (actual_parking_position_.x_ < back_wall_ + minimal_start_parking_x_)
  {
    drive(parking_speed_, 0.0);
  }
  else return true;

  return false;
}

bool ParkService::park()
{
    if(move_state_ == first_phase)
    {
        if(actual_parking_position_.y_ > middle_of_parking_spot_y_)
        {
            if(actual_front_parking_position_.x_ > front_wall_ - max_distance_to_wall_)
            {
                drive(0,max_turn_);
                move_state_ = second_phase;
                return false;
            }
            else
            {
                drive(parking_speed_, -max_turn_);
                return false;
            }

        }
        else
        {
            drive(0,0);
            return true;
        }

    }
    else
    {

        if(actual_parking_position_.y_ > middle_of_parking_spot_y_)
        {
            if(actual_back_parking_position_.x_ < back_wall_ + max_distance_to_wall_)
            {
                drive(0,-max_turn_);
                move_state_ = first_phase;
                return false;
            }
            else
            {
                drive(-parking_speed_, max_turn_);
                return false;
            }

        }
        else
        {
            drive(0,0);
            return true;
        }

    }

}

bool ParkService::leave()
{
    if(move_state_ == first_phase)
    {
        if(actual_parking_position_.y_ < leaving_target_ )
        {
            if(actual_parking_position_.x_ > front_wall_ - max_distance_to_wall_)
            {
                drive(0,max_turn_);
                move_state_ = second_phase;
                return false;
            }
            else
            {
                drive(parking_speed_, max_turn_);
                return false;
            }

        }
        else
        {
            drive(0,0);
            return true;
        }

    }
    else
    {

        if(actual_parking_position_.y_ < leaving_target_)
        {
            if(actual_parking_position_.x_ < back_wall_ + max_distance_to_wall_)
            {
                drive(0,-max_turn_);
                move_state_ = first_phase;
                return false;
            }
            else
            {
                drive(-parking_speed_, -max_turn_);
                return false;
            }

        }
        else
        {
            drive(0,0);
            return true;
        }

    }

}

float ParkService::Position::quatToRot(const geometry_msgs::Quaternion &quat)
{
  return static_cast<float>(tf::getYaw(quat));
}

ParkService::Position::Position(const nav_msgs::Odometry &msg, float offset)
{
  rot_ = quatToRot(msg.pose.pose.orientation);
  x_ = msg.pose.pose.position.x + offset * cos(rot_);
  y_ = msg.pose.pose.position.y + offset * sin(rot_);
  tf::Vector3 vec(x_, y_, 0);
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
  transform_ = tf::Transform(quat, vec);
}

ParkService::Position::Position(float x, float y, float rot) :
x_(x),
y_(y),
rot_(rot)
{
  tf::Vector3 vec(x_, y_, 0);
  tf::Quaternion quat = tf::createQuaternionFromYaw(rot_);
  transform_ = tf::Transform(quat, vec);
}

ParkService::Position ParkService::Position::operator-(const Position &other)
{
  return Position(x_ - other.x_, y_ - other.y_, rot_ - other.rot_);
}

ParkService::Position::Position(const tf::Transform &trans)
{
  x_ = trans.getOrigin().x();
  y_ = trans.getOrigin().y();
  rot_ = tf::getYaw(trans.getRotation());
  transform_ = trans;
}

ParkService::Position::Position(const Position &other, float offset)
{
  rot_ = other.rot_;
  x_ = other.x_ + cos(rot_) * offset;
  y_ = other.y_ + sin(rot_) * offset;
  tf::Quaternion quat;
  quat.setRPY(0, 0, rot_);
  tf::Vector3 vec(x_, y_, 0);
  transform_ = tf::Transform(quat, vec);
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
    if(dist_turn_ != (float)config.dist_turn)
    {
        dist_turn_ = config.dist_turn;
        ROS_INFO("dist_turn new value: %f",dist_turn_);
    }
    if(idle_time_ != (float)config.idle_time)
    {
        idle_time_ = config.idle_time;
        ROS_INFO("idle_time_ new value: %f",idle_time_);
    }
    if(max_rot_ != (float)config.max_rot)
    {
        max_rot_ = config.max_rot;
        ROS_INFO("max_rot_ new value: %f",max_rot_);
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
