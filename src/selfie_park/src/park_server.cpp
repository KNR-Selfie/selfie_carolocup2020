#include "../include/selfie_park/park_server.h"


ParkService::ParkService(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
  nh_(nh),
  pnh_(pnh),
  as_(nh_, "park", false)
{
  pnh_.param<std::string>("odom_topic", odom_topic_, "/odom");
  pnh_.param<std::string>("ackermann_topic", ackermann_topic_, "/drive");
  pnh_.param<float>("minimal_start_parking_x_", minimal_start_parking_x_, -0.16);
  pnh_.param<bool>("state_msgs_", state_msgs_, false);
  pnh_.param<float>("max_rot_", max_rot_, 0.8);
  pnh_.param<float>("dist_turn_", dist_turn_, 0.17);
  pnh_.param<float>("parking_speed", PARKING_SPEED, 0.4);
  move_state_ = first_phase;
  parking_state_ = not_parking;
  as_.registerGoalCallback(boost::bind(&ParkService::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&ParkService::preemptCB, this));
  as_.start();
  odom_sub_ = nh_.subscribe(odom_topic_, 10, &ParkService::odomCallback, this);
  ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(ackermann_topic_, 10);
  right_indicator_pub_ = nh_.advertise<std_msgs::Bool>("right_turn_indicator", 20);
  left_indicator_pub_ = nh_.advertise<std_msgs::Bool>("left_turn_indicator", 20);
}

geometry_msgs::Point ParkService::pointParkingToOdom(float x, float y)
{
  tf::Vector3 tf_point;
  geometry_msgs::Point odom_point;
  tf_point.setX(x);
  tf_point.setY(y);
  tf_point.setZ(0.0);
  tf_point = parking_spot_position_.transform_ * tf_point;
  odom_point.x = tf_point.x();
  odom_point.y = tf_point.y();
  return odom_point;
}

void ParkService::odomCallback(const nav_msgs::Odometry &msg)
{

  actual_odom_position_ = Position(msg);
  actual_back_odom_position_ = Position(actual_odom_position_, ODOM_TO_BACK);
  actual_front_odom_position_ = Position(actual_odom_position_, ODOM_TO_FRONT);
  actual_laser_odom_position_ = Position(actual_odom_position_, ODOM_TO_LASER);
  if (parking_state_ > not_parking)
  {
    actual_parking_position_ = Position(parking_spot_position_.transform_.inverse() * actual_odom_position_.transform_);
    actual_back_parking_position_ = Position(actual_parking_position_, ODOM_TO_BACK);
    actual_front_parking_position_ = Position(actual_parking_position_, ODOM_TO_FRONT);
    actual_laser_parking_position_ = Position(actual_parking_position_, ODOM_TO_LASER);
  }

  switch (parking_state_)
  {
    case not_parking:
      if (state_msgs_) ROS_INFO("not_parking");
      break;

    case go_to_parking_spot:
      if (toParkingSpot())
      {
        parking_state_ = going_in;
        leaving_target_ = actual_parking_position_.y_;
      }
      if (state_msgs_) ROS_INFO("go_to_parking_spot");
      blinkRight(true);
      blinkLeft(false);
      break;

    case going_in:
      if (state_msgs_) ROS_INFO("get_in");
      if (park()) parking_state_ = parked;
      break;

    case parked:
      if (state_msgs_) ROS_INFO("PARKED");
      drive(0, -MAX_TURN);
      blinkLeft(true);
      blinkRight(true);
      ros::Duration(2).sleep();
      parking_state_ = get_straight;
      break;

    case get_straight:
      if (actual_parking_position_.rot_ < 0.0)
      {
        drive(-PARKING_SPEED, -MAX_TURN);
      } else parking_state_ = go_back;
      break;

    case go_back:
      if (state_msgs_) ROS_INFO("go_back");
      blinkLeft(false);
      blinkRight(false);
      drive(-PARKING_SPEED, 0);
      if (actual_back_parking_position_.x_ < back_wall_ + max_distance_to_wall_)
      {
        drive(0, 0);
        parking_state_ = going_out;
      }
      break;

    case going_out:
      if (state_msgs_) ROS_INFO("get_out");
      if (leave()) parking_state_ = out;
      break;

    case out:
      drive(PARKING_SPEED, 0);
      selfie_msgs::parkResult result;
      result.done = true;
      as_.setSucceeded(result);
      parking_state_ = not_parking;
      break;
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
  actual_laser_parking_position_ = Position(actual_parking_position_, ODOM_TO_LASER);
  std::vector <tf::Vector3> parking_parking_spot;
  for (std::vector<geometry_msgs::Point32>::const_iterator it = msg.points.begin(); it < msg.points.end(); it++)
  {
    tf::Vector3 vec(it->x, it->y, 0);
    parking_parking_spot.push_back(actual_laser_parking_position_.transform_ * vec);
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
  parking_state_ = go_to_parking_spot;
}

void ParkService::preemptCB()
{
  ROS_INFO("parkService preempted");
  parking_state_ = not_parking;
  move_state_ = first_phase;
  as_.setPreempted();
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
  selfie_msgs::parkFeedback feedback;
  feedback.distance = actual_parking_position_.y_ - middle_of_parking_spot_y_;
  as_.publishFeedback(feedback);
  if (actual_parking_position_.x_ < back_wall_ + minimal_start_parking_x_) drive(PARKING_SPEED, 0.0);
  else return true;

  return false;
}

bool ParkService::park()
{
  selfie_msgs::parkFeedback feedback;
  feedback.distance = actual_parking_position_.y_ - middle_of_parking_spot_y_;
  as_.publishFeedback(feedback);
  switch (move_state_)
  {

    case first_phase:
      if (state_msgs_) ROS_INFO("1st phase");
      blinkRight(true);
      blinkLeft(false);
      drive(PARKING_SPEED, -MAX_TURN);
      if (actual_parking_position_.rot_ < -max_rot_)
      {
        move_state_ = straight;
      }
      if (actual_parking_position_.y_ < mid_y_)
      {
        move_state_ = second_phase;
      }
      break;

    case straight:
      if (state_msgs_) ROS_INFO("straight");
      drive(PARKING_SPEED, 0.0);
      if (actual_parking_position_.y_ < dist_turn_ + middle_of_parking_spot_y_)
      {
        move_state_ = second_phase;
      }
      break;

    case second_phase:
      if (state_msgs_) ROS_INFO("2nd phase");
      drive(PARKING_SPEED, MAX_TURN);
      if (actual_parking_position_.rot_ > 0.0 ||
          (actual_front_parking_position_.x_ > front_wall_ - max_distance_to_wall_))
      {
        move_state_ = first_phase;
        return true;
      }
      break;

  }
  return false;
}

bool ParkService::leave()
{
  switch (move_state_)
  {
    case first_phase:
      if (state_msgs_) ROS_INFO("1st phase");
      blinkRight(false);
      blinkLeft(true);
      drive(PARKING_SPEED, MAX_TURN);

      if (actual_parking_position_.rot_ > max_rot_)
      {
        move_state_ = straight;
      }
      if (actual_parking_position_.y_ > mid_y_)
      {
        move_state_ = second_phase;
      }
      break;

    case straight:
      if (state_msgs_) ROS_INFO("straight");
      drive(PARKING_SPEED, 0.0);
      if (actual_parking_position_.y_ > leaving_target_ - dist_turn_)
      {
        move_state_ = second_phase;
      }
      break;

    case second_phase:
      if (state_msgs_) ROS_INFO("2nd phase");
      drive(PARKING_SPEED, -MAX_TURN);
      if (actual_parking_position_.rot_ < 0.0)
      {
        move_state_ = first_phase;
        blinkLeft(false);
        blinkRight(false);
        return true;
      }
      break;


  }
  return false;
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
  return Position(x_ - other.x_, y_ - other.y_, rot_ - rot_);
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


void ParkService::blinkLeft(bool a)
{
  std_msgs::Bool msg;
  msg.data = a;
  left_indicator_pub_.publish(msg);
  return;
}

void ParkService::blinkRight(bool a)
{
  std_msgs::Bool msg;
  msg.data = a;
  right_indicator_pub_.publish(msg);
  return;
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