#include "../include/selfie_park/park_server.h"


ParkService::ParkService(const ros::NodeHandle &nh, const ros::NodeHandle &pnh):
nh_(nh),
pnh_(pnh),
as_(nh_, "park",  false),
visualize(true)
{
  pnh_.param<std::string>("odom_topic", odom_topic,"/odom");
  pnh_.param<std::string>("ackermann_topic", ackermann_topic,"/drive");
  pnh_.param<float>("minimal_start_parking_x", minimal_start_parking_x, -0.16);
  pnh_.param<float>("maximal_start_parking_x", maximal_start_parking_x, 0.0);
  pnh_.param<float>("traffic_lane_marigin",traffic_lane_marigin, 0.05);
  pnh_.param<float>("earlier_turn", earlier_turn, 0.01);
  pnh_.param<float>("first_to_second_phase_x_frontwards",first_to_second_phase_x_frontwards, 0.9/2.0);
  pnh_.param<float>("first_to_second_phase_x_backwards", first_to_second_phase_x_backwards, 0.9/2.0);
  pnh_.param<bool>("state_msgs",state_msgs, true);
  pnh_.param<float>("max_distance_to_wall", max_distance_to_wall, 0.03);
  pnh_.param<float>("max_rot", max_rot, 0.8);
  pnh_.param<float>("dist_turn", dist_turn, 0.17);
  move_state = first_phase;
  parking_state = not_parking;
  as_.registerGoalCallback(boost::bind(&ParkService::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&ParkService::preemptCB, this));
  as_.start();
  odom_sub = nh_.subscribe(odom_topic, 10, &ParkService::odom_callback, this);
  ackermann_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(ackermann_topic, 10);
  right_indicator_pub = nh_.advertise<std_msgs::Bool>("right_turn_indicator", 20);
  left_indicator_pub = nh_.advertise<std_msgs::Bool>("left_turn_indicator", 20);
	if(visualize) visualization_pub = nh_.advertise<visualization_msgs::MarkerArray>("parking_view", 10);
	mid_y = 0.0;
	mid_x = 0.0;

	std::cout<<"start"<<std::endl;
}
void ParkService::visualize_parking_spot()
{
	/*
	tf::Vector3 tf_point;
	geometry_msgs::Point marker_point;
	marker_point.z = 0;
	visualization_msgs::MarkerArray markers;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/odom";
	marker.header.stamp = ros::Time::now();
	marker.ns = "edges";
	marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
	marker.color.r = 1.0f;
  marker.color.g = 120.0f/255.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  marker.scale.x = 0.006;
  marker.scale.y = 0.006;
  marker.id = 0;
	marker.points.push_back(point_parking_to_odom(back_wall, parking_spot_width));
	marker.points.push_back(point_parking_to_odom(front_wall, parking_spot_width));
	marker.points.push_back(point_parking_to_odom(back_wall, 0.0));
	marker.points.push_back(point_parking_to_odom(front_wall, 0.0));
	markers.markers.push_back(marker);
	

  visualization_msgs::Marker marker2;
	marker2.header.frame_id = "/odom";
	marker2.header.stamp = ros::Time::now();
	marker2.ns = "wall_lines";
	marker2.type = visualization_msgs::Marker::LINE_LIST;
  marker2.action = visualization_msgs::Marker::ADD;
	marker2.color.r = 1.0f;
  marker2.color.g = 0.0f;
  marker2.color.b = 0.0f;
  marker2.color.a = 1.0f;
  marker2.scale.x = 0.006;
  marker2.scale.y = 0.006;
	marker2.id = 1;
	marker2.points.push_back(point_parking_to_odom(back_wall, parking_spot_width));
	marker2.points.push_back(point_parking_to_odom(back_wall, 0.0));
	marker2.points.push_back(point_parking_to_odom(front_wall, parking_spot_width));
	marker2.points.push_back(point_parking_to_odom(front_wall, 0.0));

	markers.markers.push_back(marker2);


	  visualization_msgs::Marker marker3;
	marker3.header.frame_id = "/odom";
	marker3.header.stamp = ros::Time::now();
	marker3.ns = "turn_lines";
	marker3.type = visualization_msgs::Marker::LINE_LIST;
  marker3.action = visualization_msgs::Marker::ADD;
	marker3.color.r = 0.5f;
  marker3.color.g = 1.0f;
  marker3.color.b = 0.0f;
  marker3.color.a = 1.0f;
  marker3.scale.x = 0.006;
  marker3.scale.y = 0.006;
	marker3.id = 1;
	marker3.points.push_back(point_parking_to_odom(back_wall, mid_y));
	marker3.points.push_back(point_parking_to_odom(front_wall, mid_y));
	marker3.points.push_back(point_parking_to_odom(mid_x, 0.0));
	marker3.points.push_back(point_parking_to_odom(mid_x, parking_spot_width));
	markers.markers.push_back(marker3);


	visualization_msgs::Marker marker4;
	marker4.header.frame_id = "/odom";
	marker4.header.stamp = ros::Time::now();
	marker4.ns = "odom_pos";
	marker4.type = visualization_msgs::Marker::LINE_LIST;
  marker4.action = visualization_msgs::Marker::ADD;
	marker4.color.r = 1.0f;
  marker4.color.g = 1.0f;
  marker4.color.b = 1.0f;
  marker4.color.a = 1.0f;
  marker4.scale.x = 0.006;
  marker4.scale.y = 0.006;
	marker4.id = 1;
	geometry_msgs::Point car_point;
	car_point.z = 0.0;
	car_point.x = actual_back_odom_position.x - CAR_WIDTH*sin(actual_back_odom_position.rot)/2.0;
	car_point.y = actual_back_odom_position.y + CAR_WIDTH*cos(actual_back_odom_position.rot)/2.0;
	marker4.points.push_back(car_point);
	car_point.x = actual_back_odom_position.x + CAR_WIDTH*sin(actual_back_odom_position.rot)/2.0;
	car_point.y = actual_back_odom_position.y - CAR_WIDTH*cos(actual_back_odom_position.rot)/2.0;
	marker4.points.push_back(car_point);
	marker4.points.push_back(car_point);
	car_point.x = actual_front_odom_position.x + CAR_WIDTH*sin(actual_back_odom_position.rot)/2.0;
	car_point.y = actual_front_odom_position.y - CAR_WIDTH*cos(actual_back_odom_position.rot)/2.0;
	marker4.points.push_back(car_point);
	marker4.points.push_back(car_point);
	car_point.x = actual_front_odom_position.x - CAR_WIDTH*sin(actual_back_odom_position.rot)/2.0;
	car_point.y = actual_front_odom_position.y + CAR_WIDTH*cos(actual_back_odom_position.rot)/2.0;
	marker4.points.push_back(car_point);
	marker4.points.push_back(car_point);
	car_point.x = actual_back_odom_position.x - CAR_WIDTH*sin(actual_back_odom_position.rot)/2.0;
	car_point.y = actual_back_odom_position.y + CAR_WIDTH*cos(actual_back_odom_position.rot)/2.0;
	marker4.points.push_back(car_point);
	
	markers.markers.push_back(marker4);

	visualization_pub.publish(markers);*/
}

geometry_msgs::Point ParkService::point_parking_to_odom(float x, float y)
{
	tf::Vector3 tf_point;
	geometry_msgs::Point odom_point;
	tf_point.setX(x);
	tf_point.setY(y);
	tf_point.setZ(0.0);
	tf_point = parking_spot_position.transform * tf_point;
	odom_point.x = tf_point.x();
  odom_point.y = tf_point.y();
	return odom_point;
}

void ParkService::odom_callback(const nav_msgs::Odometry &msg)
{
	
  actual_odom_position = Position(msg);
  actual_back_odom_position = Position(actual_odom_position, ODOM_TO_BACK);
  actual_front_odom_position = Position(actual_odom_position, ODOM_TO_FRONT);
  actual_laser_odom_position = Position(actual_odom_position, ODOM_TO_LASER);
  
  //std::cout<<actual_odom_position.x<<"  "<<actual_odom_position.y<<"  "<<actual_back_odom_position.x<<"  "<<actual_back_odom_position.y<<"  "<<actual_front_odom_position.x<<"  "<<actual_front_odom_position.y<<std::endl;

   if(parking_state>not_parking)
   {
		
    std::cout<<actual_parking_position.x<<"  "<<actual_parking_position.y<<std::endl;
    actual_parking_position = Position(parking_spot_position.transform.inverse()*actual_odom_position.transform);
    actual_back_parking_position = Position(actual_parking_position, ODOM_TO_BACK);
    actual_front_parking_position = Position(actual_parking_position, ODOM_TO_FRONT);
    actual_laser_parking_position = Position(actual_parking_position, ODOM_TO_LASER);
		if(visualize) visualize_parking_spot();
   }

  switch(parking_state)
	{
		case not_parking:
		if(state_msgs) ROS_INFO("not_parking");
		break;

		case go_to_parking_spot:
		if(to_parking_spot()){parking_state = going_in;}
		if(state_msgs) ROS_INFO("go_to_parking_spot");
		blink_right(true);
		blink_left(false);

		break;

		case going_in:
		if(state_msgs) ROS_INFO("get_in");
		if(park()) parking_state = parked;
		break;

		case parked:
		if(state_msgs) ROS_INFO("PARKED");
	
		drive(0, -MAX_TURN);
		blink_left(true);
		blink_right(true);
		ros::Duration(2).sleep();
		parking_state = get_straight;
		break;

		case get_straight:
		if(actual_parking_position.rot < 0.0)
		{
			drive(-PARKING_SPEED, -MAX_TURN);
		}
		else parking_state = go_back;
		break;


		case go_back:
		ROS_INFO("go_back");
		blink_left(false);
		blink_right(false);
		drive(-PARKING_SPEED,0);
		if(actual_back_parking_position.x < back_wall +max_distance_to_wall) 
		{
			drive(0,0);
			parking_state = going_out;
		}
		break;

		case going_out:
		if(state_msgs) ROS_INFO("get_out");
		if(leave()) parking_state = out;
		break;
		case out:
		drive(PARKING_SPEED,0);
		selfie_park::parkResult result;
		result.done = true;
		as_.setSucceeded(result);
		parking_state = not_parking;
		break;
	}
}

void ParkService::init_parking_spot(const geometry_msgs::Polygon &msg)
{
  std::vector<tf::Vector3> odom_parking_spot;
  
  for(std::vector<geometry_msgs::Point32>::const_iterator it = msg.points.begin(); it<msg.points.end(); it++)
  {
    tf::Vector3 vec(it->x, it->y, 0);
    odom_parking_spot.push_back(actual_laser_odom_position.transform*vec);
  }
  tf::Vector3 tl = odom_parking_spot[0];

  tf::Vector3 bl = odom_parking_spot[1];
  tf::Vector3 br = odom_parking_spot[2];
  tf::Vector3 tr = odom_parking_spot[3];
	std::cout<<" tl "<<tl.x()<<"  "<<tl.y()<<" bl "<<bl.x()<<"  "<<bl.y()<<" br "<<br.x()<<"  "<<br.y()<<" tr "<<tr.x()<<"  "<<tr.y()<<std::endl;
  parking_spot_position = Position(bl.x(), bl.y(), atan2(br.y()-bl.y(), br.x()-bl.x()));
  actual_parking_position = Position(parking_spot_position.transform.inverse()*actual_odom_position.transform);
	actual_laser_parking_position = Position(actual_parking_position, ODOM_TO_LASER);
  std::vector<tf::Vector3> parking_parking_spot;
  for(std::vector<geometry_msgs::Point32>::const_iterator it = msg.points.begin(); it<msg.points.end(); it++)
  {
    tf::Vector3 vec(it->x, it->y, 0);
    parking_parking_spot.push_back(actual_laser_parking_position.transform*vec);
  }
  tl = parking_parking_spot[0];
  bl = parking_parking_spot[1];
  br = parking_parking_spot[2];
  tr = parking_parking_spot[3];
	std::cout<<" tl "<<tl.x()<<"  "<<tl.y()<<" bl "<<bl.x()<<"  "<<bl.y()<<" br "<<br.x()<<"  "<<br.y()<<" tr "<<tr.x()<<"  "<<tr.y()<<std::endl;
  parking_spot_width = tl.y()<tr.y()?tl.y():tr.y();
  middle_of_parking_spot_y = parking_spot_width/2.0;
  back_wall = tl.x()>bl.x()?tl.x():bl.x();
  front_wall = tr.x()<br.x()?tr.x():br.x();
  middle_of_parking_spot_x = (front_wall - back_wall)/2.0;
  leaving_target = actual_parking_position.y;
	std::cout<<"parking spot position  "<<parking_spot_position.x<<"  "<<parking_spot_position.y<<std::endl;
	mid_y = middle_of_parking_spot_y + (leaving_target - middle_of_parking_spot_y)/2.0;
}
void ParkService::goalCB()
{
  std::cout<<"got goal"<<std::endl;
  selfie_park::parkGoal goal = *as_.acceptNewGoal();
  init_parking_spot(goal.parking_spot);
  parking_state = go_to_parking_spot;
}

void ParkService::preemptCB()
{
  ROS_INFO("parkService preempted");
  parking_state = not_parking;
  as_.setPreempted();
}

float ParkService::front_distance()
{
  return front_wall - actual_front_parking_position.x;// - sin(actual_parking_position.rot) * CAR_WIDTH/2.0;

}

float ParkService::back_distance()
{
  return actual_back_parking_position.x - back_wall;// - sin(actual_parking_position.rot) * CAR_WIDTH/2.0;
}

void ParkService::drive(float speed, float steering_angle)
{
	ackermann_msgs::AckermannDriveStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.drive.speed = speed;
	msg.drive.steering_angle = steering_angle;
	ackermann_pub.publish(msg);
}

//void go()
bool ParkService::in_parking_spot()
{
	if(actual_back_parking_position.y < middle_of_parking_spot_y + traffic_lane_marigin && actual_back_parking_position.y > middle_of_parking_spot_y - traffic_lane_marigin &&
  actual_front_parking_position.y < middle_of_parking_spot_y + traffic_lane_marigin && actual_front_parking_position.y > middle_of_parking_spot_y - traffic_lane_marigin)
  {return true;}
  else return false;
}
bool ParkService::in_traffic_lane()
{
	
  if(actual_back_parking_position.y < leaving_target + traffic_lane_marigin && actual_back_parking_position.y > leaving_target - traffic_lane_marigin &&
  actual_front_parking_position.y < leaving_target + traffic_lane_marigin && actual_front_parking_position.y > leaving_target - traffic_lane_marigin)
  {return true;}
  else return false;
}
bool ParkService::to_parking_spot()
{
  selfie_park::parkFeedback feedback;
  feedback.distance = actual_parking_position.y - middle_of_parking_spot_y;
  as_.publishFeedback(feedback);
  if(actual_parking_position.x < back_wall + minimal_start_parking_x) drive(PARKING_SPEED, 0.0);
  else return true;

  return false;
}

bool ParkService::park()
{
  selfie_park::parkFeedback feedback;
  feedback.distance = actual_parking_position.y - middle_of_parking_spot_y;
  as_.publishFeedback(feedback);
  //std::cout<<"back pos  "<<actual_back_parking_position.x<<"  back wall  "<<back_wall<<"  front pos  "<<actual_front_parking_position.x<<"  front wall  "<<front_wall<<std::endl;
	switch(move_state)
	{
		
		case first_phase:
		
		ROS_INFO("1st phase");
		blink_right(true);
		blink_left(false);
		drive(PARKING_SPEED, -MAX_TURN);
		if(actual_parking_position.rot < -max_rot){move_state = straight;}
		if(actual_parking_position.y < mid_y) {move_state =second_phase;}
		break;
		case straight:
		ROS_INFO("straight");
		drive(PARKING_SPEED, 0.0);
		if(actual_parking_position.y < dist_turn + middle_of_parking_spot_y){move_state = second_phase;}
		break;
		case second_phase:
		ROS_INFO("2nd phase");
		drive(PARKING_SPEED, MAX_TURN);
		if(actual_parking_position.rot > 0.0 || actual_front_parking_position.x > front_wall -max_distance_to_wall)
		{
			
			move_state = first_phase;
			return true;
		}
		break;
		
	
	}
	return false;
}

bool ParkService::leave()
{
	switch(move_state)
	{
		case first_phase:
		ROS_INFO("1st phase");
		blink_right(false);
		blink_left(true);
		drive(PARKING_SPEED, MAX_TURN);
		
		if(actual_parking_position.rot > max_rot){move_state = straight;}
		if(actual_parking_position.y > mid_y) {move_state =second_phase;}
		break;
		case straight:
		ROS_INFO("straight");
		drive(PARKING_SPEED, 0.0);
		if(actual_parking_position.y > leaving_target - dist_turn){move_state = second_phase;}
		break;
		case second_phase:
		ROS_INFO("2nd phase");
		drive(PARKING_SPEED, -MAX_TURN);
		if(actual_parking_position.rot < 0.0)
		{
			
			move_state = first_phase;
			return true;
		}
		break;
		
	
	}
	return false;
}

float ParkService::Position::quat_to_rot(const geometry_msgs::Quaternion &quat)
{
  //std::cout<<tf::getYaw(quat)<<std::endl;
  return static_cast<float>(tf::getYaw(quat));
}

ParkService::Position::Position(const nav_msgs::Odometry &msg, float offset)
{
    rot = quat_to_rot(msg.pose.pose.orientation);
    x = msg.pose.pose.position.x + offset*cos(rot);
    y = msg.pose.pose.position.y + offset*sin(rot);
    tf::Vector3 vec(x,y,0);
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
    transform = tf::Transform(quat, vec);
}
ParkService::Position::Position(float x_, float y_, float rot_):
x(x_),
y(y_),
rot(rot_)
{
    tf::Vector3 vec(x,y,0);
    tf::Quaternion quat = tf::createQuaternionFromYaw(rot);
    transform = tf::Transform(quat, vec);
}
ParkService::Position ParkService::Position::operator-(const Position &other)
{
  return Position(this->x - other.x, this->y - other.y, this->rot - other.rot);
}

ParkService::Position::Position(const tf::Transform &trans)
{
    x = trans.getOrigin().x();
    y = trans.getOrigin().y();
    rot = tf::getYaw(trans.getRotation());
    transform = trans;
}
ParkService::Position::Position(const Position &other, float offset)
{
  rot = other.rot;
  x = other.x + cos(rot) * offset;
  y = other.y + sin(rot) * offset;
  tf::Quaternion quat;
  quat.setRPY(0,0,rot);
  tf::Vector3 vec(x,y,0);
  transform = tf::Transform(quat, vec);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "park_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ParkService park(nh, pnh);
  ros::spin();

  return 0;
}

void ParkService::blink_left(bool a)
{
	std_msgs::Bool msg;
	msg.data = a;
	left_indicator_pub.publish(msg);
	return;
}

void ParkService::blink_right(bool a)
{
	std_msgs::Bool msg;
	msg.data = a;
	right_indicator_pub.publish(msg);
	return;
}