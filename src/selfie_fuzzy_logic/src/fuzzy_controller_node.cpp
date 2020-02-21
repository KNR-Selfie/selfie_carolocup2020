#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <selfie_fuzzy_logic/fuzzycontroller.h>
#include <selfie_fuzzy_logic/membership.h>
#include <selfie_fuzzy_logic/rule.h>

int32_t curvature = 0;
int32_t offset = 0;

// fuzzy controller
FuzzyController* fc_turining_center = new FuzzyController();
FuzzyController* fc_steering_angle  = new FuzzyController();

Range x_curvature_in_range(0, 1800);
Range y_curvature_in_range(0, 100);
Range x_turining_center_out_range(-100, 100);
Range y_turining_center_out_range(0, 100);

Range x_offset_in_range(-200, 200);
Range y_offset_in_range(0, 100);
Range x_steering_angle_out_range(-100, 100);
Range y_steering_angle_out_range(-70, 70);

Point low_curvature_point1 (400, 100);
Point low_curvature_point2 (750, 0);

Point mid_curvature_point1 (400, 0);
Point mid_curvature_point2 (750, 100);
Point mid_curvature_point3 (1050, 100);
Point mid_curvature_point4 (1400, 0);

Point high_curvature_point1 (1050, 0);
Point high_curvature_point2 (1400, 100);

Point right_offset_point1 (-50, 100);
Point right_offset_point2 (0, 0);

Point center_offset_point1 (-50, 0);
Point center_offset_point2 (0, 100);
Point center_offset_point3 (50, 0);

Point left_offset_point1 (0, 0);
Point left_offset_point2 (50, 100);

Point turining_center_front_3_point (0, 100);
Point turining_center_front_2_point (0, 50);
Point turining_center_front_1_point (0, 0);

Point steering_angle_strong_left_point (0, 70);
Point steering_angle_left_point (0, 35);
Point steering_angle_center_point (0, 0);
Point steering_angle_right_point (0, -35);
Point steering_angle_strong_right_point (0, -70);

Membership *low_curvature = new Membership(x_curvature_in_range, y_curvature_in_range);
Membership *mid_curvature = new Membership(x_curvature_in_range, y_curvature_in_range);
Membership *high_curvature = new Membership(x_curvature_in_range, y_curvature_in_range);

Membership *turining_center_front_3 = new Membership(x_turining_center_out_range, y_turining_center_out_range);
Membership *turining_center_front_2 = new Membership(x_turining_center_out_range, y_turining_center_out_range);
Membership *turining_center_front_1 = new Membership(x_turining_center_out_range, y_turining_center_out_range);

Membership *left_offset = new Membership(x_offset_in_range, y_offset_in_range);
Membership *center_offset = new Membership(x_offset_in_range, y_offset_in_range);
Membership *right_offset = new Membership(x_offset_in_range, y_offset_in_range);

Membership *steering_angle_strong_left = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);
Membership *steering_angle_left = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);
Membership *steering_angle_center = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);
Membership *steering_angle_right = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);
Membership *steering_angle_strong_right = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);

void setup();
void addRules();

void curvatureCallback(const std_msgs::Float64 &msg)
{
  curvature = msg.data * 1000;
}

void combinedOffsetCallback(const std_msgs::Float64 &msg)
{
  offset = msg.data * 1000;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "selfie_fuzzy_controller");

  ros::NodeHandle n("~");

  ros::Publisher steering_balance_pub = n.advertise<std_msgs::Float32>("/steering_balance", 50);
  ros::Publisher steering_angle_pub = n.advertise<std_msgs::Float64>("/steering_angle", 50);

  ros::Subscriber sub_curvature = n.subscribe("/curvature", 50, curvatureCallback);
  ros::Subscriber sub_combined_offset = n.subscribe("/combined_offset", 50, combinedOffsetCallback);

  setup();
  addRules();

  ros::Rate loop_rate(100);

  std_msgs::Float32 steering_balance_msg;
  std_msgs::Float64 steering_angle_msg;

  while (n.ok())
  {
    // check for incoming messages
    ros::spinOnce();

    steering_balance_msg.data = fc_turining_center->getOut() / 100;
    steering_balance_pub.publish(steering_balance_msg);

    steering_angle_msg.data = fc_steering_angle->getOut() / 100;
    steering_angle_pub.publish(steering_angle_msg);

    loop_rate.sleep();
  }
}

void setup()
{
  low_curvature->addPoint(low_curvature_point1);
  low_curvature->addPoint(low_curvature_point2);

  mid_curvature->addPoint(mid_curvature_point1);
  mid_curvature->addPoint(mid_curvature_point2);
  mid_curvature->addPoint(mid_curvature_point3);
  mid_curvature->addPoint(mid_curvature_point4);

  high_curvature->addPoint(high_curvature_point1);
  high_curvature->addPoint(high_curvature_point2);

  turining_center_front_3->addPoint(turining_center_front_3_point);
  turining_center_front_2->addPoint(turining_center_front_2_point);
  turining_center_front_1->addPoint(turining_center_front_1_point);

  left_offset->addPoint(left_offset_point1);
  left_offset->addPoint(left_offset_point2);

  center_offset->addPoint(center_offset_point1);
  center_offset->addPoint(center_offset_point2);
  center_offset->addPoint(center_offset_point3);

  right_offset->addPoint(right_offset_point1);
  right_offset->addPoint(right_offset_point2);

  steering_angle_strong_left->addPoint(steering_angle_strong_left_point);
  steering_angle_left->addPoint(steering_angle_left_point);
  steering_angle_center->addPoint(steering_angle_center_point);
  steering_angle_right->addPoint(steering_angle_right_point);
  steering_angle_strong_right->addPoint(steering_angle_strong_right_point);
}

void addRules()
{
  Rule *r1 = new Rule();
  r1->addInput(curvature, *low_curvature);
  r1->addOutput(*turining_center_front_3);

  Rule *r2 = new Rule();
  r2->addInput(curvature, *mid_curvature);
  r2->addOutput(*turining_center_front_2);

  Rule *r3 = new Rule();
  r3->addInput(curvature, *high_curvature);
  r3->addOutput(*turining_center_front_1);

  fc_turining_center->addRule(*r1);
  fc_turining_center->addRule(*r2);
  fc_turining_center->addRule(*r3);

  Rule *r4 = new Rule();
  r4->addInput(curvature, *low_curvature);
  r4->addInput(offset, *center_offset);
  r4->addOutput(*steering_angle_center);

  Rule *r5 = new Rule();
  r5->addInput(curvature, *low_curvature);
  r5->addInput(offset, *left_offset);
  r5->addOutput(*steering_angle_left);

  Rule *r6 = new Rule();
  r6->addInput(curvature, *mid_curvature);
  r6->addInput(offset, *left_offset);
  r6->addOutput(*steering_angle_left);

  Rule *r7 = new Rule();
  r7->addInput(curvature, *high_curvature);
  r7->addInput(offset, *left_offset);
  r7->addOutput(*steering_angle_strong_left);

  Rule *r8 = new Rule();
  r8->addInput(curvature, *low_curvature);
  r8->addInput(offset, *right_offset);
  r8->addOutput(*steering_angle_right);

  Rule *r9 = new Rule();
  r9->addInput(curvature, *mid_curvature);
  r9->addInput(offset, *right_offset);
  r9->addOutput(*steering_angle_right);

  Rule *r10 = new Rule();
  r10->addInput(curvature, *high_curvature);
  r10->addInput(offset, *right_offset);
  r10->addOutput(*steering_angle_strong_right);

  fc_steering_angle->addRule(*r4);
  fc_steering_angle->addRule(*r5);
  fc_steering_angle->addRule(*r6);
  fc_steering_angle->addRule(*r7);
  fc_steering_angle->addRule(*r8);
  fc_steering_angle->addRule(*r9);
  fc_steering_angle->addRule(*r10);
}