#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <selfie_fuzzy_logic/fuzzycontroller.h>
#include <selfie_fuzzy_logic/membership.h>
#include <selfie_fuzzy_logic/rule.h>

int32_t curvature = 0;
int32_t offset = 0;
int32_t delta_offset = 0;

// fuzzy controller
FuzzyController* fc_turining_center = new FuzzyController();
FuzzyController* fc_steering_angle  = new FuzzyController();

Range x_curvature_in_range(0, 1800);
Range y_curvature_in_range(0, 100);
Range x_turining_center_out_range(-100, 100);
Range y_turining_center_out_range(0, 100);

Range x_offset_in_range(-200, 200);
Range y_offset_in_range(0, 100);

Range x_delta_offset_in_range(-20, 20);
Range y_delta_offset_in_range(0, 100);

Range x_steering_angle_out_range(-100, 100);
Range y_steering_angle_out_range(-70, 70);

Range x_steering_delta_angle_out_range(-20, 20);
Range y_steering_delta_angle_out_range(-15, 15);


///////////////////

Point low_curvature_point1 (400, 100);
Point low_curvature_point2 (900, 0);

Point mid_curvature_point1 (200, 0);
Point mid_curvature_point2 (750, 100);
Point mid_curvature_point3 (1050, 100);
Point mid_curvature_point4 (1600, 0);

Point high_curvature_point1 (900, 0);
Point high_curvature_point2 (1400, 100);

//////////////////

Point right_offset_strong_point1 (-130, 100);
Point right_offset_strong_point2 (-50, 0);

Point right_offset_low_point1 (-130, 0);
Point right_offset_low_point2 (-100, 100);
Point right_offset_low_point3 (-50, 100);
Point right_offset_low_point4 (10, 0);

Point center_offset_point1 (-80, 0);
Point center_offset_point2 (-10, 100);
Point center_offset_point3 (10, 100);
Point center_offset_point4 (80, 0);

Point left_offset_low_point1 (10, 0);
Point left_offset_low_point2 (50, 100);
Point left_offset_low_point3 (100, 100);
Point left_offset_low_point4 (130, 0);

Point left_offset_strong_point1 (50, 0);
Point left_offset_strong_point2 (130, 100);

/////////////////

Point right_delta_offset_strong_point1 (-12, 100);
Point right_delta_offset_strong_point2 (-5, 0);

Point right_delta_offset_low_point1 (-12, 0);
Point right_delta_offset_low_point2 (-5, 100);
Point right_delta_offset_low_point3 (-4, 100);
Point right_delta_offset_low_point4 (0, 0);

Point center_delta_offset_point1 (-5, 0);
Point center_delta_offset_point2 (-1, 100);
Point center_delta_offset_point3 (1, 100);
Point center_delta_offset_point4 (5, 0);

Point left_delta_offset_low_point1 (0, 0);
Point left_delta_offset_low_point2 (4, 100);
Point left_delta_offset_low_point3 (5, 100);
Point left_delta_offset_low_point4 (12, 0);

Point left_delta_offset_strong_point1 (5, 0);
Point left_delta_offset_strong_point2 (12, 100);

////////////////////

Point turining_center_front_3_point (0, 100);
Point turining_center_front_2_point (0, 75);
Point turining_center_front_1_point (0, 50);

/////////////////////////

Point steering_delta_angle_high_left_point (0, 10);
Point steering_delta_angle_mid_left_point (0, 5);
Point steering_delta_angle_low_left_point (0, 3);
Point steering_delta_angle_center_point (0, 0);
Point steering_delta_angle_low_right_point (0, -3);
Point steering_delta_angle_mid_right_point (0, -5);
Point steering_delta_angle_high_right_point (0, -10);

/////////////////////////////////

Point steering_angle_high_left_point (0, 70);
Point steering_angle_mid_left_point (0, 40);
Point steering_angle_low_left_point (0, 15);
Point steering_angle_center_point (0, 0);
Point steering_angle_low_right_point (0, -15);
Point steering_angle_mid_right_point (0, -40);
Point steering_angle_high_right_point (0, -70);

///////////////////////////////

Point steering_angle_mid_left_cur_point (0, 45);
Point steering_angle_low_left_cur_point (0, 30);
Point steering_angle_low_right_cur_point (0, -30);
Point steering_angle_mid_right_cur_point (0, -45);

////////////////////////////

Membership *low_curvature = new Membership(x_curvature_in_range, y_curvature_in_range);
Membership *mid_curvature = new Membership(x_curvature_in_range, y_curvature_in_range);
Membership *high_curvature = new Membership(x_curvature_in_range, y_curvature_in_range);

Membership *turining_center_front_3 = new Membership(x_turining_center_out_range, y_turining_center_out_range);
Membership *turining_center_front_2 = new Membership(x_turining_center_out_range, y_turining_center_out_range);
Membership *turining_center_front_1 = new Membership(x_turining_center_out_range, y_turining_center_out_range);

Membership *left_offset_strong = new Membership(x_offset_in_range, y_offset_in_range);
Membership *left_offset_low = new Membership(x_offset_in_range, y_offset_in_range);
Membership *center_offset = new Membership(x_offset_in_range, y_offset_in_range);
Membership *right_offset_low = new Membership(x_offset_in_range, y_offset_in_range);
Membership *right_offset_strong = new Membership(x_offset_in_range, y_offset_in_range);

Membership *left_delta_offset_strong = new Membership(x_delta_offset_in_range, y_delta_offset_in_range);
Membership *left_delta_offset_low = new Membership(x_delta_offset_in_range, y_delta_offset_in_range);
Membership *center_delta_offset = new Membership(x_delta_offset_in_range, y_delta_offset_in_range);
Membership *right_delta_offset_low = new Membership(x_delta_offset_in_range, y_delta_offset_in_range);
Membership *right_delta_offset_strong = new Membership(x_delta_offset_in_range, y_delta_offset_in_range);

Membership *steering_angle_high_left = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);
Membership *steering_angle_mid_left = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);
Membership *steering_angle_low_left = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);
Membership *steering_angle_center = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);
Membership *steering_angle_low_right = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);
Membership *steering_angle_mid_right = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);
Membership *steering_angle_high_right = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);

Membership *steering_delta_angle_high_left = new Membership(x_steering_delta_angle_out_range, y_steering_delta_angle_out_range);
Membership *steering_delta_angle_mid_left = new Membership(x_steering_delta_angle_out_range, y_steering_delta_angle_out_range);
Membership *steering_delta_angle_low_left = new Membership(x_steering_delta_angle_out_range, y_steering_delta_angle_out_range);
Membership *steering_delta_angle_center = new Membership(x_steering_delta_angle_out_range, y_steering_delta_angle_out_range);
Membership *steering_delta_angle_low_right = new Membership(x_steering_delta_angle_out_range, y_steering_delta_angle_out_range);
Membership *steering_delta_angle_mid_right = new Membership(x_steering_delta_angle_out_range, y_steering_delta_angle_out_range);
Membership *steering_delta_angle_high_right = new Membership(x_steering_delta_angle_out_range, y_steering_delta_angle_out_range);

Membership *steering_angle_mid_left_cur = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);
Membership *steering_angle_low_left_cur = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);
Membership *steering_angle_low_right_cur = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);
Membership *steering_angle_mid_right_cur = new Membership(x_steering_angle_out_range, y_steering_angle_out_range);

void setup();
void addRules();

void curvatureCallback(const std_msgs::Float64 &msg)
{
  curvature = msg.data * 1000;
}

void combinedOffsetCallback(const std_msgs::Float64 &msg)
{
  delta_offset = offset - msg.data * 1000;
  offset = msg.data * 1000;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "selfie_fuzzy_controller");

  ros::NodeHandle n("~");

  ros::Publisher steering_balance_pub = n.advertise<std_msgs::Float32>("/steering_balance", 50);
  ros::Publisher steering_angle_pub = n.advertise<std_msgs::Float64>("/steering_angle", 50);
  ros::Publisher delta_in_steering_angle_pub = n.advertise<std_msgs::Float64>("/delta_in_steering_angle", 50);
  ros::Publisher delta_out_steering_angle_pub = n.advertise<std_msgs::Float64>("/delta_out_steering_angle", 50);

  ros::Subscriber sub_curvature = n.subscribe("/curvature", 50, curvatureCallback);
  ros::Subscriber sub_combined_offset = n.subscribe("/combined_offset", 50, combinedOffsetCallback);

  setup();
  addRules();

  ros::Rate loop_rate(100);

  std_msgs::Float32 steering_balance_msg;
  std_msgs::Float64 steering_angle_msg;
  std_msgs::Float64 delta_in_steering_angle_msg;
  std_msgs::Float64 delta_out_steering_angle_msg;
  float steering_angle = 0;
  float delta_out = 0;

  while (n.ok())
  {
    // check for incoming messages
    ros::spinOnce();

    steering_balance_msg.data = fc_turining_center->getOut() / 100;
    steering_balance_pub.publish(steering_balance_msg);

    delta_out = fc_steering_angle->getOut();
    steering_angle += delta_out / 100;
    if (steering_angle > 0.7)
      steering_angle = 0.7;
    else if (steering_angle < -0.7)
      steering_angle = -0.7;

    steering_angle_msg.data = steering_angle;
    steering_angle_pub.publish(steering_angle_msg);

    delta_in_steering_angle_msg.data = delta_offset;
    delta_in_steering_angle_pub.publish(delta_in_steering_angle_msg);
    
    delta_out_steering_angle_msg.data = delta_out;
    delta_out_steering_angle_pub.publish(delta_out_steering_angle_msg);

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

  ///////////////////////

  left_offset_strong->addPoint(left_offset_strong_point1);
  left_offset_strong->addPoint(left_offset_strong_point2);

  left_offset_low->addPoint(left_offset_low_point1);
  left_offset_low->addPoint(left_offset_low_point2);
  left_offset_low->addPoint(left_offset_low_point3);
  left_offset_low->addPoint(left_offset_low_point4);

  center_offset->addPoint(center_offset_point1);
  center_offset->addPoint(center_offset_point2);
  center_offset->addPoint(center_offset_point3);
  center_offset->addPoint(center_offset_point4);

  right_offset_low->addPoint(right_offset_low_point1);
  right_offset_low->addPoint(right_offset_low_point2);
  right_offset_low->addPoint(right_offset_low_point3);
  right_offset_low->addPoint(right_offset_low_point4);

  right_offset_strong->addPoint(right_offset_strong_point1);
  right_offset_strong->addPoint(right_offset_strong_point2);

  ///////////////////////

  left_delta_offset_strong->addPoint(left_delta_offset_strong_point1);
  left_delta_offset_strong->addPoint(left_delta_offset_strong_point2);

  left_delta_offset_low->addPoint(left_delta_offset_low_point1);
  left_delta_offset_low->addPoint(left_delta_offset_low_point2);
  left_delta_offset_low->addPoint(left_delta_offset_low_point3);
  left_delta_offset_low->addPoint(left_delta_offset_low_point4);

  center_delta_offset->addPoint(center_delta_offset_point1);
  center_delta_offset->addPoint(center_delta_offset_point2);
  center_delta_offset->addPoint(center_delta_offset_point3);
  center_delta_offset->addPoint(center_delta_offset_point4);

  right_delta_offset_low->addPoint(right_delta_offset_low_point1);
  right_delta_offset_low->addPoint(right_delta_offset_low_point2);
  right_delta_offset_low->addPoint(right_delta_offset_low_point3);
  right_delta_offset_low->addPoint(right_delta_offset_low_point4);

  right_delta_offset_strong->addPoint(right_delta_offset_strong_point1);
  right_delta_offset_strong->addPoint(right_delta_offset_strong_point2);

  /////////////////////////////////

  steering_angle_high_left->addPoint(steering_angle_high_left_point);
  steering_angle_mid_left->addPoint(steering_angle_mid_left_point);
  steering_angle_low_left->addPoint(steering_angle_low_left_point);
  steering_angle_center->addPoint(steering_angle_center_point);
  steering_angle_low_right->addPoint(steering_angle_low_right_point);
  steering_angle_mid_right->addPoint(steering_angle_mid_right_point);
  steering_angle_high_right->addPoint(steering_angle_high_right_point);

  steering_delta_angle_high_left->addPoint(steering_delta_angle_high_left_point);
  steering_delta_angle_mid_left->addPoint(steering_delta_angle_mid_left_point);
  steering_delta_angle_low_left->addPoint(steering_delta_angle_low_left_point);
  steering_delta_angle_center->addPoint(steering_delta_angle_center_point);
  steering_delta_angle_low_right->addPoint(steering_delta_angle_low_right_point);
  steering_delta_angle_mid_right->addPoint(steering_delta_angle_mid_right_point);
  steering_delta_angle_high_right->addPoint(steering_delta_angle_high_right_point);

  steering_angle_mid_left_cur->addPoint(steering_angle_mid_left_cur_point);
  steering_angle_low_left_cur->addPoint(steering_angle_low_left_cur_point);
  steering_angle_low_right_cur->addPoint(steering_angle_low_right_cur_point);
  steering_angle_mid_right_cur->addPoint(steering_angle_mid_right_cur_point);
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

  /////////

  Rule *r111 = new Rule();
  r111->addInput(offset, *left_offset_strong);
  r111->addInput(delta_offset, *left_delta_offset_strong);
  r111->addInput(curvature, *low_curvature);
  r111->addOutput(*steering_delta_angle_high_right);

  Rule *r112 = new Rule();
  r112->addInput(offset, *left_offset_strong);
  r112->addInput(delta_offset, *left_delta_offset_strong);
  r112->addInput(curvature, *mid_curvature);
  r112->addOutput(*steering_delta_angle_high_right);

  Rule *r113 = new Rule();
  r113->addInput(offset, *left_offset_strong);
  r113->addInput(delta_offset, *left_delta_offset_strong);
  r113->addInput(curvature, *high_curvature);
  r113->addOutput(*steering_delta_angle_high_right);

  /////

  Rule *r121 = new Rule();
  r121->addInput(offset, *left_offset_strong);
  r121->addInput(delta_offset, *left_delta_offset_low);
  r121->addInput(curvature, *low_curvature);
  r121->addOutput(*steering_delta_angle_high_right);

  Rule *r122 = new Rule();
  r122->addInput(offset, *left_offset_strong);
  r122->addInput(delta_offset, *left_delta_offset_low);
  r122->addInput(curvature, *mid_curvature);
  r122->addOutput(*steering_delta_angle_high_right);

  Rule *r123 = new Rule();
  r123->addInput(offset, *left_offset_strong);
  r123->addInput(delta_offset, *left_delta_offset_low);
  r123->addInput(curvature, *high_curvature);
  r123->addOutput(*steering_delta_angle_high_right);

  /////////////

  Rule *r131 = new Rule();
  r131->addInput(offset, *left_offset_strong);
  r131->addInput(delta_offset, *center_delta_offset);
  r131->addInput(curvature, *low_curvature);
  r131->addOutput(*steering_delta_angle_high_right);

  Rule *r132 = new Rule();
  r132->addInput(offset, *left_offset_strong);
  r132->addInput(delta_offset, *center_delta_offset);
  r132->addInput(curvature, *mid_curvature);
  r132->addOutput(*steering_delta_angle_high_right);

  Rule *r133 = new Rule();
  r133->addInput(offset, *left_offset_strong);
  r133->addInput(delta_offset, *center_delta_offset);
  r133->addInput(curvature, *high_curvature);
  r133->addOutput(*steering_delta_angle_high_right);

  /////////////

  Rule *r141 = new Rule();
  r141->addInput(offset, *left_offset_strong);
  r141->addInput(delta_offset, *right_delta_offset_low);
  r141->addInput(curvature, *low_curvature);
  r141->addOutput(*steering_delta_angle_mid_right);

  Rule *r142 = new Rule();
  r142->addInput(offset, *left_offset_strong);
  r142->addInput(delta_offset, *right_delta_offset_low);
  r142->addInput(curvature, *mid_curvature);
  r142->addOutput(*steering_delta_angle_mid_right);

  Rule *r143 = new Rule();
  r143->addInput(offset, *left_offset_strong);
  r143->addInput(delta_offset, *right_delta_offset_low);
  r143->addInput(curvature, *high_curvature);
  r143->addOutput(*steering_delta_angle_mid_right);

  /////////////


  Rule *r151 = new Rule();
  r151->addInput(offset, *left_offset_strong);
  r151->addInput(delta_offset, *right_delta_offset_strong);
  r151->addInput(curvature, *low_curvature);
  r151->addOutput(*steering_delta_angle_low_right);

  Rule *r152 = new Rule();
  r152->addInput(offset, *left_offset_strong);
  r152->addInput(delta_offset, *right_delta_offset_strong);
  r152->addInput(curvature, *mid_curvature);
  r152->addOutput(*steering_delta_angle_low_right);

  Rule *r153 = new Rule();
  r153->addInput(offset, *left_offset_strong);
  r153->addInput(delta_offset, *right_delta_offset_strong);
  r153->addInput(curvature, *high_curvature);
  r153->addOutput(*steering_delta_angle_low_right);

  /////////////

  Rule *r211 = new Rule();
  r211->addInput(offset, *left_offset_low);
  r211->addInput(delta_offset, *left_delta_offset_strong);
  r211->addInput(curvature, *low_curvature);
  r211->addOutput(*steering_delta_angle_high_right);

  Rule *r212 = new Rule();
  r212->addInput(offset, *left_offset_low);
  r212->addInput(delta_offset, *left_delta_offset_strong);
  r212->addInput(curvature, *mid_curvature);
  r212->addOutput(*steering_delta_angle_high_right);

  Rule *r213 = new Rule();
  r213->addInput(offset, *left_offset_low);
  r213->addInput(delta_offset, *left_delta_offset_strong);
  r213->addInput(curvature, *high_curvature);
  r213->addOutput(*steering_delta_angle_high_right);

  /////

  Rule *r221 = new Rule();
  r221->addInput(offset, *left_offset_low);
  r221->addInput(delta_offset, *left_delta_offset_low);
  r221->addInput(curvature, *low_curvature);
  r221->addOutput(*steering_delta_angle_mid_right);

  Rule *r222 = new Rule();
  r222->addInput(offset, *left_offset_low);
  r222->addInput(delta_offset, *left_delta_offset_low);
  r222->addInput(curvature, *mid_curvature);
  r222->addOutput(*steering_delta_angle_mid_right);

  Rule *r223 = new Rule();
  r223->addInput(offset, *left_offset_low);
  r223->addInput(delta_offset, *left_delta_offset_low);
  r223->addInput(curvature, *high_curvature);
  r223->addOutput(*steering_delta_angle_mid_right);

  /////////////

  Rule *r231 = new Rule();
  r231->addInput(offset, *left_offset_low);
  r231->addInput(delta_offset, *center_delta_offset);
  r231->addInput(curvature, *low_curvature);
  r231->addOutput(*steering_delta_angle_low_right);

  Rule *r232 = new Rule();
  r232->addInput(offset, *left_offset_low);
  r232->addInput(delta_offset, *center_delta_offset);
  r232->addInput(curvature, *mid_curvature);
  r232->addOutput(*steering_delta_angle_low_right);

  Rule *r233 = new Rule();
  r233->addInput(offset, *left_offset_low);
  r233->addInput(delta_offset, *center_delta_offset);
  r233->addInput(curvature, *high_curvature);
  r233->addOutput(*steering_delta_angle_low_right);

  /////////////

  Rule *r241 = new Rule();
  r241->addInput(offset, *left_offset_low);
  r241->addInput(delta_offset, *right_delta_offset_low);
  r241->addInput(curvature, *low_curvature);
  r241->addOutput(*steering_delta_angle_center);

  Rule *r242 = new Rule();
  r242->addInput(offset, *left_offset_low);
  r242->addInput(delta_offset, *right_delta_offset_low);
  r242->addInput(curvature, *mid_curvature);
  r242->addOutput(*steering_delta_angle_center);

  Rule *r243 = new Rule();
  r243->addInput(offset, *left_offset_low);
  r243->addInput(delta_offset, *right_delta_offset_low);
  r243->addInput(curvature, *high_curvature);
  r243->addOutput(*steering_delta_angle_center);

  /////////////


  Rule *r251 = new Rule();
  r251->addInput(offset, *left_offset_low);
  r251->addInput(delta_offset, *right_delta_offset_strong);
  r251->addInput(curvature, *low_curvature);
  r251->addOutput(*steering_delta_angle_center);

  Rule *r252 = new Rule();
  r252->addInput(offset, *left_offset_low);
  r252->addInput(delta_offset, *right_delta_offset_strong);
  r252->addInput(curvature, *mid_curvature);
  r252->addOutput(*steering_delta_angle_center);

  Rule *r253 = new Rule();
  r253->addInput(offset, *left_offset_low);
  r253->addInput(delta_offset, *right_delta_offset_strong);
  r253->addInput(curvature, *high_curvature);
  r253->addOutput(*steering_delta_angle_center);

 ////////////


  Rule *r311 = new Rule();
  r311->addInput(offset, *center_offset);
  r311->addInput(delta_offset, *left_delta_offset_strong);
  r311->addInput(curvature, *low_curvature);
  r311->addOutput(*steering_delta_angle_low_right);

  Rule *r312 = new Rule();
  r312->addInput(offset, *center_offset);
  r312->addInput(delta_offset, *left_delta_offset_strong);
  r312->addInput(curvature, *mid_curvature);
  r312->addOutput(*steering_delta_angle_low_right);

  Rule *r313 = new Rule();
  r313->addInput(offset, *center_offset);
  r313->addInput(delta_offset, *left_delta_offset_strong);
  r313->addInput(curvature, *high_curvature);
  r313->addOutput(*steering_delta_angle_low_right);

  /////

  Rule *r321 = new Rule();
  r321->addInput(offset, *center_offset);
  r321->addInput(delta_offset, *left_delta_offset_low);
  r321->addInput(curvature, *low_curvature);
  r321->addOutput(*steering_delta_angle_center);

  Rule *r322 = new Rule();
  r322->addInput(offset, *center_offset);
  r322->addInput(delta_offset, *left_delta_offset_low);
  r322->addInput(curvature, *mid_curvature);
  r322->addOutput(*steering_delta_angle_center);

  Rule *r323 = new Rule();
  r323->addInput(offset, *center_offset);
  r323->addInput(delta_offset, *left_delta_offset_low);
  r323->addInput(curvature, *high_curvature);
  r323->addOutput(*steering_delta_angle_center);

  /////////////

  Rule *r331 = new Rule();
  r331->addInput(offset, *center_offset);
  r331->addInput(delta_offset, *center_delta_offset);
  r331->addInput(curvature, *low_curvature);
  r331->addOutput(*steering_delta_angle_center);

  Rule *r332 = new Rule();
  r332->addInput(offset, *center_offset);
  r332->addInput(delta_offset, *center_delta_offset);
  r332->addInput(curvature, *mid_curvature);
  r332->addOutput(*steering_delta_angle_center);

  Rule *r333 = new Rule();
  r333->addInput(offset, *center_offset);
  r333->addInput(delta_offset, *center_delta_offset);
  r333->addInput(curvature, *high_curvature);
  r333->addOutput(*steering_delta_angle_center);

  /////////////

  Rule *r341 = new Rule();
  r341->addInput(offset, *center_offset);
  r341->addInput(delta_offset, *right_delta_offset_low);
  r341->addInput(curvature, *low_curvature);
  r341->addOutput(*steering_delta_angle_center);

  Rule *r342 = new Rule();
  r342->addInput(offset, *center_offset);
  r342->addInput(delta_offset, *right_delta_offset_low);
  r342->addInput(curvature, *mid_curvature);
  r342->addOutput(*steering_delta_angle_center);

  Rule *r343 = new Rule();
  r343->addInput(offset, *center_offset);
  r343->addInput(delta_offset, *right_delta_offset_low);
  r343->addInput(curvature, *high_curvature);
  r343->addOutput(*steering_delta_angle_center);

  /////////////


  Rule *r351 = new Rule();
  r351->addInput(offset, *center_offset);
  r351->addInput(delta_offset, *right_delta_offset_strong);
  r351->addInput(curvature, *low_curvature);
  r351->addOutput(*steering_delta_angle_low_left);

  Rule *r352 = new Rule();
  r352->addInput(offset, *center_offset);
  r352->addInput(delta_offset, *right_delta_offset_strong);
  r352->addInput(curvature, *mid_curvature);
  r352->addOutput(*steering_delta_angle_low_left);

  Rule *r353 = new Rule();
  r353->addInput(offset, *center_offset);
  r353->addInput(delta_offset, *right_delta_offset_strong);
  r353->addInput(curvature, *high_curvature);
  r353->addOutput(*steering_delta_angle_low_left);

  /////////////

  Rule *r411 = new Rule();
  r411->addInput(offset, *right_offset_low);
  r411->addInput(delta_offset, *left_delta_offset_strong);
  r411->addInput(curvature, *low_curvature);
  r411->addOutput(*steering_delta_angle_low_right);

  Rule *r412 = new Rule();
  r412->addInput(offset, *right_offset_low);
  r412->addInput(delta_offset, *left_delta_offset_strong);
  r412->addInput(curvature, *mid_curvature);
  r412->addOutput(*steering_delta_angle_low_right);

  Rule *r413 = new Rule();
  r413->addInput(offset, *right_offset_low);
  r413->addInput(delta_offset, *left_delta_offset_strong);
  r413->addInput(curvature, *high_curvature);
  r413->addOutput(*steering_delta_angle_low_right);

  /////

  Rule *r421 = new Rule();
  r421->addInput(offset, *right_offset_low);
  r421->addInput(delta_offset, *left_delta_offset_low);
  r421->addInput(curvature, *low_curvature);
  r421->addOutput(*steering_delta_angle_center);

  Rule *r422 = new Rule();
  r422->addInput(offset, *right_offset_low);
  r422->addInput(delta_offset, *left_delta_offset_low);
  r422->addInput(curvature, *mid_curvature);
  r422->addOutput(*steering_delta_angle_center);

  Rule *r423 = new Rule();
  r423->addInput(offset, *right_offset_low);
  r423->addInput(delta_offset, *left_delta_offset_low);
  r423->addInput(curvature, *high_curvature);
  r423->addOutput(*steering_delta_angle_center);

  /////////////

  Rule *r431 = new Rule();
  r431->addInput(offset, *right_offset_low);
  r431->addInput(delta_offset, *center_delta_offset);
  r431->addInput(curvature, *low_curvature);
  r431->addOutput(*steering_delta_angle_low_left);

  Rule *r432 = new Rule();
  r432->addInput(offset, *right_offset_low);
  r432->addInput(delta_offset, *center_delta_offset);
  r432->addInput(curvature, *mid_curvature);
  r432->addOutput(*steering_delta_angle_low_left);

  Rule *r433 = new Rule();
  r433->addInput(offset, *right_offset_low);
  r433->addInput(delta_offset, *center_delta_offset);
  r433->addInput(curvature, *high_curvature);
  r433->addOutput(*steering_delta_angle_low_left);

  /////////////

  Rule *r441 = new Rule();
  r441->addInput(offset, *right_offset_low);
  r441->addInput(delta_offset, *right_delta_offset_low);
  r441->addInput(curvature, *low_curvature);
  r441->addOutput(*steering_delta_angle_mid_left);

  Rule *r442 = new Rule();
  r442->addInput(offset, *right_offset_low);
  r442->addInput(delta_offset, *right_delta_offset_low);
  r442->addInput(curvature, *mid_curvature);
  r442->addOutput(*steering_delta_angle_mid_left);

  Rule *r443 = new Rule();
  r443->addInput(offset, *right_offset_low);
  r443->addInput(delta_offset, *right_delta_offset_low);
  r443->addInput(curvature, *high_curvature);
  r443->addOutput(*steering_delta_angle_mid_left);

  /////////////


  Rule *r451 = new Rule();
  r451->addInput(offset, *right_offset_low);
  r451->addInput(delta_offset, *right_delta_offset_strong);
  r451->addInput(curvature, *low_curvature);
  r451->addOutput(*steering_delta_angle_high_left);

  Rule *r452 = new Rule();
  r452->addInput(offset, *right_offset_low);
  r452->addInput(delta_offset, *right_delta_offset_strong);
  r452->addInput(curvature, *mid_curvature);
  r452->addOutput(*steering_delta_angle_high_left);

  Rule *r453 = new Rule();
  r453->addInput(offset, *right_offset_low);
  r453->addInput(delta_offset, *right_delta_offset_strong);
  r453->addInput(curvature, *high_curvature);
  r453->addOutput(*steering_delta_angle_high_left);

  /////////////

  Rule *r511 = new Rule();
  r511->addInput(offset, *right_offset_strong);
  r511->addInput(delta_offset, *left_delta_offset_strong);
  r511->addInput(curvature, *low_curvature);
  r511->addOutput(*steering_delta_angle_low_left);

  Rule *r512 = new Rule();
  r512->addInput(offset, *right_offset_strong);
  r512->addInput(delta_offset, *left_delta_offset_strong);
  r512->addInput(curvature, *mid_curvature);
  r512->addOutput(*steering_delta_angle_low_left);

  Rule *r513 = new Rule();
  r513->addInput(offset, *right_offset_strong);
  r513->addInput(delta_offset, *left_delta_offset_strong);
  r513->addInput(curvature, *high_curvature);
  r513->addOutput(*steering_delta_angle_low_left);

  /////

  Rule *r521 = new Rule();
  r521->addInput(offset, *right_offset_strong);
  r521->addInput(delta_offset, *left_delta_offset_low);
  r521->addInput(curvature, *low_curvature);
  r521->addOutput(*steering_delta_angle_mid_left);

  Rule *r522 = new Rule();
  r522->addInput(offset, *right_offset_strong);
  r522->addInput(delta_offset, *left_delta_offset_low);
  r522->addInput(curvature, *mid_curvature);
  r522->addOutput(*steering_delta_angle_mid_left);

  Rule *r523 = new Rule();
  r523->addInput(offset, *right_offset_strong);
  r523->addInput(delta_offset, *left_delta_offset_low);
  r523->addInput(curvature, *high_curvature);
  r523->addOutput(*steering_delta_angle_mid_left);

  /////////////

  Rule *r531 = new Rule();
  r531->addInput(offset, *right_offset_strong);
  r531->addInput(delta_offset, *center_delta_offset);
  r531->addInput(curvature, *low_curvature);
  r531->addOutput(*steering_delta_angle_high_left);

  Rule *r532 = new Rule();
  r532->addInput(offset, *right_offset_strong);
  r532->addInput(delta_offset, *center_delta_offset);
  r532->addInput(curvature, *mid_curvature);
  r532->addOutput(*steering_delta_angle_high_left);

  Rule *r533 = new Rule();
  r533->addInput(offset, *right_offset_strong);
  r533->addInput(delta_offset, *center_delta_offset);
  r533->addInput(curvature, *high_curvature);
  r533->addOutput(*steering_delta_angle_high_left);

  /////////////

  Rule *r541 = new Rule();
  r541->addInput(offset, *right_offset_strong);
  r541->addInput(delta_offset, *right_delta_offset_low);
  r541->addInput(curvature, *low_curvature);
  r541->addOutput(*steering_delta_angle_high_left);

  Rule *r542 = new Rule();
  r542->addInput(offset, *right_offset_strong);
  r542->addInput(delta_offset, *right_delta_offset_low);
  r542->addInput(curvature, *mid_curvature);
  r542->addOutput(*steering_delta_angle_high_left);

  Rule *r543 = new Rule();
  r543->addInput(offset, *right_offset_strong);
  r543->addInput(delta_offset, *right_delta_offset_low);
  r543->addInput(curvature, *high_curvature);
  r543->addOutput(*steering_delta_angle_high_left);

  /////////////


  Rule *r551 = new Rule();
  r551->addInput(offset, *right_offset_strong);
  r551->addInput(delta_offset, *right_delta_offset_strong);
  r551->addInput(curvature, *low_curvature);
  r551->addOutput(*steering_delta_angle_high_left);

  Rule *r552 = new Rule();
  r552->addInput(offset, *right_offset_strong);
  r552->addInput(delta_offset, *right_delta_offset_strong);
  r552->addInput(curvature, *mid_curvature);
  r552->addOutput(*steering_delta_angle_high_left);

  Rule *r553 = new Rule();
  r553->addInput(offset, *right_offset_strong);
  r553->addInput(delta_offset, *right_delta_offset_strong);
  r553->addInput(curvature, *high_curvature);
  r553->addOutput(*steering_delta_angle_high_left);

  /////////////


  fc_steering_angle->addRule(*r111);
  fc_steering_angle->addRule(*r112);
  fc_steering_angle->addRule(*r113);

  fc_steering_angle->addRule(*r121);
  fc_steering_angle->addRule(*r122);
  fc_steering_angle->addRule(*r123);

  fc_steering_angle->addRule(*r131);
  fc_steering_angle->addRule(*r132);
  fc_steering_angle->addRule(*r133);

  fc_steering_angle->addRule(*r141);
  fc_steering_angle->addRule(*r142);
  fc_steering_angle->addRule(*r143);

  fc_steering_angle->addRule(*r151);
  fc_steering_angle->addRule(*r152);
  fc_steering_angle->addRule(*r153);



  fc_steering_angle->addRule(*r211);
  fc_steering_angle->addRule(*r212);
  fc_steering_angle->addRule(*r213);

  fc_steering_angle->addRule(*r221);
  fc_steering_angle->addRule(*r222);
  fc_steering_angle->addRule(*r223);

  fc_steering_angle->addRule(*r231);
  fc_steering_angle->addRule(*r232);
  fc_steering_angle->addRule(*r233);

  fc_steering_angle->addRule(*r241);
  fc_steering_angle->addRule(*r242);
  fc_steering_angle->addRule(*r243);

  fc_steering_angle->addRule(*r251);
  fc_steering_angle->addRule(*r252);
  fc_steering_angle->addRule(*r253);



  fc_steering_angle->addRule(*r311);
  fc_steering_angle->addRule(*r312);
  fc_steering_angle->addRule(*r313);

  fc_steering_angle->addRule(*r321);
  fc_steering_angle->addRule(*r322);
  fc_steering_angle->addRule(*r323);

  fc_steering_angle->addRule(*r331);
  fc_steering_angle->addRule(*r332);
  fc_steering_angle->addRule(*r333);

  fc_steering_angle->addRule(*r341);
  fc_steering_angle->addRule(*r342);
  fc_steering_angle->addRule(*r343);

  fc_steering_angle->addRule(*r351);
  fc_steering_angle->addRule(*r352);
  fc_steering_angle->addRule(*r353);



  fc_steering_angle->addRule(*r411);
  fc_steering_angle->addRule(*r412);
  fc_steering_angle->addRule(*r413);

  fc_steering_angle->addRule(*r421);
  fc_steering_angle->addRule(*r422);
  fc_steering_angle->addRule(*r423);

  fc_steering_angle->addRule(*r431);
  fc_steering_angle->addRule(*r432);
  fc_steering_angle->addRule(*r433);

  fc_steering_angle->addRule(*r441);
  fc_steering_angle->addRule(*r442);
  fc_steering_angle->addRule(*r443);

  fc_steering_angle->addRule(*r451);
  fc_steering_angle->addRule(*r452);
  fc_steering_angle->addRule(*r453);


  fc_steering_angle->addRule(*r511);
  fc_steering_angle->addRule(*r512);
  fc_steering_angle->addRule(*r513);

  fc_steering_angle->addRule(*r521);
  fc_steering_angle->addRule(*r522);
  fc_steering_angle->addRule(*r523);

  fc_steering_angle->addRule(*r531);
  fc_steering_angle->addRule(*r532);
  fc_steering_angle->addRule(*r533);

  fc_steering_angle->addRule(*r541);
  fc_steering_angle->addRule(*r542);
  fc_steering_angle->addRule(*r543);

  fc_steering_angle->addRule(*r551);
  fc_steering_angle->addRule(*r552);
  fc_steering_angle->addRule(*r553);
}