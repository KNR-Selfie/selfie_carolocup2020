#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <selfie_control/PIDTunerConfig.h>
#include <dynamic_reconfigure/server.h>

#include <pid/PidConfig.h>
#include <dynamic_reconfigure/Config.h>

float act_speed = 1.0;
bool running = true;

float kp_base = 1.0;
float speed_base = 1.0;
float coeff = 0.5;
float deadzone = 0.1;
float kp_base_scale = 1;

// variables used for changing settings of PID
dynamic_reconfigure::ReconfigureRequest srv_req_;
dynamic_reconfigure::ReconfigureResponse srv_resp_;
dynamic_reconfigure::DoubleParameter double_param_;
dynamic_reconfigure::Config conf_;

void setKp(float Kp)
{
  float scale = 1.0;
  while (Kp > 1 || Kp <= 0.1)
  {
    if (Kp > 1)
    {
      Kp = Kp / 10;
      scale = scale * 10;
    }
    else if (Kp <= 0.1)
    {
      Kp = Kp * 10;
      scale = scale / 10;
    }
  }

  conf_.clear();
  double_param_.name = "Kp";
  double_param_.value = Kp;
  conf_.doubles.push_back(double_param_);

  double_param_.name = "Kp_scale";
  double_param_.value = scale;
  conf_.doubles.push_back(double_param_);

  srv_req_.config = conf_;

  ros::service::call("/pid_controller/set_parameters", srv_req_, srv_resp_);
}

void speedCallback(const std_msgs::Float32 &msg)
{
  act_speed = msg.data;
}

bool startRunningCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  running = true;
  ROS_INFO("PID_tuner started");
  return true;
}

bool stopRunningCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  running = false;
  ROS_INFO("PID_tuner stopped");
  return true;
}

void reconfigureCB(selfie_control::PIDTunerConfig& config, uint32_t level);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pid_tuner");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("kp_base", kp_base);
  pnh.getParam("speed_base", speed_base);
  pnh.getParam("coeff", coeff);
  pnh.getParam("deadzone", deadzone);

  ROS_INFO("kp_base: %.3f", kp_base);
  ROS_INFO("speed_base: %.3f", speed_base);
  ROS_INFO("coeff: %.3f", coeff);
  ROS_INFO("deadzone: %.3f\n", deadzone);
  ROS_INFO("pid_tuner initialized");

  ros::Subscriber sub_speed = nh.subscribe("stm32/speed", 50, speedCallback);
  ros::ServiceServer run_service = nh.advertiseService("/PID_tuner_start", startRunningCallback);
  ros::ServiceServer not_run_service = nh.advertiseService("/PID_tuner_stop", stopRunningCallback);

  dynamic_reconfigure::Server<selfie_control::PIDTunerConfig> dr_server_;
  dynamic_reconfigure::Server<selfie_control::PIDTunerConfig>::CallbackType dr_server_CB_;
  dr_server_CB_ = boost::bind(&reconfigureCB, _1, _2);
  dr_server_.setCallback(dr_server_CB_);



  ros::Rate loop_rate(10);

  while (nh.ok())
  {
    // check for incoming messages
    ros::spinOnce();

    if(running)
    {
      float speed_diff = act_speed - speed_base;
      float kp_diff = speed_diff * coeff;
      if (std::abs(kp_diff) > deadzone)
      {
        float new_kp = kp_base + kp_diff;
        setKp(new_kp);
      }
    }

    loop_rate.sleep();
  }
}

void reconfigureCB(selfie_control::PIDTunerConfig& config, uint32_t level)
{
  if(kp_base != config.kp_base)
  {
    kp_base = config.kp_base;
    ROS_INFO("kp_base to end new value: %f", kp_base);
  }
  if(speed_base != config.speed_base)
  {
    speed_base = config.speed_base;
    ROS_INFO("speed_base new value: %f", max_speed_);
  }
  if(coeff != config.coeff)
  {
    coeff = config.coeff;
    ROS_INFO("coeff to end new value %f", coeff);
  }
  if(deadzone != config.deadzone)
  {
    deadzone = config.deadzone;
    ROS_INFO("deadzone new value %f", deadzone);
  }
}