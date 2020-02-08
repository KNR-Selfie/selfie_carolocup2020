#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <selfie_control/PIDTunerConfig.h>
#include <dynamic_reconfigure/server.h>

#include <pid/PidConfig.h>
#include <dynamic_reconfigure/Config.h>

float act_speed = 1.0;
bool running = true;
bool change_switch = false;

float kp_base = 1.0;
float speed_base = 1.0;
float coeff = 0.5;
float deadzone = 0.1;
float kp_base_scale = 1;


float L_speed_threshold;
float M_speed_threshold;
float H_speed_threshold;

float L_Kp;
float L_Ki;
float L_Kd;

float M_Kp;
float M_Ki;
float M_Kd;

float H_Kp;
float H_Ki;
float H_Kd;

// variables used for changing settings of PID
dynamic_reconfigure::ReconfigureRequest srv_req_;
dynamic_reconfigure::ReconfigureResponse srv_resp_;
dynamic_reconfigure::DoubleParameter double_param_;
dynamic_reconfigure::Config conf_;

void setKd(float Kd, ros::NodeHandle &pnh)
{
  float scale = 1.0;
  while (Kd > 1 || Kd <= 0.1)
  {
    if (Kd > 1)
    {
      Kd = Kd / 10;
      scale = scale * 10;
    }
    else if (Kd <= 0.1)
    {
      Kd = Kd * 10;
      scale = scale / 10;
    }
  }

  conf_.doubles.clear();
  double_param_.name = "Kd";
  double_param_.value = Kd;
  conf_.doubles.push_back(double_param_);

  double_param_.name = "Kd_scale";
  double_param_.value = scale;
  conf_.doubles.push_back(double_param_);

  srv_req_.config = conf_;

  ros::service::call("/pid_controller/set_parameters", srv_req_, srv_resp_);

  pnh.setParam("/pid_controller/Kd",Kd);
  pnh.setParam("/pid_controller/Kd_scale",scale);
}

void setKp(float Kp, ros::NodeHandle &pnh)
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

  conf_.doubles.clear();
  double_param_.name = "Kp";
  double_param_.value = Kp;
  conf_.doubles.push_back(double_param_);

  double_param_.name = "Kp_scale";
  double_param_.value = scale;
  conf_.doubles.push_back(double_param_);

  srv_req_.config = conf_;

  ros::service::call("/pid_controller/set_parameters", srv_req_, srv_resp_);

  pnh.setParam("/pid_controller/Kp",Kp);
  pnh.setParam("/pid_controller/Kp_scale",scale);
}

void setKi(float Ki, ros::NodeHandle &pnh)
{
  float scale = 1.0;
  while (Ki > 1 || Ki <= 0.1)
  {
    if (Ki > 1)
    {
      Ki = Ki / 10;
      scale = scale * 10;
    }
    else if (Ki <= 0.1)
    {
      Ki = Ki * 10;
      scale = scale / 10;
    }
  }

  conf_.doubles.clear();
  double_param_.name = "Ki";
  double_param_.value = Ki;
  conf_.doubles.push_back(double_param_);

  double_param_.name = "Ki_scale";
  double_param_.value = scale;
  conf_.doubles.push_back(double_param_);

  srv_req_.config = conf_;

  ros::service::call("/pid_controller/set_parameters", srv_req_, srv_resp_);

  pnh.setParam("/pid_controller/Ki",Ki);
  pnh.setParam("/pid_controller/Ki_scale",scale);
}
void speedCallback(const std_msgs::Float32 &msg)
{
  if (std::abs(act_speed - msg.data) > 0.3)
  {
    act_speed = msg.data;
    change_switch = true;
  }
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

  pnh.getParam("L_speed_threshold", L_speed_threshold);
  pnh.getParam("M_speed_threshold", M_speed_threshold);
  pnh.getParam("H_speed_threshold", H_speed_threshold);

  pnh.getParam("L_Kp", L_Kp);
  pnh.getParam("L_Ki", L_Ki);
  pnh.getParam("L_Kd", L_Kd);

  pnh.getParam("M_Kp", M_Kp);
  pnh.getParam("M_Ki", M_Ki);
  pnh.getParam("M_Kd", M_Kd);

  pnh.getParam("H_Kp", H_Kp);
  pnh.getParam("H_Ki", H_Ki);
  pnh.getParam("H_Kd", H_Kd);

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
    if(running && change_switch)
    {
      change_switch = false;

      float new_kp = -1.0;
      float new_ki = -1.0;
      float new_kd = -1.0;

      if(act_speed > H_speed_threshold)
      { 
        new_kp = H_Kp;
        new_ki = H_Ki;
        new_kd = H_Kd;
        ROS_INFO("set H speed PID values");

      }
      else if(act_speed > M_speed_threshold)
      {
        new_kp = M_Kp;
        new_ki = M_Ki;
        new_kd = M_Kd;
        ROS_INFO("set M speed PID values");
      }
      else if(act_speed > L_speed_threshold)
      {
        new_kp = L_Kp;
        new_ki = L_Ki;
        new_kd = L_Kd;
        ROS_INFO("set L speed PID values");
      }
      if(new_kp != -1.0 && new_ki != -1.0 && new_kd != -1.0)
      {
        setKp(new_kp, pnh);
        setKd(new_kd, pnh);
        setKi(new_ki, pnh);
      }
      
    }
    loop_rate.sleep();
  }
}

void reconfigureCB(selfie_control::PIDTunerConfig& config, uint32_t level)
{ 
  
  if(H_Kp != (float)config.H_Kp)
  {
    H_Kp = config.H_Kp;
    ROS_INFO("H_Kp new value %f",H_Kp);
  }
  if(H_Ki != (float)config.H_Ki)
  {
    H_Ki = config.H_Ki;
    ROS_INFO("H_Ki new value %f",H_Ki);
  }
  if(H_Kd != (float)config.H_Kd)
  {
    H_Kd = config.H_Kd;
    ROS_INFO("H_Kd new value %f", H_Kd);
  }

  if(M_Kp != (float)config.M_Kp)
  {
    M_Kp = config.M_Kp;
    ROS_INFO("M_Kp new value %f",M_Kp);
  }
  if(M_Ki != (float)config.M_Ki)
  {
    M_Ki = config.M_Ki;
    ROS_INFO("M_Ki new value %f",M_Ki);
  }
  if(M_Kd != (float)config.M_Kd)
  {
    M_Kd = config.M_Kd;
    ROS_INFO("M_Kd new value %f", M_Kd);
  }


  if(L_Kp != (float)config.L_Kp)
  {
    L_Kp = config.L_Kp;
    ROS_INFO("L_Kp new value %f",L_Kp);
  }
  if(L_Ki != (float)config.L_Ki)
  {
    L_Ki = config.L_Ki;
    ROS_INFO("L_Ki new value %f",L_Ki);
  }
  if(L_Kd != (float)config.L_Kd)
  {
    L_Kd = config.L_Kd;
    ROS_INFO("L_Kd new value %f", L_Kd);
  }
  
  if(H_speed_threshold != (float)config.H_speed_threshold)
  {
    H_speed_threshold = config.H_speed_threshold;
    ROS_INFO("H_speed_threshold new value %f", H_speed_threshold);
  }
  if(M_speed_threshold != (float)config.M_speed_threshold)
  {
    M_speed_threshold = config.M_speed_threshold;
    ROS_INFO("M_speed_threshold new value %f", M_speed_threshold);
  }
  if(L_speed_threshold != (float)config.L_speed_threshold)
  {
    L_speed_threshold = config.L_speed_threshold;
    ROS_INFO("L_speed_threshold new value %f", L_speed_threshold);
  }
  change_switch = true;
}