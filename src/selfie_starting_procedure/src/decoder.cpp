/***Copyright ( c) 2019, KNR Selfie*
 * This code is licensed under BSD license (see LICENSE for details)
 **/
#include "selfie_starting_procedure/qr_decoder.h"

QrDecoder::QrDecoder(const ros::NodeHandle &nh, const ros::NodeHandle &pnh): nh_(nh), pnh_(pnh)
{
  zbar_scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
  gate_open_pub_ = nh_.advertise<std_msgs::Empty>("qr_gate_open", 1);
  pnh_.param<float>("qr_invisible_time_thresh", qr_invisible_time_thresh_, 1.f);
  pnh_.param<int>("detect_samples_length",detect_samples_length_,20);
  pnh_.param<float>("min_detect_rate",min_detect_rate_,0.6);
  pnh_.param("visualize",visualize_,true);
  // oneshot on, autostart off
  timer_ = nh_.createTimer(ros::Duration(qr_invisible_time_thresh_), &QrDecoder::timerCallback, this, true, false);
  start_serv_ = nh_.advertiseService("startQrSearch", &QrDecoder::startSearching, this);
  timer_running_ = false;


}
bool QrDecoder::startSearching(std_srvs::Empty::Request &rq, std_srvs::Empty::Response &rp)
{
  image_sub_ = nh_.subscribe("image_rect", 1, &QrDecoder::imageRectCallback, this);
  std::cout<<"start searching"<<std::endl;
  detect_list_ = std::list<float>(detect_samples_length_);
  detect_rate_ = 0.f;
  return true;
}

void QrDecoder::timerCallback(const ros::TimerEvent &e)
{
  gate_open_pub_.publish(std_msgs::Empty());
  if(visualize_) cv::destroyAllWindows();
  image_sub_.shutdown();
  timer_running_ = false;
}

void QrDecoder::imageRectCallback(const sensor_msgs::Image::ConstPtr img)
{
  this->decodeImage(cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8));
}

void QrDecoder::decodeImage(const cv_bridge::CvImagePtr raw_img)
{
  cv_ptr = raw_img;
  if(visualize_)
  {
    cv::namedWindow("not equalized",cv::WINDOW_NORMAL);
    cv::imshow("not equalized",cv_ptr->image);
  }
  cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2GRAY);
  cv::equalizeHist(cv_ptr->image,cv_ptr->image);
  if(visualize_)
  {
    cv::namedWindow("equalized",cv::WINDOW_NORMAL);
    cv::imshow("equalized",cv_ptr->image);
    cv::waitKey(1);
  }

  int width = cv_ptr->image.cols;
  int height = cv_ptr->image.rows;

  zbar::Image img(width, height, "Y800",cv_ptr->image.data, width * height);

  zbar_scanner_.scan(img);
  if (boost::algorithm::any_of(img.symbol_begin(), img.symbol_end(),
   [](zbar::Symbol sym){return sym.get_data() == "STOP";}))
  {
    detect_list_.push_back(1.0f);
    if(detect_rate_ > min_detect_rate_ && !timer_running_)
    {
      timer_.start();
      timer_running_ = true;
    }
    if(timer_running_)
    {
      resetTimer();
    }
  }
  else
  {
    detect_list_.push_back(0.0f);
  }
  detect_rate_ += (detect_list_.back() - detect_list_.front()) / static_cast<float>(detect_samples_length_);
  detect_list_.pop_front();
  std::cout<<"detect rate "<<detect_rate_<<std::endl;
  
}

void QrDecoder::resetTimer()
{
  timer_.setPeriod(ros::Duration(qr_invisible_time_thresh_), true);  // reset
  ROS_INFO("timer reset");
}
