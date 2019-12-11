#include "ros/ros.h"
#include "rectify.h"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_proc_fisheye::RectifyNodelet, nodelet::Nodelet)


namespace image_proc_fisheye {
  void RectifyNodelet::onInit(){
    ros::NodeHandle &nh = getNodeHandle();
    camera_set_=false;
    getPrivateNodeHandle().getParam("mapx_file", mapx_file_);
    getPrivateNodeHandle().getParam("mapy_file", mapy_file_);

    cv::FileStorage fs1;
    fs1.open(mapx_file_, cv::FileStorage::READ);
    fs1["mapx"] >> mapx_;
    fs1.release();
    ROS_INFO("mapx loaded");

    cv::FileStorage fs2;
    fs2.open(mapy_file_, cv::FileStorage::READ);
    fs2["mapy"] >> mapy_;
    fs2.release();
    ROS_INFO("mapy loaded");

    camera_set_ = true;

    sub_ = nh.subscribe("image_raw", 5, &RectifyNodelet::process_image, this);
    pub_ = nh.advertise<sensor_msgs::Image>("image_rect", 10);
  };

  void RectifyNodelet::process_image(const sensor_msgs::ImageConstPtr& frame){
    if(!camera_set_){
      return;
    }
    cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(frame);
    cv::Mat image_gpu(image->image);
    cv::Mat image_gpu_rect(cv::Size(image->image.rows, image->image.cols), image->image.type());
    cv::remap(image_gpu, image_gpu_rect, mapx_, mapy_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::Mat image_rect = cv::Mat(image_gpu_rect);
	
    cv_bridge::CvImage out_msg;
    out_msg.header   = frame->header;
    out_msg.encoding = frame->encoding;
    out_msg.image  = image_rect;
    pub_.publish(out_msg.toImageMsg());
  }

  void RectifyNodelet::camera_info(const sensor_msgs::CameraInfoConstPtr& info_msg){
    sub_info_.shutdown();
    camera_set_ = true;
  }
} // namespace

