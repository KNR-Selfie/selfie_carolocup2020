#ifndef QR_DECODER_H 
#define QR_DECODER_H

#include <ros/ros.h>
#include <zbar.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/algorithm/cxx11/any_of.hpp>

class QrDecoder
{
    ros::NodeHandle nh_,pnh_;
    ros::Subscriber imageSub_; 

    zbar::ImageScanner zbarScanner_;
    zbar::Image zbarImage_;


    float qrInvisibleTimeThresh_;

    ros::Timer timer_;
    cv_bridge::CvImagePtr cv_ptr;

    void startSearching();
    void imageRectCallback(const sensor_msgs::Image::ConstPtr msg);
    void decodeImage(const cv_bridge::CvImagePtr img);
    void timerCallback(const ros::TimerEvent &e);
    void resetTimer();
public:
    QrDecoder(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

};
#endif