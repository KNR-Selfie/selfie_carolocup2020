#ifndef SELFIE_PERCEPTION_ROAD_LINE_H
#define SELFIE_PERCEPTION_ROAD_LINE_H

#include <opencv2/imgproc/imgproc.hpp>

#include <selfie_perception/definitions.h>
#include <selfie_perception/polynomials.h>
#include <selfie_perception/particle_filter.h>

class RoadLine
{
public:
  RoadLine();

  void aprox();
  int pointsSize();
  void calcParams();
  void addBottomPoint();

  void pfSetup(int num_particles, int num_control_points, float std);
  void pfInit();
  bool pfExecute();


  // getters
  bool isExist()                        { return exist_; }
  int getDegree()                       { return degree_; }
  float getLength()                     { return length_; }
  bool isShort()                        { return is_short_; }
  std::vector<float> getCoeff()         { return coeff_; }
  std::vector<cv::Point2f> getPoints()  { return points_; }

  // setters
  void setShortParam(float param)
  {
    length_not_short_ = param;
  }

  void setDegree(int degree)
  {
    degree_ = degree;
  }

  void setPoints(std::vector<cv::Point2f> points)
  {
    points_ = points;
  }

  void setCoeff(std::vector<float> coeff)
  {
    coeff_ = coeff;
  }

  void setExist(bool exist)
  {
    if (!exist)
    {
      points_.clear();
      is_short_ = true;
      length_ = 0;
    }
    exist_ = exist;
  }

private:
  bool exist_    {false};
  int degree_    {2};
  float length_  {0};
  bool is_short_ {false};
  std::vector<cv::Point2f> points_;
  std::vector<float> coeff_;

  ParticleFilter pf_;
  float pf_std_;
  int pf_num_particles_;
  int pf_num_points_;

  float length_not_short_ {0.5};
};

#endif  //  SELFIE_PERCEPTION_ROAD_LINE_H