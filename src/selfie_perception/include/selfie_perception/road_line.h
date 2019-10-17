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

  void pfSetup(int num_particles, int num_control_points, float std);
  void pfInit();
  void aprox();
  int pointsSize();
  void calcParams();
  void addBottomPoint();


  // getters and setters
  void setShortParam(float param) { length_not_short_ = param; };

  bool isExist() { return exist_; }
  void setExist(bool exist) { exist_ = exist; }

  int getDegree() { return degree_; }
  void setDegree(int degree) {degree_ = degree; }

  float getLength() { return length_; }

  bool isShort() { return is_short_; }

  std::vector<cv::Point2f> getPoints() { return points_; }
  void setPoints(std::vector<cv::Point2f> points)
  {
    points_.clear();
    points_ = points;
  }

  std::vector<float> getCoeff() { return coeff_; }
  void setCoeff(std::vector<float> coeff)
  {
    coeff_.clear();
    coeff_ = coeff;
  }

private:
  //int index_     {-1};
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