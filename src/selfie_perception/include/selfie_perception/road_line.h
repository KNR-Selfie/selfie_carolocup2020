#ifndef SELFIE_PERCEPTION_ROAD_LINE_H
#define SELFIE_PERCEPTION_ROAD_LINE_H

#include <selfie_perception/definitions.h>
#include <selfie_perception/polynomials.h>
#include <selfie_perception/particle_filter.h>

class RoadLine
{
public:
  RoadLine()

  pfSetup(int num_particles, int num_control_points, float std);
  pfInit();
  aprox();


  // getters and setters
  //int getIndex() { return index_; }
  //void setIndex(int index) {index_ = index; }

  bool isExist() { return exist_; }
  void setExist(bool exist) { exist_ = exist; }

  int getDegree() { return degree_; }
  void setDegree(int degree) {degree_ = degree; }

  float getLength() { return length_; }

  bool isShort() { return is_short_; }

  std::vector<cv::Point2f> getPoints() { return points_; }
  void setPoints(std::vector<cv::Point2f> points) { points_ = points; }

  std::vector<float> getCoeff() { return coeff_; }
  void getCoeff(std::vector<float> coeff) { coeff_ = coeff; }

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
};

#endif  //  SELFIE_PERCEPTION_ROAD_LINE_H