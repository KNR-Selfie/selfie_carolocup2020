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
  void addBottomPoint(bool force = false);
  void generateForDensity();
  void reduceTopPoints(float ratio);
  cv::Point2f getPointNextToBottom(float min_dist_to_bottom);
  void reducePointsToStraight(int check_to_index);
  float getMaxDiffonX();
  int getIndexOnMerge();
  void reset();

  void pfSetup(int num_particles, int num_control_points, float std_min, float std_max);
  void pfInit();
  bool pfExecute();
  void pfReset();


  // getters
  bool isExist()                        { return exist_; }
  int getDegree()                       { return degree_; }
  float getLength()                     { return length_; }
  bool isShort()                        { return is_short_; }
  bool isPFInitialized()                { return pf_.initialized(); }
  std::vector<float> getCoeff()         { return coeff_; }
  std::vector<cv::Point2f> getPoints()  { return points_; }
  std::vector<float> getParticleCoeff(int particle_id)
  {
    return pf_.getCoeff(particle_id);
  }

  std::vector<cv::Point2f> getParticleControlPoints(int particle_id)
  {
    return pf_.getControlPoints(particle_id);
  }

  // setters
  void setShortParam(float param)
  {
    length_not_short_ = param;
  }

  void setPointsDensity(float param)
  {
    points_density_ = param;
  }

  void setDegree(int degree)
  {
    degree_ = degree;
    pf_.setPolyDegree_(degree);
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
      pf_.reset();
      is_short_ = true;
      length_ = 0;
    }
    exist_ = exist;
  }

  void addPoints(std::vector<cv::Point2f> points)
  {
    points_.insert(points_.end(), std::begin(points), std::end(points));
    exist_ = true;
  }

  void clearPoints()
  {
    points_.clear();
    exist_ = false;
  }

private:
  bool exist_    {false};
  int degree_    {2};
  float length_  {0};
  bool is_short_ {false};
  std::vector<cv::Point2f> points_;
  std::vector<float> coeff_;

  ParticleFilter pf_;
  float pf_std_min_;
  float pf_std_max_;
  int pf_num_particles_;
  int pf_num_points_;

  float length_not_short_ {0.5};
  float points_density_   {15};

  float getDistance(cv::Point2f p1, cv::Point2f p2);
  float getA(cv::Point2f p1, cv::Point2f p2);
};

#endif  //  SELFIE_PERCEPTION_ROAD_LINE_H