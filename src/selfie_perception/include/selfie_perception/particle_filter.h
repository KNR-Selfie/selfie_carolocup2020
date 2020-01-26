/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef SELFIE_PARTICLE_FILTER_H
#define SELFIE_PARTICLE_FILTER_H
#pragma once

#include <iostream>
#include <vector>
#include <random>
//#include <algorithm>
//#include <numeric>
#include <selfie_perception/polynomials.h>
#include <selfie_perception/definitions.h>

struct Particle
{
  std::vector<cv::Point2f> points;
  std::vector<float> coeff;
  float weight;
  int poly_degree;
};

class ParticleFilter
{
public:
  ParticleFilter(int num_particles, int num_control_points) : num_particles_(num_particles),
                                                              num_control_points_(num_control_points_) { };
  ParticleFilter() { };
  ~ParticleFilter() { };

  void init(std::vector<cv::Point2f> points);

  void prediction(float std_min, float std_max);

  void calculateBest();

  void updateWeights(std::vector<cv::Point2f> &p_obs);

  void resample();

  std::vector<float> getCoeff(int particle_id);

  std::vector<float> getBestCoeff();

  int getBestDegree();

  std::vector<cv::Point2f> getControlPoints(int particle_id);

  void reset();

  const bool initialized() const
  {
    return is_initialized_;
  }

  void setNumParticles(int num)
  {
    num_particles_ = num;
  }

  void setNumPoints(int num)
  {
    num_control_points_ = num;
  }

  void setPolyDegree_(int degree)
  {
    poly_degree_ = degree;
  }

private:
  int poly_degree_{2};
  int num_particles_{0};
  int num_control_points_{0};
  bool is_initialized_{false};
  std::vector<float> weights_;
  std::vector<Particle> particles_;
  Particle best_particle_;

  float findMinPointToParabola(cv::Point2f p, std::vector<float> coeff);
  float getDistance(cv::Point2f p1, cv::Point2f p2);
};

#endif  //  SELFIE_PARTICLE_FILTER_H