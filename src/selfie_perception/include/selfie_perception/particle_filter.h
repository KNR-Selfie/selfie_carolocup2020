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
#include <algorithm>
#include <numeric>

#include <iostream>
#include <string>
#include <math.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

template<class T, size_t N>
constexpr size_t size(T (&)[N]) { return N; }

struct Particle
{
  int id;
  std::vector<cv::Point2f> points;
  std::vector<float> coeff;
  float weight;
};

class ParticleFilter
{
public:
  ParticleFilter(int num_particles, int num_control_points) : num_particles_(num_particles),
                                                              num_control_points_(num_control_points_) { };
  ParticleFilter() { };
  ~ParticleFilter() { };

  void init(std::vector<cv::Point2f> points, double std);

  void prediction(double std);

  void updateWeights(std::vector<cv::Point2f> &p_obs);

  void resample();

  std::vector<float> getCoeff(int particle_id);

  std::vector<float> getBestCoeff();

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

  bool polyfitt(std::vector<cv::Point2f> line, int degree, std::vector<float> &vec)
{
  int nDegree = degree;
  if (line.size() < nDegree)
    return false;

  // more intuative this way
  ++nDegree;

  size_t nCount = line.size();
  boost::numeric::ublas::matrix<float> oXMatrix(nCount, nDegree);
  boost::numeric::ublas::matrix<float> oYMatrix(nCount, 1);

  // copy y matrix
  for (size_t i = 0; i < nCount; ++i)
  {
    oYMatrix(i, 0) = line[i].y;
  }

  // create the X matrix
  for (size_t nRow = 0; nRow < nCount; ++nRow)
  {
    float nVal = 1.0f;
    for (int nCol = 0; nCol < nDegree; ++nCol)
    {
      oXMatrix(nRow, nCol) = nVal;
      nVal *= line[nRow].x;
    }
  }
  // transpose X matrix
  boost::numeric::ublas::matrix<float> oXtMatrix(trans(oXMatrix));
  // multiply transposed X matrix with X matrix
  boost::numeric::ublas::matrix<float> oXtXMatrix(prec_prod(oXtMatrix, oXMatrix));
  // multiply transposed X matrix with Y matrix
  boost::numeric::ublas::matrix<float> oXtYMatrix(prec_prod(oXtMatrix, oYMatrix));

  // lu decomposition
  boost::numeric::ublas::permutation_matrix<int> pert(oXtXMatrix.size1());
  const std::size_t singular = lu_factorize(oXtXMatrix, pert);

  // must be singular
  if (singular != 0)
    return false;

  // backsubstitution
  lu_substitute(oXtXMatrix, pert, oXtYMatrix);

  // copy the result to coeff

  std::vector<float> vec_temp(oXtYMatrix.data().begin(), oXtYMatrix.data().end());
  vec = vec_temp;
  return true;
}

float getY(std::vector<float> coeff, float x)
{
  size_t nDegree = coeff.size();

  float nY = 0;
  float nXdouble = 1;
  float nX = x;

  for (size_t j = 0; j < nDegree; ++j)
  {
    // multiply current x by a coefficient
    nY += coeff[j] * nXdouble;
    // power up the X
    nXdouble *= nX;
  }
  return nY;
}

private:
  int poly_degree_{3};
  int num_particles_{0};
  int num_control_points_{0};
  bool is_initialized_{false};
  std::vector<double> weights_;
  std::vector<Particle> particles_;
};

#endif  //  SELFIE_PARTICLE_FILTER_H