// Fit a polynomial.
// Adapted from:
// https://github.com/patLoeber/Polyfit/blob/master/PolyfitBoost.hpp

#ifndef SELFIE_POLYNOMIALS_H
#define SELFIE_POLYNOMIALS_H

#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <opencv2/highgui/highgui.hpp>
//#define BOOST_UBLAS_TYPE_CHECK 0

bool polyfit(const std::vector<cv::Point2f> &points, int degree, std::vector<float> &coeff);
float getPolyY(const std::vector<float> &coeff, float x);

#endif  //  SELFIE_POLYNOMIALS_H