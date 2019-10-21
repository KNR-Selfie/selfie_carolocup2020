#include <selfie_perception/polynomials.h>

bool polyfit(const std::vector<cv::Point2f> &points, int degree, std::vector<float> &coeff)
{
  if (points.size() < degree)
    return false;

  // more intuative this way
  ++degree;

  size_t nCount = points.size();
  boost::numeric::ublas::matrix<float> oXMatrix(nCount, degree);
  boost::numeric::ublas::matrix<float> oYMatrix(nCount, 1);

  // copy y matrix
  for (size_t i = 0; i < nCount; ++i)
  {
    oYMatrix(i, 0) = points[i].y;
  }

  // create the X matrix
  for (size_t nRow = 0; nRow < nCount; ++nRow)
  {
    float nVal = 1.0f;
    for (int nCol = 0; nCol < degree; ++nCol)
    {
      oXMatrix(nRow, nCol) = nVal;
      nVal *= points[nRow].x;
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
  coeff = vec_temp;
  return true;
}

float getPolyY(const std::vector<float> &coeff, float x)
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