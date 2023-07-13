#ifndef _SPATIAL_H_
#define _SPATIAL_H_

#include <eigen3/Eigen/Dense>
#include <math.h>

using namespace Eigen;

Matrix<double, 6, 6> xlt(Matrix<double, 3, 1> r);
Matrix<double, 6, 6> rotx(double theta);
Matrix<double, 6, 6> roty(double theta);
Matrix<double, 6, 6> rotz(double theta);
Matrix<double, 6, 6> rotR(Matrix<double, 3, 3> R);
Matrix<double, 4, 4> Rp2T(Matrix<double, 3, 3> R, Matrix<double, 3, 1> p);
Matrix<double, 4, 4> rox(double theta);
Matrix<double, 4, 4> roy(double theta);
Matrix<double, 4, 4> roz(double theta);
Matrix<double, 4, 4> Flt_Transform(double q[]);
Matrix<double, 6, 6> crm(Matrix<double, 6, 1> v);
Matrix<double, 6, 6> crf(Matrix<double, 6, 1> v);
#endif