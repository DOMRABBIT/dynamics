#include "spatial.h"

using namespace Eigen;

Matrix<double, 6, 6> xlt(Matrix<double, 3, 1> r)
{
    Matrix<double, 6, 6> X;
    X << 1,    0,     0,    0, 0, 0,
         0,    1,     0,    0, 0, 0,
         0,    0,     1,    0, 0, 0,
         0,    -r[2], r[1], 1, 0, 0,
        r[2], 0,     -r[0], 0, 1, 0,
        -r[1], r[0],  0,    0, 0, 1;

    return X;
}

Matrix<double, 6, 6> rotx(double theta)
{
    Matrix<double, 6, 6> X;

    double c = cos(theta);
    double s = sin(theta);

    X << 1,  0, 0, 0,  0, 0,
         0,  c, -s, 0,  0, 0,
         0, s, c, 0,  0, 0,
         0,  0, 0, 1,  0, 0,
         0,  0, 0, 0,  c, -s,
         0,  0, 0, 0, s, c;

    return X;
}

Matrix<double, 6, 6> roty(double theta)
{
    Matrix<double, 6, 6> X;

    double c = cos(theta);
    double s = sin(theta);

    X << c, 0, s, 0, 0,  0,
         0, 1,  0, 0, 0,  0,
         -s, 0,  c, 0, 0,  0,
         0, 0,  0, c, 0, s,
         0, 0,  0, 0, 1,  0,
         0, 0,  0, -s, 0,  c;

    return X;
}

Matrix<double, 6, 6> rotz(double theta)
{
    Matrix<double, 6, 6> X;

    double c = cos(theta);
    double s = sin(theta);

    X << c, -s, 0,  0, 0, 0,
        s, c, 0,  0, 0, 0,
         0, 0, 1,  0, 0, 0,
         0, 0, 0,  c, -s, 0,
         0, 0, 0, s, c, 0,
         0, 0, 0,  0, 0, 1;

    return X;
}

Matrix<double, 6, 6> rotR(Matrix<double, 3, 3> R)
{
    Matrix<double, 6, 6> X;
    X.setZero(6, 6);
    X.block(0, 0, 3, 3) = R;
    X.block(3, 3, 3, 3) = R;

    return X;
}

Matrix<double, 4, 4> Rp2T(Matrix<double, 3, 3> R, Matrix<double, 3, 1> p)
{
    Matrix<double, 4, 4> T;
    T.setZero(4, 4);
    T(3, 3) = 1;
    T.block(0, 0, 3, 3) = R;
    T.block(0, 3, 3, 1) = p;
    return T;
}

Matrix<double, 4, 4> rox(double theta)
{
    Matrix<double, 4, 4> R;

    double c = cos(theta);
    double s = sin(theta);

    R << 1, 0, 0, 0,
        0, c, -s, 0,
        0, s, c, 0,
        0, 0, 0, 1;

    return R;
}

Matrix<double, 4, 4> roy(double theta)
{
    Matrix<double, 4, 4> R;

    double c = cos(theta);
    double s = sin(theta);

    R << c, 0, s, 0,
        0, 1, 0, 0,
        -s, 0, c, 0,
        0, 0, 0, 1;

    return R;
}

Matrix<double, 4, 4> roz(double theta)
{
    Matrix<double, 4, 4> R;

    double c = cos(theta);
    double s = sin(theta);

    R << c, -s, 0, 0,
        s, c, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return R;
}

// q(0:3):quaternion   
// q(4:6):xyz
Matrix<double, 4, 4> Flt_Transform(double q[])
{
    Matrix<double, 4, 4> T;
    Matrix3d R;
    Vector3d xyz;
    xyz << q[4], q[5], q[6];

    Quaterniond qua(q[0],q[1],q[2],q[3]); //w x y z
    qua.normalize();
    R = qua.matrix();
    T.setIdentity(4, 4);
    T.block(0, 0, 3, 3) = R;
    T.block(0, 3, 3, 1) = xyz;
    return T;
}

Matrix<double,6,6> crm(Matrix<double,6,1> v)
{
    Matrix<double, 6, 6> vcross;
    vcross << 0,   -v(2), v(1), 0,    0,    0,
              v(2), 0,   -v(0), 0,    0,    0,
             -v(1), v(0), 0,    0,    0,    0,
              0,   -v(5), v(4), 0,   -v(2), v(1),
              v(5), 0,   -v(3), v(2), 0,   -v(0),
             -v(4), v(3), 0,   -v(1), v(0), 0;

    return vcross;
}

Matrix<double, 6, 6> crf(Matrix<double, 6, 1> v)
{
    Matrix<double, 6, 6> vcross;
    vcross = -crm(v).transpose();

    return vcross;
}