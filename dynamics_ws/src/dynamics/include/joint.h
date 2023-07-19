#ifndef _JOINT_H_
#define _JOINT_H_

#include <eigen3/Eigen/Dense>

enum _jtype {Rx,Ry,Rz,Px,Py,Pz,Flt};

class Joint
{
public:
    Joint()
    {
        jtype = Rz;
        value = 0;
        Set_S_body(jtype);
    }
    Joint(_jtype jt, float q)
    {
        jtype = jt;
        value = q;
        Set_S_body(jtype);
    }
    void Set_S_body(_jtype jty)
    {
        switch (jty)
        {
        case Rx:
            S_body << 1, 0, 0, 0, 0, 0;
            break;
        case Ry:
            S_body << 0, 1, 0, 0, 0, 0;
            break;
        case Rz:
            S_body << 0, 0, 1, 0, 0, 0;
            break;
        case Px:
            S_body << 0, 0, 0, 1, 0, 0;
            break;
        case Py:
            S_body << 0, 0, 0, 0, 1, 0;
            break;
        case Pz:
            S_body << 0, 0, 0, 0, 0, 1;
            break;
        case Flt:
            S_Fltbase.setIdentity(6, 6);
        default:
            S_body << 0, 0, 1, 0, 0, 0;
            break;
        }
    }

    _jtype jtype = Rz;
    Eigen::Matrix<double, 6, 1> S_body; // 运动子空间
    Eigen::Matrix<double, 6, 6> S_Fltbase;
    double value = 0.0f;
    double valueFlt[7] = {0, 0, 0, 0, 0, 0, 0};
};

#endif