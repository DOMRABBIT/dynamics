#ifndef _BODY_H_
#define _BODY_H_

#include <eigen3/Eigen/Dense>
#include <iostream>


// 创建一个连杆，固定是圆柱
class Body 
{
public:
    Body()
    {
        radius = 0.05;
        length = 0.05;
        mass = 0.25;
        com << 0, 0, 0.025;
        Ic << 0.1, 0, 0,
            0, 0.1, 0,
            0, 0, 0.1;
    }
    Body(double m, Eigen::Vector3d c, Eigen::Matrix3d inertial_c)
    {
        radius = 0.05;
        length = 0.05;
        mass = m;
        com = c;
        Ic = inertial_c;
    }
    void print_CoM()
    {
        std::cout << this->com << std::endl;
    }
    void print_Ic()
    {
        std::cout << this->Ic << std::endl;
    }
    
    double radius = 0.0f;
    double length = 0.0f;
    double mass = 1.0f;
    Eigen::Matrix3d Ic = Eigen::Matrix3d::Zero(3,3);
    Eigen::Vector3d com = Eigen::Vector3d::Zero(3);
    std::string _type = "cyl";

};


#endif