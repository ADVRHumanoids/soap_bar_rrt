#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "constant_values.hpp"
#include "enum.h"

inline double euclideanDistance(Eigen::Vector3d v1, Eigen::Vector3d v2) {
	double dist = (double)sqrt(std::pow(v1(0) - v2(0), 2) + std::pow(v1(1) - v2(1), 2) + std::pow(v1(2) - v2(2), 2));
	return dist;
}

inline std::string getTaskStringName(EndEffector ee){
    std::string ee_str;

    if(ee == L_HAND) ee_str = "TCP_L";
    else if(ee == R_HAND) ee_str = "TCP_R";
    else if(ee == L_FOOT) ee_str = "l_sole";
    else if(ee == R_FOOT) ee_str = "r_sole";
    else if(ee == HEAD) ee_str = "Head";
    else ee_str = "com";

    return ee_str;
}

inline EndEffector getTaskEndEffectorName(std::string ee_str){
    EndEffector ee;

    if(ee_str.compare("TCP_L") == 0) ee = L_HAND;
    else if(ee_str.compare("TCP_R") == 0) ee = R_HAND;
    else if(ee_str.compare("l_sole") == 0) ee = L_FOOT;
    else if(ee_str.compare("r_sole") == 0) ee = R_FOOT;
    else if(ee_str.compare("Head") == 0) ee = HEAD;
    else ee = COM;

    return ee;
}

inline Eigen::Matrix3d generateRotationFrictionCone(Eigen::Vector3d axis)
{
    Eigen::Matrix3d rot;

    bool vertical = false;
    Eigen::Vector3d aux = axis - Eigen::Vector3d(0.0, 0.0, 1.0);
    if(abs(aux(0)) < 1e-3 && abs(aux(1)) < 1e-3 && abs(aux(2)) < 1e-3) vertical = true;

    if(vertical){
            rot << 1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0;
    }
    else{
            rot <<  0.0, 0.0, -1.0,
                    0.0, 1.0, 0.0,
                    1.0, 0.0, 0.0;
    }

    return rot;
}

#endif
