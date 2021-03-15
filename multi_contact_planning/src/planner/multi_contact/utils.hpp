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

    if(ee == L_HAND_C) ee_str = "TCP_L";
    else if(ee == R_HAND_C) ee_str = "TCP_R";
    else if(ee == L_HAND_D) ee_str = "l_ball_tip_d";
    else if(ee == R_HAND_D) ee_str = "r_ball_tip_d";
    else if(ee == L_FOOT) ee_str = "l_sole";
    else if(ee == R_FOOT) ee_str = "r_sole";
    else if(ee == HEAD) ee_str = "Head";
    else ee_str = "com";

    return ee_str;
}

inline std::vector<std::string> getTasksStringName(EndEffector ee){
    std::vector<std::string> ee_str;

    if(ee == L_HAND_C){
        ee_str.push_back("LeftHandCNoZ");
        ee_str.push_back("LeftHandCZ");
    }
    else if(ee == R_HAND_C){
        ee_str.push_back("RightHandCNoZ");
        ee_str.push_back("RightHandCZ");
    }
    else if(ee == L_HAND_D){
        ee_str.push_back("LeftHandDNoZ");
        ee_str.push_back("LeftHandDZ");
    }
    else if(ee == R_HAND_D){
        ee_str.push_back("RightHandDNoZ");
        ee_str.push_back("RightHandDZ");
    }
    else if(ee == L_FOOT){
        ee_str.push_back("LeftFootNoZ");
        ee_str.push_back("LeftFootZ");
    }
    else if(ee == R_FOOT){
        ee_str.push_back("RightFootNoZ");
        ee_str.push_back("RightFootZ");
    }
    
    return ee_str;
}

inline std::string getMainTaskStringName(std::string ee_str){
    std::string ee;
    
    if(ee_str.compare("LeftHandCNoZ") == 0) ee = "TCP_L";
    else if(ee_str.compare("LeftHandCZ") == 0) ee = "TCP_L"; 
    else if(ee_str.compare("RightHandCNoZ") == 0) ee = "TCP_R"; 
    else if(ee_str.compare("RightHandCZ") == 0) ee = "TCP_R"; 
    else if(ee_str.compare("LeftHandDNoZ") == 0) ee = "l_ball_tip_d"; 
    else if(ee_str.compare("LeftHandDZ") == 0) ee = "l_ball_tip_d"; 
    else if(ee_str.compare("RightHandDNoZ") == 0) ee = "r_ball_tip_d"; 
    else if(ee_str.compare("RightHandDZ") == 0) ee = "r_ball_tip_d"; 
    else if(ee_str.compare("LeftFootNoZ") == 0) ee = "l_sole"; 
    else if(ee_str.compare("LeftFootZ") == 0) ee = "l_sole"; 
    else if(ee_str.compare("RightFootNoZ") == 0) ee = "r_sole"; 
    else if(ee_str.compare("RightFootZ") == 0) ee = "r_sole"; 

    return ee;
}

inline EndEffector getTaskEndEffectorName(std::string ee_str){
    EndEffector ee;

    if(ee_str.compare("TCP_L") == 0) ee = L_HAND_C;
    else if(ee_str.compare("TCP_R") == 0) ee = R_HAND_C;
    else if(ee_str.compare("l_ball_tip_d") == 0) ee = L_HAND_D;
    else if(ee_str.compare("r_ball_tip_d") == 0) ee = R_HAND_D;
    else if(ee_str.compare("l_sole") == 0) ee = L_FOOT;
    else if(ee_str.compare("r_sole") == 0) ee = R_FOOT;
    else if(ee_str.compare("Head") == 0) ee = HEAD;
    else ee = COM;

    return ee;
}

inline Eigen::Matrix3d generateRotationFrictionCone(Eigen::Vector3d axis){
    Eigen::Matrix3d rot;
    
    Eigen::Vector3d n1(0.0, 0.0, +1.0);
    Eigen::Vector3d n2(0.0, -1.0, 0.0);
    Eigen::Vector3d n3(-1.0, 0.0, 0.0);
    Eigen::Vector3d n4(0.0, +1.0, 0.0);

    Eigen::Vector3d d1 = axis - n1;
    Eigen::Vector3d d2 = axis - n2;
    Eigen::Vector3d d3 = axis - n3;
    Eigen::Vector3d d4 = axis - n4;
    
    if(d1.norm() < 1e-3){
        rot << 1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0;
    }
    else if(d2.norm() < 1e-3){
        rot << 0.0, -1.0, 0.0,
                0.0, 0.0, -1.0,
                1.0, 0.0, 0.0;
    }
    else if(d3.norm() < 1e-3){
        rot << 0.0, 0.0, -1.0,
                0.0, 1.0, 0.0,
                1.0, 0.0, 0.0;
    }
    else if(d4.norm() < 1e-3){
        rot << 0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
                1.0, 0.0, 0.0;
    }
    else rot.setZero();
        
    return rot;
}

inline Eigen::Matrix3d generateRotationAroundAxis(EndEffector ee, Eigen::Vector3d axis){
    Eigen::Matrix3d rot;
        
    Eigen::Vector3d n1(0.0, 0.0, +1.0);
    Eigen::Vector3d n2(0.0, -1.0, 0.0);
    Eigen::Vector3d n3(-1.0, 0.0, 0.0);
    Eigen::Vector3d n4(0.0, +1.0, 0.0);

    Eigen::Vector3d d1 = axis - n1;
    Eigen::Vector3d d2 = axis - n2;
    Eigen::Vector3d d3 = axis - n3;
    Eigen::Vector3d d4 = axis - n4;

    if(ee == L_HAND_C || ee == R_HAND_C || ee == L_HAND_D || ee == R_HAND_D){
        if(d1.norm() < 1e-3){
            rot << -1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, -1.0;
        }
        else if(d2.norm() < 1e-3){
            rot << 0.0, -1.0, 0.0,
                    0.0, 0.0, 1.0,
                    -1.0, 0.0, 0.0;
        }
        else if(d3.norm() < 1e-3){
            rot << 0.0, 0.0, 1.0,
                    0.0, 1.0, 0.0,
                    -1.0, 0.0, 0.0;
        }
        else if(d4.norm() < 1e-3){
            rot << 0.0, 1.0, 0.0,
                    0.0, 0.0, -1.0,
                    -1.0, 0.0, 0.0;
        }
        else rot.setZero();
    }
    else if(ee == L_FOOT || ee == R_FOOT){
        if(d1.norm() < 1e-3){
            rot << 1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0;
        }
        else if(d2.norm() < 1e-3){
            rot << 0.0, -1.0, 0.0,
                    0.0, 0.0, -1.0,
                    1.0, 0.0, 0.0;
        }
        else if(d3.norm() < 1e-3){
            rot << 0.0, 0.0, -1.0,
                    0.0, 1.0, 0.0,
                    1.0, 0.0, 0.0;
        }
        else if(d4.norm() < 1e-3){
            rot << 0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0,
                    1.0, 0.0, 0.0;
        }
        else rot.setZero();
    }
    else rot.setZero();
    
    return rot;
}



#endif
