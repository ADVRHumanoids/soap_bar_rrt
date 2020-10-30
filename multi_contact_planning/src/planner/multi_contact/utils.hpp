#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "constant_values.hpp"

inline double euclideanDistance(Eigen::Vector3d v1, Eigen::Vector3d v2) {
	double dist = (double)sqrt(std::pow(v1(0) - v2(0), 2) + std::pow(v1(1) - v2(1), 2) + std::pow(v1(2) - v2(2), 2));
	return dist;
}

inline double angleSignedDistance(double a, double b){
	double d = abs(a - b);
	while(d > 2.0*M_PI) d = d - 2.0*M_PI; 

	double r = 0.0;
	if(d > M_PI) r = 2.0*M_PI - d;
	else r = d;
	double sign = 0.0;
	if( (a-b>=0.0 && a-b<=M_PI) || (a-b<=-M_PI && a-b>=-2.0*M_PI) ) sign = +1.0;
	else sign = -1.0;

	r = sign * r;
	return r; 
}

#endif
