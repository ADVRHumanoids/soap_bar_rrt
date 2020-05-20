#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "constant_values.hpp"


inline double euclideanDistance(Eigen::Vector3d v1, Eigen::Vector3d v2) {
	double dist = (double)sqrt(std::pow(v1(0) - v2(0), 2) + std::pow(v1(1) - v2(1), 2) + std::pow(v1(2) - v2(2), 2));
	return dist;
}

#endif
