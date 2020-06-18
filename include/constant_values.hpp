#ifndef CONSTANT_VALUES_HPP
#define	CONSTANT_VALUES_HPP

const int MAX_ITERATIONS = 200; 
const double EXPLOITATION_RATE = 0.3; // between 0 and 1 (it is a threshould for the probability)
const double GOAL_TOLERANCE = 0.05; // it means 1 cm
const int MAX_NUM_EXP_PER_VERTEX = 10; // maximum number of times that a certain vertex can be used for an expansion attempt (both successful or not) 
const double KINEMATIC_CONTROL_ERROR_TOL = 0.002; 
const double WORKSPACE_RADIUS = 0.8;

#endif


