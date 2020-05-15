#ifndef CONSTANT_VALUES_HPP
#define	CONSTANT_VALUES_HPP

const int MAX_ITERATIONS = 1000; // maximum number of iterations in the RRT 
const double EXPLOITATION_RATE = 0.3; // between 0 and 1 (it is a threshold for the probability)
const double GOAL_TOLERANCE = 0.01; // it means 1 cm
const int MAX_NUM_EXP_PER_VERTEX = 10; // maximum number of times that a certain vertex can be used for an expansion attempt (both successful or not)  

#endif
