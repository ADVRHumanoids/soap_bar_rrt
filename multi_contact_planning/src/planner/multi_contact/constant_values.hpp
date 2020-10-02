#ifndef CONSTANT_VALUES_HPP
#define	CONSTANT_VALUES_HPP

const int MAX_ITERATIONS = 500; 
const double EXPLOITATION_RATE = 0.9; // between 0 and 1 (it is a threshould for the probability)
const double GOAL_TOLERANCE = 0.01; // it means 1 cm
const int MAX_NUM_EXP_PER_VERTEX = 5; //3; // maximum number of times that a certain vertex can be used for an expansion attempt (both successful or not) 
 
const double WORKSPACE_RADIUS = 0.25;  //1.5;  
const double WORKSPACE_RADIUS_FOOT = 0.40;//0.25;   
const double WORKSPACE_RADIUS_HAND = 0.80;   
const double GOAL_SAMPLER_TIME_BUDGET = 5.0;

const double ROBOT_MASS = 70.0;
const double GRAVITY = -9.81;
const double MU_FRICTION = 0.5;
const double FORCE_THRES = 20.0; //10.0;
const double COM_WEIGHT_CPL = 100.0; //10000.0;
const double COM_REF_HEIGHT = 0.915;

#endif

