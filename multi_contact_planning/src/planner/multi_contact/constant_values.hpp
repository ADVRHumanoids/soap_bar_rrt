#ifndef CONSTANT_VALUES_HPP
#define	CONSTANT_VALUES_HPP

const int MAX_ITERATIONS = 10000;
const double EXPLOITATION_RATE = 1.0; //0.9; // between 0 and 1 (it is a threshold for the probability)
const double GOAL_TOLERANCE = 0.05; // it means 1 cm
const int MAX_NUM_EXP_PER_VERTEX = 5; // maximum number of times that a certain vertex can be used for an expansion attempt (both successful or not)

const int NUM_CONF_PER_VERTEX = 1; //5; // number of different configurations generated for an expansion attempt (both successful or not)
 
const double WORKSPACE_RADIUS_FOOT = 0.23; //0.40;  
const double WORKSPACE_RADIUS_HAND = 0.23;
const double GOAL_SAMPLER_TIME_BUDGET = 3.0;
const double GOAL_SAMPLER_TIME_BUDGET_COM = 3.0; 

const double ROBOT_MASS = 70.0;
const double GRAVITY = -9.81;
const double MU_FRICTION = 0.8; //0.5;
const double FORCE_THRES_HAND = 15.0;
const double FORCE_THRES_FOOT = 65.0;
const double COM_WEIGHT_CPL = 1000; //100.0; 
const double COM_REF_HEIGHT = 0.915;

const double DIST_THRES = 0.65;

const double DIST_THRES_MIN = 0.60;
const double DIST_THRES_MAX = 1.80;
const double DIST_HANDS_THRES_MAX = 1.0;

const double CS_THRES = 5*1e-2;

const int SCENARIO = 3;
const double RESOLUTION = 0.05;

const int INIT_INDEX = 0;
const int GOAL_INDEX = 2;

#endif
