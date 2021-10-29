#ifndef CONSTANT_VALUES_HPP
#define	CONSTANT_VALUES_HPP

const int NUM_SIM = 1;

// planner
const int MAX_ITERATIONS = 5000; //10000;
const double EXPLOITATION_RATE = 1.0; //0.9; // between 0 and 1 (it is a threshold for the probability)
const double GOAL_TOLERANCE = 0.05; // it means 5 cm
const int MAX_NUM_EXP_PER_VERTEX = 10; // maximum number of times that a certain vertex can be used for an expansion attempt (both succ or not)
const double DIST_THRES_MAX_HANDS = 0.3;
const double DIST_THRES_MAX_FEET = 0.3;


// cpl
const double ROBOT_MASS = 70.0;
const double GRAVITY = -9.81;
const double FORCE_THRES_HAND = 15.0;
const double FORCE_THRES_FOOT = 65.0;
const double COM_WEIGHT_CPL = 1000; //100.0; 
const double COM_REF_HEIGHT = 0.915;
const double GOAL_SAMPLER_TIME_BUDGET_COM = 3.0; 

// point cloud
const double RESOLUTION = 0.05;

// centroidal statics
const double CS_THRES = 0.5*1e-2;

// nspg
const bool RAND_VEL_CHAINS = false;
const double DT = 0.002; //0.001
const int ITER_MAX = 100;
const double GAIN_VEL_FB_X = 100; //500;
const double GAIN_VEL_FB_Y = 100; //250;
const double GAIN_VEL_FB_Z = 50; //100;
const double FORCE_THRES = 10.0;
const double CoP_LIM_X = 0.08;
const double CoP_LIM_Y = 0.035;


// SCENARIO 1, PHASE 0
//const int SCENARIO = 1;
//const int INIT_INDEX = 0;
//const int GOAL_INDEX = 1;
//const double MU_FRICTION = 0.5;
//const double MU_FRICTION_HANDS = 0.5;
//const double WORKSPACE_RADIUS_FOOT = 0.33;
//const double WORKSPACE_RADIUS_HAND = 0.33;
//const double GOAL_SAMPLER_TIME_BUDGET = 3.0;
//const bool FREE_YAW_ROTATION = false;
//const double DIST_HANDS_THRES_MAX = 1.0;
//const double DIST_THRES_MIN = 0.60;
//const double DIST_THRES_MAX = 1.80;



// SCENARIO 1, PHASE 1
const int SCENARIO = 1;
const int INIT_INDEX = 1;
const int GOAL_INDEX = 2;
const double MU_FRICTION = 0.5;
const double MU_FRICTION_HANDS = 0.5;
const double WORKSPACE_RADIUS_FOOT = 0.23;
const double WORKSPACE_RADIUS_HAND = 0.23;
const double GOAL_SAMPLER_TIME_BUDGET = 3.0;
const bool FREE_YAW_ROTATION = false;
const double DIST_HANDS_THRES_MAX = 1.0;
const double DIST_THRES_MIN = 0.6;
const double DIST_THRES_MAX = 0.95;



// SCENARIO 1, PHASE 2
//const int SCENARIO = 1;
//const int INIT_INDEX = 2;
//const int GOAL_INDEX = 3;
//const double MU_FRICTION = 0.4;
//const double MU_FRICTION_HANDS = 0.4;
//const double WORKSPACE_RADIUS_FOOT = 0.3;
//const double WORKSPACE_RADIUS_HAND = 0.8;
//const double GOAL_SAMPLER_TIME_BUDGET = 3.0;
//const bool FREE_YAW_ROTATION = false;
//const double DIST_HANDS_THRES_MAX = 1.0;
//const double DIST_THRES_MIN = 0.60;
//const double DIST_THRES_MAX = 1.80;


/* //LUCA
const int SCENARIO = 1;
const int INIT_INDEX = 0;
const int GOAL_INDEX = 1;
const double MU_FRICTION = 0.5;  
const double MU_FRICTION_HANDS = 0.5;  
const double WORKSPACE_RADIUS_FOOT = 0.23;  
const double WORKSPACE_RADIUS_HAND = 0.23;
const double GOAL_SAMPLER_TIME_BUDGET = 3.0;
const bool FREE_YAW_ROTATION = false;
const double DIST_HANDS_THRES_MAX = 1.0;
*/

/*
const int SCENARIO = 1;
const int INIT_INDEX = 2;
const int GOAL_INDEX = 3;
const double MU_FRICTION = 0.5;
const double MU_FRICTION_HANDS = 0.5;
const double WORKSPACE_RADIUS_FOOT = 0.3;
const double WORKSPACE_RADIUS_HAND = 0.8;
const double GOAL_SAMPLER_TIME_BUDGET = 3.0;
const bool FREE_YAW_ROTATION = false;
const double DIST_HANDS_THRES_MAX = 1.0;
*/

/*
// SCENARIO 2, PHASE 0
const int SCENARIO = 2;
const int INIT_INDEX = 0;
const int GOAL_INDEX = 1;
const double MU_FRICTION = 0.8;  
const double MU_FRICTION_HANDS = 0.8;  
const double WORKSPACE_RADIUS_FOOT = 0.63;  
const double WORKSPACE_RADIUS_HAND = 0.63;
const double GOAL_SAMPLER_TIME_BUDGET = 3.0;
const bool FREE_YAW_ROTATION = false;
const double DIST_HANDS_THRES_MAX = 1.0;
const double DIST_THRES_MIN = 0.60;
const double DIST_THRES_MAX = 1.80;
*/

/*
// SCENARIO 2, PHASE 1
const int SCENARIO = 2;
const int INIT_INDEX = 1;
const int GOAL_INDEX = 2;
const double MU_FRICTION = 0.8;  
const double MU_FRICTION_HANDS = 0.8;  
const double WORKSPACE_RADIUS_FOOT = 0.23;    
const double WORKSPACE_RADIUS_HAND = 0.23;  
const double GOAL_SAMPLER_TIME_BUDGET = 3.0;
const bool FREE_YAW_ROTATION = false;
const double DIST_HANDS_THRES_MAX = 1.0;
const double DIST_THRES_MIN = 0.60;
const double DIST_THRES_MAX = 1.80;
*/

/*
// SCENARIO 3
const int SCENARIO = 3;
const int INIT_INDEX = 0;
const int GOAL_INDEX = 2;
const double MU_FRICTION = 0.8;  
const double MU_FRICTION_HANDS = 0.8;  
const double WORKSPACE_RADIUS_FOOT = 0.23;  
const double WORKSPACE_RADIUS_HAND = 0.23;
const double GOAL_SAMPLER_TIME_BUDGET = 3.0;
const bool FREE_YAW_ROTATION = false;
const double DIST_HANDS_THRES_MAX = 1.0;
const double DIST_THRES_MIN = 0.60;
const double DIST_THRES_MAX = 1.80;
*/

#endif
