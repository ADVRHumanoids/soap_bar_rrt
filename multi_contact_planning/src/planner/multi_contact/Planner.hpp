#ifndef PLANNER_H
#define PLANNER_H

#include <stdio.h>
#include <stdlib.h> 
#include <vector>
#include <map>
#include <iostream>
#include <chrono>
#include <fstream>
#include <random>
#include <limits>
#include "enum.h"
#include <Eigen/Dense>
#include "constant_values.hpp"
#include <CentroidalPlanner/CentroidalPlanner.h>
#include <CentroidalPlanner/CoMPlanner.h>
#include "Contact.hpp"
#include "Configuration.hpp"
#include "Vertex.hpp"
#include "Tree.hpp"

#include <XBotInterface/ModelInterface.h>
#include "nodes/goal_generation.h"
#include "goal/NSPG.h"
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <multi_contact_planning/SetContactFrames.h>

#include <fstream>


class Planner {

	private:

		Configuration qInit; 
		Stance sigmaInit;
		Configuration qGoal; 
		Stance sigmaGoal;
		Eigen::MatrixXd pointCloud;
		Eigen::MatrixXd pointNormals;
		Tree* tree;
        std::vector<EndEffector> endEffectorsList;

        XBot::ModelInterface::Ptr planner_model;
        GoalGenerator::Ptr goal_generator;
        XBot::Cartesian::Planning::NSPG::Ptr NSPG;
        
        XBot::Cartesian::CartesianInterfaceImpl::Ptr ci;
        
        XBot::Cartesian::Planning::ValidityCheckContext vc_context;

        int n_dof;
        Eigen::VectorXd qmin, qmax;

		bool isGoalStance(Vertex* v);
		Eigen::Vector3d pickRandomPoint();
		bool nonEmptyReachableWorkspace(EndEffector pk, Configuration q);
		Eigen::Vector3d pickPointInReachableWorkspace(EndEffector pk, Configuration q, Eigen::Vector3d rRand, int &index);
		EndEffector pickRandomEndEffector();
		Contact* pickRandomContactFromGoalStance();
		int findNearestVertexIndex(EndEffector pk, Eigen::Vector3d r);
		std::string getTaskStringName(EndEffector ee);
		EndEffector getTaskEndEffectorName(std::string ee_str);
		bool computeIKSolution(Stance sigma, bool refCoM, Eigen::Vector3d rCoM, Configuration &q, Configuration qPrev);
		bool computeCentroidalStatics(std::vector<EndEffector> activeEEsDes, Eigen::Vector3d rCoMdes, Eigen::MatrixXd rCdes, Eigen::MatrixXd nCdes, Eigen::Vector3d &rCoM, Eigen::MatrixXd &rC, Eigen::MatrixXd &FC);
		Eigen::Vector3d getNormalAtPoint(Eigen::Vector3d p);
		Eigen::Vector3d getNormalAtPointByIndex(int index);
		Eigen::Vector3d computeCoM(Configuration q);
		Eigen::Affine3d computeForwardKinematics(Configuration q, EndEffector ee);
		bool similarityTest(Stance sigmaNew);
		double computeHrange(Configuration q);
		double computeHtorso(Configuration q);	
                Eigen::Matrix3d generateRotationAroundAxis(EndEffector pk, Eigen::Vector3d axis);
                Eigen::Matrix3d generateRotationFrictionCone(Eigen::Vector3d axis);

        ros::NodeHandle _nh;
        ros::Publisher _pub;

	public:

        Planner(Configuration _qInit, 
                std::vector<EndEffector> _activeEEsInit, Configuration _qGoal, 
                std::vector<EndEffector> _activeEEsGoal, Eigen::MatrixXd _pointCloud, 
                Eigen::MatrixXd _pointNormals, std::vector<EndEffector> _allowedEEs, 
                XBot::ModelInterface::Ptr _planner_model, 
                XBot::Cartesian::Planning::NSPG::Ptr _NSPG,
                XBot::Cartesian::Planning::ValidityCheckContext _vc_context,
                ros::NodeHandle& nh);
		~Planner();
		void run();
		bool retrieveSolution(std::vector<Stance> &sigmaList, std::vector<Configuration> &qList);
		int getTreeSize();
				
};

#endif






