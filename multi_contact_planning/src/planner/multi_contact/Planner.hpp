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
#include <cartesian_interface/CartesianInterfaceImpl.h>

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
        XBot::Cartesian::CartesianInterfaceImpl::Ptr ci;
        
        XBot::Cartesian::Planning::ValidityCheckContext vc_context;

        std::vector<Stance> sigmaList2ndStage;
        std::vector<Configuration> qList2ndStage;

        int n_dof;

        int sol_len;

		bool isGoalStance(Vertex* v);
		Eigen::Vector3d pickRandomPoint(); 
		Eigen::Vector3d pickRandomPointInReachableWorkspace(Configuration q); // NOT USED
		Eigen::Vector3d pickRandomPointInReachableWorkspace(EndEffector pk, Configuration q); // NOT USED
		EndEffector pickRandomEndEffector();
		Contact* pickRandomContactFromGoalStance();
		int findNearestVertexIndex(EndEffector pk, Eigen::Vector3d r);
		bool isGoalContactReachable(EndEffector pk, Configuration q); // NOT USED
		Eigen::Vector3d getNormalAtPoint(Eigen::Vector3d p);	
		Eigen::Vector3d getNormalAtPointByIndex(int index);
		bool computeIKSolution(Stance sigma, Eigen::Vector3d rCoM, Configuration &q, Configuration q_ref);
		bool computeCentroidalStatics(std::vector<EndEffector> activeEEsDes, Eigen::Vector3d rCoMdes, Eigen::MatrixXd rCdes, Eigen::MatrixXd nCdes, Eigen::Vector3d &rCoM, Eigen::MatrixXd &rC, Eigen::MatrixXd &FC);

		Eigen::Vector3d computeCoM(Configuration qNear);
		Eigen::Affine3d computeForwardKinematics(Configuration q, EndEffector ee);
		Eigen::Matrix3d generateRandomRotationAroundAxis(Eigen::Vector3d axis);
		Eigen::Matrix3d generateRotationAroundAxis(EndEffector pk, Eigen::Vector3d axis);
		std::string getTaskStringName(EndEffector ee);
		bool nonEmptyReachableWorkspace(EndEffector pk, Configuration q);
		Eigen::Vector3d pickPointInReachableWorkspace(EndEffector pk, Configuration q, Eigen::Vector3d rRand);
		Eigen::Vector3d pickPointInReachableWorkspace(EndEffector pk, Configuration q, Eigen::Vector3d rRand, int &index);

		Eigen::Vector3d getNormalAtContact(EndEffector ee, Eigen::Matrix3d rot);

		Eigen::Vector3d generateRandomCoM(Configuration qNear);

		bool computeIKSolutionWithoutCoM(Stance sigma, Configuration &q, Configuration q_ref);
		bool computeIKSolutionWithCoM(Stance sigma, Eigen::Vector3d rCoM, Configuration &q, Configuration q_ref);

		bool similarityTest(Configuration qNew);
		bool similarityTest(Configuration qNew, Stance sigmaNew);
		bool similarityTest(Stance sigmaNew);

		void updateEndEffectorsList(Configuration qNew, Stance sigmaNew);

		

	public:

        //Planner(Configuration _qInit, Eigen::MatrixXd _posActiveEEsInit, Eigen::MatrixXd _contactForcesInit, std::vector<EndEffector> _activeEEsInit, Eigen::MatrixXd _posActiveEEsGoal, std::vector<EndEffector> _activeEEsGoal, Eigen::MatrixXd _pointCloud, Eigen::MatrixXd _pointNormals);
		//Planner(Configuration _qInit, std::vector<Eigen::Affine3d> _poseActiveEEsInit, std::vector<Eigen::Vector3d> _contactForcesInit, std::vector<EndEffector> _activeEEsInit, std::vector<Eigen::Affine3d> _poseActiveEEsGoal, std::vector<EndEffector> _activeEEsGoal, Eigen::MatrixXd _pointCloud, Eigen::MatrixXd _pointNormals, std::vector<EndEffector> _allowedEEs, XBot::ModelInterface::Ptr _planner_model, GoalGenerator::Ptr _goal_generator);
		//Planner(Configuration _qInit, std::vector<Eigen::Affine3d> _poseActiveEEsInit, std::vector<EndEffector> _activeEEsInit, std::vector<Eigen::Affine3d> _poseActiveEEsGoal, std::vector<EndEffector> _activeEEsGoal, Eigen::MatrixXd _pointCloud, Eigen::MatrixXd _pointNormals, std::vector<EndEffector> _allowedEEs, XBot::ModelInterface::Ptr _planner_model, GoalGenerator::Ptr _goal_generator);
		Planner(Configuration _qInit, std::vector<Eigen::Affine3d> _poseActiveEEsInit, std::vector<EndEffector> _activeEEsInit, std::vector<Eigen::Affine3d> _poseActiveEEsGoal, std::vector<EndEffector> _activeEEsGoal, Eigen::MatrixXd _pointCloud, Eigen::MatrixXd _pointNormals, std::vector<EndEffector> _allowedEEs, XBot::ModelInterface::Ptr _planner_model, GoalGenerator::Ptr _goal_generator, XBot::Cartesian::Planning::ValidityCheckContext _vc_context);
		Planner(Configuration _qInit, std::vector<EndEffector> _activeEEsInit, Configuration _qGoal, std::vector<EndEffector> _activeEEsGoal, Eigen::MatrixXd _pointCloud, Eigen::MatrixXd _pointNormals, std::vector<EndEffector> _allowedEEs, XBot::ModelInterface::Ptr _planner_model, GoalGenerator::Ptr _goal_generator, XBot::Cartesian::Planning::ValidityCheckContext _vc_context);

		~Planner(); 
		int getTreeSize();

		void run1stStage();
		void run2ndStage(std::vector<Stance> sigmaList, std::vector<Configuration> qList);
		bool retrieveSolution1stStage(std::vector<Stance> &sigmaList, std::vector<Configuration> &qList);
		bool retrieveSolution2ndStage(std::vector<Stance> &sigmaList, std::vector<Configuration> &qList);	

		void run();
		bool retrieveSolution(std::vector<Stance> &sigmaList, std::vector<Configuration> &qList);

		void runSingleStage();
				
};

#endif
