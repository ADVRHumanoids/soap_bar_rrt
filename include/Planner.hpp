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


class Planner {

	private:

		Configuration qInit;
		Stance sigmaInit;
		Stance sigmaGoal;
		Eigen::MatrixXd pointCloud;
		Eigen::MatrixXd pointNormals;
		Tree* tree;
        std::vector<EndEffector> endEffectorsList;

		bool isGoalStance(Vertex* v);
		Eigen::Vector3d pickRandomPoint(); 
		Eigen::Vector3d pickRandomPointInReachableWorkspace(Configuration q, Eigen::MatrixXd w);
		EndEffector pickRandomEndEffector();
		Contact* pickRandomContactFromGoalStance();
		int findNearestVertexIndex(EndEffector pk, Eigen::Vector3d r);
		Eigen::MatrixXd computeReachableWorkspace(EndEffector pk, Configuration q);
		bool isGoalContactReachable(EndEffector pk, Configuration q, Eigen::MatrixXd w);
		Eigen::Vector3d getNormalAtPoint(Eigen::Vector3d p);	
		bool computeIKSolution(Stance sigma, Eigen::Vector3d rCoM, Configuration &q);
		bool computeCentroidalStatics(std::vector<EndEffector> activeEEsDes, Eigen::Vector3d rCoMdes, Eigen::MatrixXd rCdes, Eigen::MatrixXd nCdes, Eigen::Vector3d &rCoM, Eigen::MatrixXd &rC, Eigen::MatrixXd &FC);

	public:

        Planner(Configuration _qInit, Eigen::MatrixXd _posActiveEEsInit, Eigen::MatrixXd _contactForcesInit, std::vector<EndEffector> _activeEEsInit, Eigen::MatrixXd _posActiveEEsGoal, std::vector<EndEffector> _activeEEsGoal, Eigen::MatrixXd _pointCloud, Eigen::MatrixXd _pointNormals);
		~Planner(); 
		void run();
		bool retrieveSolution(std::vector<Stance> &sigmaList, std::vector<Configuration> &qList);
		int getTreeSize();

};

#endif
