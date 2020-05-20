#ifndef PLANNER_H
#define PLANNER_H

// DA VEDERE COSA INCLUDERE
#include <stdio.h>
#include <vector>
#include <map>
#include <iostream>
#include "enum.h"
#include "Eigen/Dense"
#include "Vertex.hpp"
#include "Tree.hpp"
#include "matrix_tools.hpp"
#include "constant_values.hpp"
#include <limits>
// in this class it is assumed that there exist two classes:
//	-	Superquadric: an object of this class represents one of the SQs composing the environment (and contains all the info/parameters)
//	-	Kinematics: in particular I am calling inside the code a function forwardKinematics(q, ee) that is assumed to compute a Eigen::Vector6d representing the pose of the end-effector ee in the configuration q; in this vector the first 3 components represent the position (head(3)), while the last 3 represent the orientation (tail(3))

class Planner {

	private:

		Configuration qInit;
		Eigen::MatrixXd contactForcesInit;
        std::vector<EndEffector> activeEEsInit;
		Eigen::MatrixXd posActiveEEsGoal;
        std::vector<EndEffector> activeEEsGoal;
        std::vector<Superquadric*> superquadricsList;
		Stance sigmaInit;
		Stance sigmaGoal;
		Tree* tree;
        std::vector<EndEffector> endEffectorsList;

		bool isGoalStance(Vertex* v);
		Eigen::Vector3d pickRandomPoint();
        EndEffector pickRandomEndEffector();
		Contact* pickRandomContactFromGoalStance();
        int findNearestVertexIndex(EndEffector pk, Eigen::Vector3d r);
        bool isActiveEndEffector(EndEffector pk, std::vector<EndEffector> activeEEs);
        Eigen::MatrixXd computeReachableWorkspace(EndEffector pk, Configuration q);
        Eigen::Vector3d retrieveContactPosition(EndEffector pk, Stance sigma);
        bool isGoalContactReachable(EndEffector pk, Eigen::MatrixXd w);
        bool checkSuperquadricPointIntersection(Superquadric sq, Eigen::Vector3d  r);
        int retrieveSuperquadricIndex(Eigen::Vector3d r);
        bool checkSuperquadricWorkspaceIntersection(Superquadric sq, Eigen::MatrixXd w);
		std::vector<int> retrieveReachableSuperquadricsIndices(Eigen::MatrixXd w);
		int pickRandomSuperquadricIndex(std::vector<int> reachableSuperquadricsIndices);
        Stance computeCandidateContacts(Eigen::Vector3d rCoMDes, Eigen::MatrixXd rCDes, std::vector<EndEffector> activeEEsDes, int superquadricIndexDes, EndEfector pk, Action f);
		Configuration computeIKSolution(Stance sigma, bool &qFound);		
		
	public:

        Planner(Configuration _qInit, Eigen::MatrixXd _contactForcesInit, std::vector<EndEffector> _activeEEsInit, Eigen::MatrixXd _posActiveEEsGoal, std::vector<EndEffector> _activeEEsGoal, std::vector<Superquadric*> _superquadricsList);
		~Planner();
		void run();
		bool retrieveSolution(std::vector<Stance> &sigmaList, std::vector<Configuration> &qList);		
		int getTreeSize();
};

#endif
