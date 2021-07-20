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
#include "validity_checker/stability/centroidal_statics.h"

#include <fstream>
#include <memory>


class Planner {

    private:

        Configuration qInit;
        Stance sigmaInit;
        Configuration qGoal;
        Stance sigmaGoal;
        Eigen::MatrixXd pointCloud;
        Eigen::MatrixXd pointNormals;
        std::shared_ptr<Tree> tree;
        std::vector<EndEffector> endEffectorsList;

        XBot::ModelInterface::Ptr planner_model;
        XBot::Cartesian::Planning::NSPG::Ptr NSPG;

        XBot::Cartesian::CartesianInterfaceImpl::Ptr ci;

        XBot::Cartesian::Planning::ValidityCheckContext vc_context;

        ros::NodeHandle _nh;
        ros::Publisher _pub;

        int n_dof;
        Eigen::VectorXd qmin, qmax;

        bool isGoalStance(std::shared_ptr<Vertex> v);
        Eigen::Vector3d pickRandomPoint();
        bool nonEmptyReachableWorkspace(EndEffector pk, Configuration q);
        Eigen::Vector3d pickPointInReachableWorkspace(EndEffector pk, Configuration q, Eigen::Vector3d rRand, int &index);
        Eigen::Vector3d pickPointInGrowingReachableWorkspace(EndEffector pk, Configuration q, Eigen::Vector3d rRand, int &index);
        EndEffector pickRandomEndEffector();
        std::shared_ptr<Contact> pickRandomContactFromGoalStance();
        int findNearestVertexIndex(EndEffector pk, Eigen::Vector3d r);
        bool computeCentroidalStatics(std::vector<EndEffector> activeEEsDes, Eigen::Vector3d rCoMdes, Eigen::MatrixXd rCdes, Eigen::MatrixXd nCdes, Eigen::Vector3d &rCoM, Eigen::MatrixXd &rC, Eigen::MatrixXd &FC);
        Eigen::Vector3d getNormalAtPoint(Eigen::Vector3d p);
        Eigen::Vector3d getNormalAtPointByIndex(int index);
        Eigen::Vector3d computeCoM(Configuration q);
        Eigen::Affine3d computeForwardKinematics(Configuration q, EndEffector ee);

        bool similarityCheck(Stance sigmaNew);
        bool distanceCheck(Stance sigmaNew);

        void retrieveContactForces(Configuration q, Stance &sigma);
        void retrieveContactPoses(Configuration q, Stance &sigma);

        bool computeIKandCS(Stance sigmaSmall, Stance sigmaLarge, Configuration qNear, Configuration &qNew, bool adding);


    public:

        Planner(Configuration _qInit,
                std::vector<EndEffector> _activeEEsInit, Configuration _qGoal,
                std::vector<EndEffector> _activeEEsGoal, Eigen::MatrixXd _pointCloud,
                Eigen::MatrixXd _pointNormals, std::vector<EndEffector> _allowedEEs,
                XBot::ModelInterface::Ptr _planner_model,
                XBot::Cartesian::Planning::NSPG::Ptr _NSPG,
                XBot::Cartesian::Planning::ValidityCheckContext _vc_context,
                ros::NodeHandle& nh);
        std::unique_ptr<XBot::Cartesian::Planning::CentroidalStatics> _cs;
        ~Planner() = default;
        void run();
        bool retrieveSolution(std::vector<Stance> &sigmaList, std::vector<Configuration> &qList);
        int getTreeSize();

        void checkSolutionCS(std::vector<Stance> sigmaList, std::vector<Configuration> qList);

        void clearTree();

};

#endif

