#include <ik/position_ik_solver.h>
#include <chrono>
#include <functional>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <random>

#include <XBotInterface/ModelInterface.h>
#include "validity_checker/validity_checker_context.h"
#include "utils/robot_viz.h"
#include <cartesian_interface/utils/RobotStatePublisher.h>

#include <planner/multi_contact/enum.h>
#include <planner/multi_contact/constant_values.hpp>
#include <CentroidalPlanner/CentroidalPlanner.h>
#include <CentroidalPlanner/CoMPlanner.h>

namespace XBot { namespace Cartesian { namespace Planning {
    
    class NSPG {
           
    public: 
        
        typedef std::shared_ptr<NSPG> Ptr;

        NSPG(PositionCartesianSolver::Ptr ik_solver,
                     Planning::ValidityCheckContext vc_context);
               
        void setIKSolver(PositionCartesianSolver::Ptr new_ik_solver);
        
        PositionCartesianSolver::Ptr getIKSolver() const;
        
        XBot::JointNameMap generateRandomVelocities(std::vector<XBot::ModelChain> colliding_chains);
        
        bool sample(double timeout);
        bool sample(double timeout, std::vector<EndEffector> activeEEsDes, Eigen::MatrixXd rCDes, Eigen::MatrixXd nCDes);
        bool sampleOLD(double timeout, std::vector<EndEffector> activeEEsDes, Eigen::MatrixXd rCDes, Eigen::MatrixXd nCDes);
        
        double generateRandom();

        std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;
        
    private:        
        
        PositionCartesianSolver::Ptr _ik_solver;
        
        Planning::ValidityCheckContext _vc_context;
        
        bool computeCentroidalStatics(std::vector<EndEffector> activeEEsRef, Eigen::Vector3d rCoMRef, Eigen::MatrixXd rCRef, Eigen::MatrixXd nCRef, Eigen::MatrixXd &FC);
        
        XBot::JointNameMap generateRandomVelocitiesCollision(std::vector<XBot::ModelChain> colliding_chains); 
        XBot::JointNameMap generateRandomVelocitiesBalance(); 

        

    };
}}}
