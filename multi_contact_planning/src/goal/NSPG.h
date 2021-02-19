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

#include <planner/multi_contact/Stance.hpp>
#include <multi_contact_planning/SetContactFrames.h>
#include "validity_checker/stability/centroidal_statics.h"

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
        
        double generateRandom();

        std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;
        
        bool sample(double timeout, Stance sigmaSmall, Stance sigmaLarge, Eigen::Vector3d dir); 
        bool balanceCheck(Stance sigma);
        XBot::JointNameMap generateRandomVelocities(bool collisionCheckRes, bool balanceCheckRes, std::vector<XBot::ModelChain> colliding_chains);
        XBot::JointNameMap generateRandomVelocities(bool collisionCheckRes, bool balanceCheckRes, std::vector<XBot::ModelChain> colliding_chains, Eigen::Vector3d dir);
        
    private:        
        
        PositionCartesianSolver::Ptr _ik_solver;
        
        Planning::ValidityCheckContext _vc_context;
        
        std::unique_ptr<XBot::Cartesian::Planning::CentroidalStatics> _cs;
        

    };
}}}
