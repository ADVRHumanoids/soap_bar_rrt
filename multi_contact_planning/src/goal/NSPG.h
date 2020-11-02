#include <ik/position_ik_solver.h>
#include <chrono>
#include <functional>
#include <stdio.h>
#include <stdlib.h>
#include <thread>

#include <XBotInterface/ModelInterface.h>
#include "validity_checker/validity_checker_context.h"
#include "utils/robot_viz.h"
#include <cartesian_interface/utils/RobotStatePublisher.h>

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
        
    private:        
        
        PositionCartesianSolver::Ptr _ik_solver;
        
        Planning::ValidityCheckContext _vc_context;
    };
}}}