#ifndef __CARTESIO_PLANNING_POSITION_IK_SOLVER_H__
#define __CARTESIO_PLANNING_POSITION_IK_SOLVER_H__

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include "planner/multi_contact/utils.hpp" 


namespace XBot { namespace Cartesian { namespace Planning {

/**
 * @brief The PositionCartesianSolver class implements a position-level IK
 * solver on top of a CartesianInterface object (provided by the user)
 * which solves differential IK. This is done by iterating the local solution
 * and integrating the model state until suitable termination conditions are
 * satisfied.
 */
class PositionCartesianSolver
{

public:

    typedef std::shared_ptr<PositionCartesianSolver> Ptr;

    static const double DEFAULT_ERR_TOL;
    static const int DEFAULT_MAX_ITER;

    PositionCartesianSolver(CartesianInterfaceImpl::Ptr ci);
    bool solve();
    void getError(Eigen::VectorXd& error) const;
    void getJacobian(Eigen::MatrixXd& J) const;
    int getSize() const;
    ModelInterface::Ptr getModel() const;
    double getErrorThreshold() const;
    void setRosServerClass(RosServerClass::Ptr ros_server);
    void setIterCallback(std::function<void ()> f);
    void reset();
    void setErrorTolerance(const double error_tolerance);
    void setMaxIterations(const int max_iter);
    CartesianInterfaceImpl::Ptr getCI() {return _ci;}

    
private:
    
    std::vector<std::string> _all_tasks;    
    std::vector<std::string> _all_subtasks; 
    CartesianInterfaceImpl::Ptr _ci;
    ModelInterface::Ptr _model;
    
    int _max_iter;
    double _err_tol;
    int _n;

    RosServerClass::Ptr _ros_server;
    std::function<void(void)> _iter_callback;    

};


} } }


#endif
