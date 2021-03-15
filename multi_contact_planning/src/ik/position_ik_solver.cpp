#include "position_ik_solver.h"

using namespace XBot;
using namespace XBot::Cartesian::Planning;

const double PositionCartesianSolver::DEFAULT_ERR_TOL = 1e-4;
const int PositionCartesianSolver::DEFAULT_MAX_ITER = 1000;//60;

PositionCartesianSolver::PositionCartesianSolver(CartesianInterfaceImpl::Ptr ci):
    _n(0),
    _ci(ci),
    _model(ci->getModel()),
    _max_iter(DEFAULT_MAX_ITER),
    _err_tol(DEFAULT_ERR_TOL),
    _iter_callback([](){})
{
  
    _all_tasks = {"l_sole", "r_sole", "TCP_L", "TCP_R", "l_ball_tip_d", "r_ball_tip_d"};
    
    for(int i = 0; i < _all_tasks.size(); i++){
        std::vector<std::string> subtasks = getSubtasksStringName(_all_tasks[i]);
        _all_subtasks.push_back(subtasks[0]);
        _all_subtasks.push_back(subtasks[1]);
    }
    
}


bool PositionCartesianSolver::solve()
{
    _n = 0;
    for(int i = 0; i < _all_subtasks.size(); i++){
        if(_ci->getActivationState(_all_subtasks[i]) == ActivationState::Enabled){
            std::vector<int> indices = _ci->getTask(_all_subtasks[i])->getIndices();
            _n += indices.size();
        }
    }
    
    Eigen::VectorXd q, dq, error;
    bool tol_satisfied = false;
    int iter = 0;
    double dt = 1.0; // dummy dt

    while(!tol_satisfied && iter < _max_iter)
    {
        if(!_ci->update(0.0, dt)) return false;
        
        _model->getJointPosition(q);
        _model->getJointVelocity(dq);
        q += dt*dq;
        _model->setJointPosition(q);
        _model->update();

        if(_ros_server) _ros_server->run();

        getError(error);
        tol_satisfied = error.cwiseAbs().maxCoeff() < _err_tol;

        iter++;
        //_iter_callback(); //TODO why this?
    }

    return tol_satisfied;
}

void PositionCartesianSolver::getError(Eigen::VectorXd& error) const
{
    error.setZero(_n);
    
    int error_idx = 0;
    for(int i = 0; i < _all_subtasks.size(); i++){
        if(_ci->getActivationState(_all_subtasks[i]) == ActivationState::Enabled){
            std::vector<int> indices = _ci->getTask(_all_subtasks[i])->getIndices();
            
            // compute i-th error
            Eigen::Affine3d T_c, T_d;
            std::string main_task = getMainTaskStringName(_all_subtasks[i]);
            _ci->getCurrentPose(main_task, T_c);
            _ci->getPoseReference(main_task, T_d);
            
            Eigen::Matrix3d R_c, R_d;
            R_c = T_c.linear();
            R_d = T_d.linear();                              
            Eigen::Vector3d pos_error = T_c.translation() - T_d.translation();
            Eigen::Vector3d rot_error = 0.5 * (R_c.col(0).cross(R_d.col(0)) + R_c.col(1).cross(R_d.col(1)) + R_c.col(2).cross(R_d.col(2)));
    
            Eigen::VectorXd error6_i(6);
            error6_i << pos_error, rot_error;
            
            Eigen::VectorXd error_i(indices.size());
            for(int j = 0; j < indices.size(); j++) error_i(j) = error6_i(indices[j]);
                
            // store error
            error.segment(error_idx, indices.size()) = error_i;
            error_idx += indices.size();
            
        }
    }
}

void PositionCartesianSolver::getJacobian(Eigen::MatrixXd & J) const
{ // TODO test this, it should work only in case the base frame is the world
    J.setZero(_n, _model->getJointNum());

    int jac_idx = 0;
    for(int i = 0; i < _all_subtasks.size(); i++){
        if(_ci->getActivationState(_all_subtasks[i]) == ActivationState::Enabled){
            std::vector<int> indices = _ci->getTask(_all_subtasks[i])->getIndices();
    
             Eigen::MatrixXd J6_i;
             std::string distal_link =_all_subtasks[i];
             _model->getJacobian(distal_link, J6_i);
             
            Eigen::MatrixXd J_i(indices.size(), _model->getJointNum());
            for(int j = 0; j < indices.size(); j++) J6_i.row(j) = J_i.row(indices[j]);
            
            // store jacobian
            J.middleRows(jac_idx, indices.size()) = J_i;
            jac_idx += indices.size();
           
        }
    }
}

int PositionCartesianSolver::getSize() const
{
    return _n;
}

void PositionCartesianSolver::setIterCallback(std::function<void ()> f)
{
    _iter_callback = f;
}

XBot::ModelInterface::Ptr PositionCartesianSolver::getModel() const  
{
    return _model;
}

double PositionCartesianSolver::getErrorThreshold() const
{
    return _err_tol * _n;
}

void PositionCartesianSolver::setRosServerClass(XBot::Cartesian::RosServerClass::Ptr ros_server)
{
    _ros_server = ros_server;
}

void PositionCartesianSolver::reset()
{
    _ci->reset(0.0);
}

void PositionCartesianSolver::setErrorTolerance(const double error_tolerance)
{
    _err_tol = error_tolerance;
}

void PositionCartesianSolver::setMaxIterations(const int max_iter)
{
    _max_iter = max_iter;
}
