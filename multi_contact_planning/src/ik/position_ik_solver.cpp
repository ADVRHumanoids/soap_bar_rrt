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
    _all_tasks = {"LeftFootNoZ", "RightFootNoZ", "LeftHandCNoZ", "RightHandCNoZ", "LeftHandDNoZ", "RightHandDNoZ", "LeftFootZ", "RightFootZ", "LeftHandCZ", "RightHandCZ", "LeftHandDZ", "RightHandDZ"};
//      _all_tasks = {"LeftFoot", "RightFoot", "LeftHandC", "RightHandC", "LeftHandD", "RightHandD"};
//     _all_tasks = {"l_sole", "r_sole", "TCP_L", "TCP_R", "l_ball_tip_d", "r_ball_tip_d"};
    

}


bool PositionCartesianSolver::solve()
{
    _n = 0;
    for(int i = 0; i < _all_tasks.size(); i++){
        if(_ci->getActivationState(_all_tasks[i]) == ActivationState::Enabled){
            std::vector<int> indices = _ci->getTask(_all_tasks[i])->getIndices();
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
    for(int i = 0; i < _all_tasks.size(); i++){
        if(_ci->getActivationState(_all_tasks[i]) == ActivationState::Enabled){
            std::vector<int> indices = _ci->getTask(_all_tasks[i])->getIndices();
            
            // compute i-th error
            Eigen::Affine3d T_c, T_d;
            std::string main_task = getMainTaskStringName(_all_tasks[i]);
            _ci->getCurrentPose(main_task, T_c);
            _ci->getPoseReference(main_task, T_d);
            
            //_ci->getCurrentPose(_all_tasks[i], T_c);
            //_ci->getPoseReference(_all_tasks[i], T_d);
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
{
    J.setZero(_n, _model->getJointNum());

    int jac_idx = 0;
    for(int i = 0; i < _all_tasks.size(); i++){
        if(_ci->getActivationState(_all_tasks[i]) == ActivationState::Enabled){
            std::vector<int> indices = _ci->getTask(_all_tasks[i])->getIndices();
    
             Eigen::MatrixXd J6_i;
             std::string distal_link =_all_tasks[i];
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

// PositionCartesianSolver::TaskData::TaskData(int a_size):
//     size(a_size)
// {
//     error.setZero(size);
//     J.setZero(size, 0);
// }
// 
// PositionCartesianSolver::TaskData::~TaskData()
// {
// 
// }
// 
// PositionCartesianSolver::CartesianTaskData::CartesianTaskData(std::string a_distal_link,
//                                                               std::string a_base_link,
//                                                               std::vector<int> a_indices):
//     TaskData(a_indices.size()),
//     distal_link(a_distal_link),
//     base_link(a_base_link),
//     indices(a_indices)
// {
// 
// }
// 
// void PositionCartesianSolver::CartesianTaskData::update(XBot::Cartesian::CartesianInterfaceImpl::Ptr ci, XBot::ModelInterface::Ptr model)
// {
//     std::vector<int> indices_c = ci->getTask(distal_link)->getIndices();
//     int size_c = indices_c.size();
//     
//     std::cout << "error size = " << size_c << std::endl;
//     
//     J.setZero(size_c, model->getJointNum());
//     error.setZero(size_c);
// 
//     /* If task was disabled, error and jacobian are zero */
//     auto active = ci->getActivationState(distal_link);
//     if(active == ActivationState::Disabled)
//     {
//         return;
//     }
// 
// 
//     /* Error computation */
//     Eigen::Affine3d T, Tdes;
//     ci->getCurrentPose(distal_link, T);
//     ci->getPoseReference(distal_link, Tdes);
// 
//     Eigen::Vector3d pos_error = T.translation() - Tdes.translation();
//     Eigen::Vector3d rot_error;
//     Eigen::Matrix3d L;
//     compute_orientation_error(Tdes.linear(),
//                               T.linear(),
//                               rot_error,
//                               L);
// 
//     Eigen::Vector6d error6d;
//     error6d << pos_error, rot_error;
// 
//     for(int i = 0; i < size_c; i++)
//     {
//         error[i] = error6d[indices_c[i]];
//     }
// 
//     /* Jacobian computation */
//     Eigen::MatrixXd Ji;
// 
//     if(distal_link == "com")
//     {
//         model->getCOMJacobian(Ji);
//     }
//     else if(base_link == "world")
//     {
//         model->getJacobian(distal_link, Ji);
//     }
//     else
//     {
//         model->getJacobian(distal_link, base_link, Ji);
//     }
// 
//     //Ji.bottomRows<3>() = L * Ji.bottomRows<3>(); //FIXME why this?
// 
//     for(int i = 0; i < size_c; i++)
//     {
//         J.row(i) = Ji.row(indices_c[i]);
//     }
// 
// }
// 
// void PositionCartesianSolver::CartesianTaskData::updateOld(XBot::Cartesian::CartesianInterfaceImpl::Ptr ci,
//                                                         XBot::ModelInterface::Ptr model)
// {
//     J.setZero(size, model->getJointNum());
//     error.setZero(size);
// 
//     /* If task was disabled, error and jacobian are zero */
//     auto active = ci->getActivationState(distal_link);
//     if(active == ActivationState::Disabled)
//     {
//         return;
//     }
// 
// 
//     /* Error computation */
//     Eigen::Affine3d T, Tdes;
//     ci->getCurrentPose(distal_link, T);
//     ci->getPoseReference(distal_link, Tdes);
// 
//     Eigen::Vector3d pos_error = T.translation() - Tdes.translation();
//     Eigen::Vector3d rot_error;
//     Eigen::Matrix3d L;
//     compute_orientation_error(Tdes.linear(),
//                               T.linear(),
//                               rot_error,
//                               L);
// 
//     Eigen::Vector6d error6d;
//     error6d << pos_error, rot_error;
// 
//     for(int i = 0; i < size; i++)
//     {
//         error[i] = error6d[indices[i]];
//     }
// 
//     //std::cout << "error size = " << size << std::endl;
// 
//     /* Jacobian computation */
//     Eigen::MatrixXd Ji;
// 
//     if(distal_link == "com")
//     {
//         model->getCOMJacobian(Ji);
//     }
//     else if(base_link == "world")
//     {
//         model->getJacobian(distal_link, Ji);
//     }
//     else
//     {
//         model->getJacobian(distal_link, base_link, Ji);
//     }
// 
//     //Ji.bottomRows<3>() = L * Ji.bottomRows<3>(); //FIXME why this?
// 
//     for(int i = 0; i < size; i++)
//     {
//         J.row(i) = Ji.row(indices[i]);
//     }
// 
// }
// 
// // float angleSignedDistance(float a, float b){ //FIXME possibly put this in utils.hpp
// // 	float d = fabs(a - b);
// // 	while(d > 2.0*M_PI) d = d - 2.0*M_PI; 
// // 
// // 	float r = 0.0;
// // 	if(d > M_PI) r = 2.0*M_PI - d;
// // 	else r = d;
// // 	float sign = 0.0;
// // 	if( (a-b>=0.0 && a-b<=M_PI) || (a-b<=-M_PI && a-b>=-2.0*M_PI) ) sign = +1.0;
// // 	else sign = -1.0;
// // 
// // 	r = sign * r;
// // 	return r; 
// // }
// 
// void PositionCartesianSolver::CartesianTaskData::compute_orientation_error(const Eigen::Matrix3d & Rd,
//                                                                            const Eigen::Matrix3d & Re,
//                                                                            Eigen::Vector3d & e_o,
//                                                                            Eigen::Matrix3d & L)
// {
//     auto S = XBot::Utils::skewSymmetricMatrix;
// 
//     e_o = 0.5 * (Re.col(0).cross(Rd.col(0)) + Re.col(1).cross(Rd.col(1)) + Re.col(2).cross(Rd.col(2)));
//     L = 0.5 * ( S(Rd.col(0))*S(Re.col(0)) + S(Rd.col(1))*S(Re.col(1)) + S(Rd.col(2))*S(Re.col(2)));
// 
// 
//     //Eigen::Matrix3d rot_error = Rd.inverse() * Re;
//     //Eigen::Vector3d e_o_euler = rot_error.eulerAngles(2, 1, 0);
//     //e_o = e_o_euler;
// 
//     //e_o = 0.5 * ( Re.col(2).cross(Rd.col(2)));
//     
//     //e_o = Rd.eulerAngles(0, 1, 2) - Re.eulerAngles(0, 1, 2);
// //     Eigen::Vector3d e_d = Rd.eulerAngles(0, 1, 2);
// //     Eigen::Vector3d e_c = Re.eulerAngles(0, 1, 2);
// //     e_o(0) = angleSignedDistance(e_d(0), e_c(0));
// //     e_o(1) = angleSignedDistance(e_d(1), e_c(1));
// //     e_o(2) = angleSignedDistance(e_d(2), e_c(2));
// //     
// //      L = Eigen::MatrixXd::Identity(3,3);
//     
// //     Eigen::Vector3d e_d = Rd.eulerAngles(0, 1, 2);
// //     Eigen::Vector3d e_c = Re.eulerAngles(0, 1, 2);
// //     e_o(0) = angleSignedDistance(e_c(0), e_d(0));
// //     e_o(1) = angleSignedDistance(e_c(1), e_d(1));
// //     e_o(2) = angleSignedDistance(e_c(2), e_d(2));
// }


