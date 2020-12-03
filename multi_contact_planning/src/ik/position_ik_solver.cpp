#include "position_ik_solver.h"

using namespace XBot;
using namespace XBot::Cartesian::Planning;

const double PositionCartesianSolver::DEFAULT_ERR_TOL = 1e-4;
const int PositionCartesianSolver::DEFAULT_MAX_ITER = 1000;//60;

PositionCartesianSolver::PositionCartesianSolver(CartesianInterfaceImpl::Ptr ci):
    _n_task(0),
    _ci(ci),
    _model(ci->getModel()),
    _max_iter(DEFAULT_MAX_ITER),
    _err_tol(DEFAULT_ERR_TOL),
    _iter_callback([](){})
{
    for(auto t : ci->getIkProblem().getTask(0))
    {
        if(auto cart = std::dynamic_pointer_cast<Cartesian::CartesianTask>(t))
        {
            //////////////
            //std::vector<int> ind;
            //ind.push_back(0);
            //ind.push_back(1);
            //ind.push_back(2);
            //cart->setIndices(ind); // do this only for hands
            //Eigen::MatrixXd w = Eigen::MatrixXd::Identity(6,6);
            //w(3,3) = 0.0;
            //w(4,4) = 0.0;
            //w(5,5) = 0.0;
            //t->setWeight(w); // do this only for hands
            /////////////   

            std::string distal_link=cart->getDistalLink();

            auto tdata = std::make_shared<CartesianTaskData>(distal_link,
                                                             cart->getBaseLink(),
                                                             cart->getIndices());


            std::cout << "TASKS ------" << std::endl;    
            std::cout << "WEIGHTS" << std::endl;
            std::cout << t->getWeight() << std::endl;
            //std::cout << cart->getLambda() << std::endl;    
            std::cout << "distal_link" << std::endl;
            std::cout << distal_link << std::endl;
            std::cout << "cart->getBaseLink()" << std::endl;    
            std::cout << cart->getBaseLink() << std::endl;           
            std::cout << "cart->getIndices()" << std::endl;
            for(int i=0; i < cart->getIndices().size(); i++){
                std::cout << cart->getIndices().at(i) << std::endl;    
            }        
            
            _n_task += tdata->size;

            _task_map[distal_link] = tdata;

            printf("[PositionCartesianSolver] adding cartesian task '%s' to '%s', size is %d \n",
                   cart->getBaseLink().c_str(),
                   distal_link,
                   tdata->size);
        }
    }

}

void PositionCartesianSolver::setDesiredPose(std::string distal_frame,
                                             const Eigen::Affine3d & pose)
{
    if(!_ci->setPoseReference(distal_frame, pose))
    {
        throw std::runtime_error("Unable to set desired pose for task '" + distal_frame + "'");
    }
}


bool PositionCartesianSolver::solve()
{

    // allocate variables
    Eigen::VectorXd q, dq, error;

    // main solver loop
    bool tol_satisfied = false;
    int iter = 0;
    const double step_size = 1.0;

    while(!tol_satisfied && iter < _max_iter)
    {

        double dt = 1.0; // dummy dt
        
        if(!_ci->update(0.0, dt))
        {
            return false;
        }

        _model->getJointPosition(q);
        _model->getJointVelocity(dq);
        q += step_size*dq;
        _model->setJointPosition(q);
        _model->update();

        
        if(_ros_server)
            _ros_server->run();

        
        getError(error);
        tol_satisfied = error.cwiseAbs().maxCoeff() < _err_tol;

        //std::cout << "error = " << error.transpose() << std::endl;


        iter++;


        _iter_callback();


    }

//    printf("Error at iter #%d is %f \n", iter, error.norm());
    return tol_satisfied;

}

void PositionCartesianSolver::getError(Eigen::VectorXd& error) const
{
    error.setZero(_n_task);

    int error_idx = 0;
    for(auto pair : _task_map)
    {
        auto t = pair.second;
        t->update(_ci, _model); // tbd: optimize
        
        error.segment(error_idx, t->size) = t->error;
        error_idx += t->size;
    }

}

void PositionCartesianSolver::getJacobian(Eigen::MatrixXd & J) const
{
    J.setZero(_n_task, _model->getJointNum());

    int jac_idx = 0;
    for(auto pair : _task_map)
    {
        auto t = pair.second;
        t->update(_ci, _model); // tbd: optimize

        J.middleRows(jac_idx, t->size) = t->J;
        jac_idx += t->size;
    }
}

int PositionCartesianSolver::getSize() const
{
    return _n_task;
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
    return _err_tol * _n_task;
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

PositionCartesianSolver::TaskData::TaskData(int a_size):
    size(a_size)
{
    error.setZero(size);
    J.setZero(size, 0);
}

PositionCartesianSolver::TaskData::~TaskData()
{

}

PositionCartesianSolver::CartesianTaskData::CartesianTaskData(std::string a_distal_link,
                                                              std::string a_base_link,
                                                              std::vector<int> a_indices):
    TaskData(a_indices.size()),
    distal_link(a_distal_link),
    base_link(a_base_link),
    indices(a_indices)
{

}

void PositionCartesianSolver::CartesianTaskData::update(XBot::Cartesian::CartesianInterfaceImpl::Ptr ci,
                                                        XBot::ModelInterface::Ptr model)
{
    J.setZero(size, model->getJointNum());
    error.setZero(size);

    /* If task was disabled, error and jacobian are zero */
    auto active = ci->getActivationState(distal_link);
    if(active == ActivationState::Disabled)
    {
        return;
    }


    /* Error computation */
    Eigen::Affine3d T, Tdes;
    ci->getCurrentPose(distal_link, T);
    ci->getPoseReference(distal_link, Tdes);

    Eigen::Vector3d pos_error = T.translation() - Tdes.translation();
    Eigen::Vector3d rot_error;
    Eigen::Matrix3d L;
    compute_orientation_error(Tdes.linear(),
                              T.linear(),
                              rot_error,
                              L);

    Eigen::Vector6d error6d;
    error6d << pos_error, rot_error;

    for(int i = 0; i < size; i++)
    {
        error[i] = error6d[indices[i]];
    }

    //std::cout << "error size = " << size << std::endl;

    /* Jacobian computation */
    Eigen::MatrixXd Ji;

    if(distal_link == "com")
    {
        model->getCOMJacobian(Ji);
    }
    else if(base_link == "world")
    {
        model->getJacobian(distal_link, Ji);
    }
    else
    {
        model->getJacobian(distal_link, base_link, Ji);
    }

    Ji.bottomRows<3>() = L * Ji.bottomRows<3>();

    for(int i = 0; i < size; i++)
    {
        J.row(i) = Ji.row(indices[i]);
    }

}

void PositionCartesianSolver::CartesianTaskData::compute_orientation_error(const Eigen::Matrix3d & Rd,
                                                                           const Eigen::Matrix3d & Re,
                                                                           Eigen::Vector3d & e_o,
                                                                           Eigen::Matrix3d & L)
{
    auto S = XBot::Utils::skewSymmetricMatrix;

    e_o = 0.5 * (Re.col(0).cross(Rd.col(0)) + Re.col(1).cross(Rd.col(1)) + Re.col(2).cross(Rd.col(2)));
    L = 0.5 * ( S(Rd.col(0))*S(Re.col(0)) + S(Rd.col(1))*S(Re.col(1)) + S(Rd.col(2))*S(Re.col(2)));


    //Eigen::Matrix3d rot_error = Rd.inverse() * Re;
    //Eigen::Vector3d e_o_euler = rot_error.eulerAngles(2, 1, 0);
    //e_o = e_o_euler;

    //e_o = 0.5 * ( Re.col(2).cross(Rd.col(2)));
      
}

