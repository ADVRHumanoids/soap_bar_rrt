#include "NSPG.h"   

using namespace XBot::Cartesian::Planning;

static std::default_random_engine randGenerator;
static std::uniform_real_distribution<double> randDistribution(-1.0, 1.0);

NSPG::NSPG ( PositionCartesianSolver::Ptr ik_solver, ValidityCheckContext vc_context ):
    _ik_solver(ik_solver),
    _vc_context(vc_context)
    {
        
        auto a = std::chrono::system_clock::now();
        time_t b = std::chrono::system_clock::to_time_t(a);
        randGenerator.seed(b);
        
        _rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_ik_solver->getModel());         
        
        std::vector<std::string> links = {"r_sole", "l_sole", "TCP_R", "TCP_L", "l_ball_tip_d", "r_ball_tip_d"};
        //_cs = std::unique_ptr<XBot::Cartesian::Planning::CentroidalStatics>(new XBot::Cartesian::Planning::CentroidalStatics(_ik_solver->getModel(), links, MU_FRICTION*sqrt(2), true, Eigen::Vector2d(-0.1, 0.1), Eigen::Vector2d(-0.05, 0.05)));
        
        Eigen::Vector2d CoP_xlim;
        Eigen::Vector2d CoP_ylim;
        if(SCENARIO == 3){ 
            CoP_xlim << -0.1, 0.1;
            CoP_ylim << -0.05, 0.05;
        }
        else{
            CoP_xlim << -0.04, 0.04;
            CoP_ylim << -0.04, 0.04;
        }
        _cs = std::unique_ptr<XBot::Cartesian::Planning::CentroidalStatics>(new XBot::Cartesian::Planning::CentroidalStatics(_ik_solver->getModel(), links, MU_FRICTION*sqrt(2), true, CoP_xlim, CoP_ylim));

    }
    
void NSPG::setIKSolver ( PositionCartesianSolver::Ptr new_ik_solver )
{
    _ik_solver = new_ik_solver;
}

PositionCartesianSolver::Ptr NSPG::getIKSolver () const 
{
    return _ik_solver;
}


double NSPG::generateRandom() 
{    
    return randDistribution(randGenerator); 
}


XBot::JointNameMap NSPG::generateRandomVelocities(std::vector<XBot::ModelChain> colliding_chains){
    XBot::JointNameMap random_map, chain_map, velocityLim_map;
    Eigen::VectorXd velocity_lim;
    
    _ik_solver->getModel()->getVelocityLimits(velocity_lim);
    _ik_solver->getCI()->getReferencePosture(velocityLim_map);
    _ik_solver->getModel()->eigenToMap(velocity_lim, velocityLim_map);
    
    if(SCENARIO != 3) random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*GAIN_VEL_FB_X));
    else random_map.insert(std::make_pair("VIRTUALJOINT_1", -fabs(generateRandom()*GAIN_VEL_FB_X)));
    random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*GAIN_VEL_FB_Y));
    random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*GAIN_VEL_FB_Z));              
    
    if(RAND_VEL_CHAINS){
        for (auto i:colliding_chains)
        {
            i.getJointPosition(chain_map);
                
            for (auto j : chain_map)
            {
                j.second = generateRandom() * velocityLim_map[j.first];
                random_map.insert(std::make_pair(j.first, j.second));
            }
        }
    }
    
    return random_map;
}

bool NSPG::sample(double timeout, Stance sigmaSmall, Stance sigmaLarge) 
{
    // BE SURE THAT _ik_solver AND _vc_context HAS THE SAME MODEL
    Eigen::VectorXd x, dqlimits;
    XBot::JointNameMap chain_map, joint_map, velocity_map, random_map;
    
    // Start initializing joint_map
    _ik_solver->getModel()->getJointPosition(joint_map);
    _ik_solver->getModel()->getVelocityLimits(dqlimits);
    _ik_solver->getModel()->getJointPosition(x); // now x is the qNominal
    
    // Fill velocity_map with the velocity limits
    _ik_solver->getModel()->eigenToMap(x, velocity_map);
    _ik_solver->getModel()->eigenToMap(dqlimits, velocity_map);
    
    /* mio
    float T = 0.0;
    double dt = 0.01;
    int iter = 0;
    int iterMax = 100;
    */
    
    // luca
    float T = 0.0;
    double dt = DT;
    int iter = 0;
    int iterMax = ITER_MAX;
    
    initializeBalanceCheck(sigmaSmall);
        
    bool ik_solved = true;  
    bool collisionCheckRes = _vc_context.vc_aggregate.check("collisions"); 
    bool balanceCheckRes = balanceCheck(sigmaSmall);  
    
    while(!collisionCheckRes || !balanceCheckRes || !ik_solved)
    {
        auto tic = std::chrono::high_resolution_clock::now();
        
        // Acquire colliding chains
        auto colliding_chains = _vc_context.planning_scene->getCollidingChains();
        
        // Generate a random velocity vector for colliding chains' joints only every n iterations
        if (iter % iterMax == 0)
        {
            _ik_solver->getModel()->eigenToMap(x, joint_map);
            random_map = generateRandomVelocities(colliding_chains);
            
            // reset robot in qNominal
            //_ik_solver->getModel()->setJointPosition(x);
        }
        
        // Update joint_map with integrated random velocities       
        for (auto i : random_map)
            joint_map[i.first] += i.second * dt;
        
        iter ++;
     
        _ik_solver->getCI()->setReferencePosture(joint_map);
        
        ik_solved = _ik_solver->solve();
        if(ik_solved) collisionCheckRes = _vc_context.vc_aggregate.check("collisions"); 
        if(ik_solved && collisionCheckRes) balanceCheckRes = balanceCheck(sigmaSmall);        
        
        _rspub->publishTransforms(ros::Time::now(), "/planner");
                        
        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> fsec = toc-tic;
        T += fsec.count();
        
        if(T >= timeout)
        {
            std::cout << "[NSPG]: timeout" <<std::endl;
            return false;
        }
        
    }
    
    std::cout << "[NSPG]: done!" << std::endl;
    return true;
}

void NSPG::initializeBalanceCheck(Stance sigma){  
    
    std::vector<std::string> active_links;
    std::vector<Eigen::Affine3d> ref_tasks;
    std::vector<Eigen::Vector3d> normals;
    for(int i = 0; i < sigma.getSize(); i++)
    {
        EndEffector ee = sigma.getContact(i)->getEndEffectorName();
        std::string contact_link = getTaskStringName(ee);
        Eigen::Vector3d nC_i = sigma.getContact(i)->getNormal();
        active_links.push_back(contact_link);
        normals.push_back(nC_i);
    }
    
    _cs->setContactLinks(active_links);
    
    //(sigma.getSize() == 2) _cs->setOptimizeTorque(true);
    //else _cs->setOptimizeTorque(false);
    
    _cs->init(false);  
    
    for (int i = 0; i < sigma.getContacts().size(); i ++)
    {
        Eigen::Matrix3d rot = generateRotationFrictionCone(normals[i]);
        _cs->setContactRotationMatrix(active_links[i], rot);
    }
    
    // set (possibly different) friction coefficients for the hands
    XBot::Cartesian::CartesianInterface::Ptr ci_CS = _cs->getCI();
    auto tasks_CS = ci_CS->getTaskList();
    for(int i = 0; i < tasks_CS.size(); i++){
        XBot::Cartesian::TaskDescription::Ptr task_fc = nullptr;
        
        if(tasks_CS[i].compare("TCP_R_fc") == 0) task_fc = ci_CS->getTask("TCP_R_fc");
        if(tasks_CS[i].compare("TCP_L_fc") == 0) task_fc = ci_CS->getTask("TCP_L_fc");
        if(tasks_CS[i].compare("l_ball_tip_d_fc") == 0) task_fc = ci_CS->getTask("l_ball_tip_d_fc");
        if(tasks_CS[i].compare("r_ball_tip_d_fc") == 0) task_fc = ci_CS->getTask("r_ball_tip_d_fc");
                    
        if(task_fc != nullptr){
            auto task_fc_0 = std::dynamic_pointer_cast<XBot::Cartesian::acceleration::FrictionCone>(task_fc);
            task_fc_0->setFrictionCoeff(MU_FRICTION_HANDS*sqrt(2)); 
        }
    }
    
}

bool NSPG::balanceCheck(Stance sigma){
    if (_cs->checkStability(CS_THRES)) return true; 
    return false;
}
