#include "NSPG.h"   

using namespace XBot::Cartesian::Planning;

// PF
static std::default_random_engine randGenerator;
static std::uniform_real_distribution<double> randDistribution(-1.0, 1.0);

//std::string env(getenv("ROBOTOLOGY_ROOT"));
static std::ofstream foutLogNSPG("/home/paolo/catkin_ws/external/src/soap_bar_rrt/multi_contact_planning/PlanningData/logNSPG.txt", std::ofstream::trunc);

NSPG::NSPG ( PositionCartesianSolver::Ptr ik_solver, ValidityCheckContext vc_context ):
    _ik_solver(ik_solver),
    _vc_context(vc_context)
    {
        
        // PF
        auto a = std::chrono::system_clock::now();
        time_t b = std::chrono::system_clock::to_time_t(a);
        randGenerator.seed(b);
        
        _rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_ik_solver->getModel()); 
        
        
        /////////////
        std::vector<std::string> links = {"r_sole", "l_sole", "TCP_R", "TCP_L"};
        
        _cs = std::unique_ptr<XBot::Cartesian::Planning::CentroidalStatics>(new XBot::Cartesian::Planning::CentroidalStatics(_ik_solver->getModel(), links, 0.5*sqrt(2)));
    }
    
void NSPG::setIKSolver ( PositionCartesianSolver::Ptr new_ik_solver )
{
    _ik_solver = new_ik_solver;
}

PositionCartesianSolver::Ptr NSPG::getIKSolver () const 
{
    return _ik_solver;
}

bool NSPG::sample ( double timeout ) 
{
    // BE SURE THAT _ik_solver AND _vc_context HAS THE SAME MODEL
    Eigen::VectorXd x, dqlimits;
    XBot::JointNameMap chain_map, joint_map, velocity_map, random_map;
    
    // Start initializing joint_map
    _ik_solver->getModel()->getJointPosition(joint_map);
    
    _ik_solver->getModel()->getVelocityLimits(dqlimits);
    
    _ik_solver->getModel()->getJointPosition(x);
    
    // Fill velocity_map with the velocity limits
    _ik_solver->getModel()->eigenToMap(x, velocity_map);
    _ik_solver->getModel()->eigenToMap(dqlimits, velocity_map);
    
    float T = 0.0;
    double dt = 0.001;
    int iter = 0;
    
    while(!_vc_context.vc_aggregate.checkAll())
    {
        
        //std::cout << "[NSPG]: iter = " << iter << std::endl;
        
        auto tic = std::chrono::high_resolution_clock::now();
       
        // Acquire colliding chains
        auto colliding_chains = _vc_context.planning_scene->getCollidingChains();
        
        // Generate a random velocity vector for colliding chains' joints only every n iterations
        if (iter % 100 == 0)
        {
            _ik_solver->getModel()->eigenToMap(x, joint_map);
            random_map = generateRandomVelocities(colliding_chains);          
        }
        

        // Update joint_map with integrated random velocities       
        for (auto i : random_map)
            joint_map[i.first] += i.second * dt;
        
        iter ++;
     
        _ik_solver->getCI()->setReferencePosture(joint_map);
        if (!_ik_solver->solve())
        { 
            auto toc = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> fsec = toc-tic;
            T += fsec.count();
            if(T >= timeout)
            {
                std::cout << "[NSPG]: timeout" <<std::endl;
                return false;
            }
            continue;
        }
        
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

double NSPG::generateRandom() 
{    
    return randDistribution(randGenerator); //PF
}

/*
XBot::JointNameMap NSPG::generateRandomVelocities(std::vector<XBot::ModelChain> colliding_chains) 
{
    XBot::JointNameMap random_map, chain_map, velocityLim_map;
    Eigen::VectorXd velocity_lim;
    
    _ik_solver->getModel()->getVelocityLimits(velocity_lim);
    
    _ik_solver->getCI()->getReferencePosture(velocityLim_map);
    _ik_solver->getModel()->eigenToMap(velocity_lim, velocityLim_map);
    
    // Add random velocities for colliding chains
    if (!_vc_context.vc_aggregate.check("collisions")) 
    {
        for (auto i:colliding_chains)
        {
            // Here you can add extra joints to the kinematic chains in collision.           
            
            i.getJointPosition(chain_map);
            
//             if (i.getChainName() == "head")
//             {
                random_map.insert(std::make_pair("VIRTUALJOINT_1", 50*generateRandom()));
                random_map.insert(std::make_pair("VIRTUALJOINT_2", 50*generateRandom()));
                random_map.insert(std::make_pair("VIRTUALJOINT_3", 50*generateRandom()));
//             }
            
                
            for (auto j : chain_map)
            {
                j.second = generateRandom() * velocityLim_map[j.first];
                random_map.insert(std::make_pair(j.first, j.second));
            }
            
        }
    }
    if (_vc_context.vc_aggregate.exist("stability"))
    {
        if (!_vc_context.vc_aggregate.check("stability"))
        {
            random_map.insert(std::make_pair("VIRTUALJOINT_1", 50*generateRandom()));
            random_map.insert(std::make_pair("VIRTUALJOINT_2", 50*generateRandom()));
            random_map.insert(std::make_pair("VIRTUALJOINT_3", 50*generateRandom()));
        }
    }
    
    // Add random velocities to the floating base when the convex hull check fails

    return random_map;
}
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////
bool NSPG::sample(double timeout, Stance sigmaSmall, Stance sigmaLarge) 
{
    // BE SURE THAT _ik_solver AND _vc_context HAS THE SAME MODEL
    Eigen::VectorXd x, dqlimits;
    XBot::JointNameMap chain_map, joint_map, velocity_map, random_map;
    
    // Start initializing joint_map
    _ik_solver->getModel()->getJointPosition(joint_map);
    _ik_solver->getModel()->getVelocityLimits(dqlimits);
    _ik_solver->getModel()->getJointPosition(x);
    
    // Fill velocity_map with the velocity limits
    _ik_solver->getModel()->eigenToMap(x, velocity_map);
    _ik_solver->getModel()->eigenToMap(dqlimits, velocity_map);
    
    float T = 0.0;
    double dt = 0.01;
    int iter = 0;
    unsigned int counter = 0;
    unsigned int max_counter = 1;
    
    initializeBalanceCheck(sigmaSmall);
    
    bool collisionCheckRes = _vc_context.vc_aggregate.check("collisions");
    bool balanceCheckRes = balanceCheck(sigmaSmall);
        
    while(!collisionCheckRes || !balanceCheckRes || counter < max_counter)
    {
        auto tic = std::chrono::high_resolution_clock::now();
        
        // Acquire colliding chains
        auto colliding_chains = _vc_context.planning_scene->getCollidingChains();
        
        // Generate a random velocity vector for colliding chains' joints only every n iterations
        if (iter % 100 == 0)
        {
            _ik_solver->getModel()->eigenToMap(x, joint_map);
            random_map = generateRandomVelocities(collisionCheckRes, balanceCheckRes, colliding_chains);
        }
         
        // Update joint_map with integrated random velocities       
        for (auto i : random_map)
            joint_map[i.first] += i.second * dt;
        
        iter ++;

        _ik_solver->getCI()->setReferencePosture(joint_map);
        if (!_ik_solver->solve())
        {
            auto toc = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> fsec = toc-tic;
            T += fsec.count();
            if(T >= timeout)
            {
                std::cout << "timeout" <<std::endl;
                foutLogNSPG << iter << std::endl;
    
                return false;
            }
            continue;
        }
        
        collisionCheckRes = _vc_context.vc_aggregate.check("collisions");
        balanceCheckRes = balanceCheck(sigmaSmall);
        if (collisionCheckRes && balanceCheckRes)
            counter += 1;
        else
            counter = 0;

        _rspub->publishTransforms(ros::Time::now(), "/planner");
                        
        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> fsec = toc-tic;
        T += fsec.count();
        if(T >= timeout)
        {
            std::cout << "timeout" <<std::endl;
            foutLogNSPG << iter << std::endl;
    
            return false;
        }
    }
    
    std::cout << "[NSPG]: solution found in " << T << " seconds!" << std::endl;
    foutLogNSPG << iter << std::endl;
    return true;
}

XBot::JointNameMap NSPG::generateRandomVelocities(bool collisionCheckRes, bool balanceCheckRes, std::vector<XBot::ModelChain> colliding_chains){
    XBot::JointNameMap random_map, chain_map, velocityLim_map;
    Eigen::VectorXd velocity_lim;
    
    _ik_solver->getModel()->getVelocityLimits(velocity_lim);
    
    _ik_solver->getCI()->getReferencePosture(velocityLim_map);
    _ik_solver->getModel()->eigenToMap(velocity_lim, velocityLim_map);
    
    // Add random velocities for colliding chains
    if (!collisionCheckRes) 
    {
        for (auto i:colliding_chains)
        {
            random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
            random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
            random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));                
            
            i.getJointPosition(chain_map);
                
            for (auto j : chain_map)
            {
                j.second = generateRandom() * velocityLim_map[j.first];
                random_map.insert(std::make_pair(j.first, j.second));
            }
            
        }
    }
    
    // Add random velocities to the floating base when the convex hull check fails
    if (!balanceCheckRes)
    {
        random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
        random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
        random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));
    }

    random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
    random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
    random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));
    
    return random_map;
}

XBot::JointNameMap NSPG::generateRandomVelocities(std::vector<XBot::ModelChain> colliding_chains){
    XBot::JointNameMap random_map, chain_map, velocityLim_map;
    Eigen::VectorXd velocity_lim;
    
    _ik_solver->getModel()->getVelocityLimits(velocity_lim);
    _ik_solver->getCI()->getReferencePosture(velocityLim_map);
    _ik_solver->getModel()->eigenToMap(velocity_lim, velocityLim_map);
    
    random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
    random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
    random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));              
    
    
    for (auto i:colliding_chains)
    {
        i.getJointPosition(chain_map);
            
        for (auto j : chain_map)
        {
            j.second = generateRandom() * velocityLim_map[j.first];
            random_map.insert(std::make_pair(j.first, j.second));
        }
    }
   
    return random_map;
}
//////////////////////////////////////////

bool NSPG::sample(double timeout, Stance sigmaSmall, Stance sigmaLarge, Eigen::Vector3d dir) 
{
    // BE SURE THAT _ik_solver AND _vc_context HAS THE SAME MODEL
    Eigen::VectorXd x, dqlimits;
    XBot::JointNameMap chain_map, joint_map, velocity_map, random_map;
    
    // Start initializing joint_map
    _ik_solver->getModel()->getJointPosition(joint_map);
    _ik_solver->getModel()->getVelocityLimits(dqlimits);
    _ik_solver->getModel()->getJointPosition(x);
    
    // Fill velocity_map with the velocity limits
    _ik_solver->getModel()->eigenToMap(x, velocity_map);
    _ik_solver->getModel()->eigenToMap(dqlimits, velocity_map);
    
    float T = 0.0;
    double dt = 0.01;
    int iter = 0;
    int iterMax = 100;
    
    initializeBalanceCheck(sigmaSmall);
    
    bool collisionCheckRes = _vc_context.vc_aggregate.check("collisions");
    bool balanceCheckRes = balanceCheck(sigmaSmall);
    bool ik_solved = true;
        
    while(!collisionCheckRes || !balanceCheckRes || !ik_solved)
    {
        auto tic = std::chrono::high_resolution_clock::now();
       
        // Acquire colliding chains
        auto colliding_chains = _vc_context.planning_scene->getCollidingChains();
        
        // Generate a random velocity vector for colliding chains' joints only every n iterations
        if (iter % iterMax == 0)
        {
            _ik_solver->getModel()->eigenToMap(x, joint_map);
            //random_map = generateRandomVelocities(collisionCheckRes, balanceCheckRes, colliding_chains, dir);
            random_map = generateRandomVelocities(colliding_chains);
        }
        
        // Update joint_map with integrated random velocities       
        for (auto i : random_map)
            joint_map[i.first] += i.second * dt;
        
        iter ++;
     
        _ik_solver->getCI()->setReferencePosture(joint_map);
        
        ik_solved = _ik_solver->solve();
        collisionCheckRes = _vc_context.vc_aggregate.check("collisions");
        balanceCheckRes = balanceCheck(sigmaSmall);
        
        _rspub->publishTransforms(ros::Time::now(), "/planner");
                        
        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> fsec = toc-tic;
        T += fsec.count();
        
        if(T >= timeout)
        {
            std::cout << "[NSPG]: timeout" <<std::endl;
            foutLogNSPG << T << std::endl;
            return false;
        }
        
    }
    
    std::cout << "[NSPG]: done!" << std::endl;
    foutLogNSPG << T << std::endl;
    return true;
}

std::string getTaskStringName(EndEffector ee){
    std::string ee_str;

    if(ee == L_HAND) ee_str = "TCP_L";
    else if(ee == R_HAND) ee_str = "TCP_R";
    else if(ee == L_FOOT) ee_str = "l_sole";
    else if(ee == R_FOOT) ee_str = "r_sole";
    else if(ee == HEAD) ee_str = "Head";
    else ee_str = "com";

    return ee_str;
}

Eigen::Matrix3d generateRotationFrictionCone(Eigen::Vector3d axis)
{
    Eigen::Matrix3d rot;

    bool vertical = false;
    Eigen::Vector3d aux = axis - Eigen::Vector3d(0.0, 0.0, 1.0);
    if(abs(aux(0)) < 1e-3 && abs(aux(1)) < 1e-3 && abs(aux(2)) < 1e-3) vertical = true;

    if(vertical){
            rot << 1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0;
    }
    else{
            rot <<  0.0, 0.0, -1.0,
                    0.0, 1.0, 0.0,
                    1.0, 0.0, 0.0;
    }

    return rot;
}

void NSPG::initializeBalanceCheck(Stance sigma){    
    std::vector<std::string> active_links;
    std::vector<Eigen::Affine3d> ref_tasks;
    for(int i = 0; i < sigma.getSize(); i++)
    {
        EndEffector ee = sigma.getContact(i)->getEndEffectorName();
        active_links.push_back(getTaskStringName(ee));   
        ref_tasks.push_back(sigma.retrieveContactPose(ee));
    }

    _cs->setContactLinks(active_links);

    _cs->init(false);  

    for (int i = 0; i < sigma.getContacts().size(); i ++)
    {
        auto nC_i = sigma.getContact(i)->getNormal();
        Eigen::Matrix3d rot = generateRotationFrictionCone(nC_i);
        _cs->setContactRotationMatrix(active_links[i], rot);
    }
}


bool NSPG::balanceCheck(Stance sigma){
    
//     std::vector<std::string> active_links;
//     std::vector<Eigen::Affine3d> ref_tasks;
//     for(int i = 0; i < sigma.getSize(); i++)
//     {
//         EndEffector ee = sigma.getContact(i)->getEndEffectorName();
//         active_links.push_back(getTaskStringName(ee));   
//         ref_tasks.push_back(sigma.retrieveContactPose(ee));
//     }
// 
//     _cs->setContactLinks(active_links);
//     
//     _cs->init(false);  
//     
//     for (int i = 0; i < sigma.getContacts().size(); i ++)
//     {
//         auto nC_i = sigma.getContact(i)->getNormal();
//         Eigen::Matrix3d rot = generateRotationFrictionCone(nC_i);
//         _cs->setContactRotationMatrix(active_links[i], rot);
//     }
    
    if (_cs->checkStability(CS_THRES)) return true; 
    
    return false;
    
}

XBot::JointNameMap NSPG::generateRandomVelocities(bool collisionCheckRes, bool balanceCheckRes, std::vector<XBot::ModelChain> colliding_chains, Eigen::Vector3d dir) 
{
    XBot::JointNameMap random_map, chain_map, velocityLim_map;
    Eigen::VectorXd velocity_lim;
    
    _ik_solver->getModel()->getVelocityLimits(velocity_lim);
    
    _ik_solver->getCI()->getReferencePosture(velocityLim_map);
    _ik_solver->getModel()->eigenToMap(velocity_lim, velocityLim_map);
    
    double alpha_pos = 50.0;
    double alpha_rpy = 10.0;

    random_map.insert(std::make_pair("VIRTUALJOINT_1", 50.0*generateRandom()));
    random_map.insert(std::make_pair("VIRTUALJOINT_2", 50.0*generateRandom()));
    random_map.insert(std::make_pair("VIRTUALJOINT_3", 50.0*generateRandom()));
     
    
    /*
    random_map.insert(std::make_pair("VIRTUALJOINT_1", dir(0)*alpha_pos));
    random_map.insert(std::make_pair("VIRTUALJOINT_2", dir(1)*alpha_pos));
    random_map.insert(std::make_pair("VIRTUALJOINT_3", alpha_pos*generateRandom()));
    random_map.insert(std::make_pair("VIRTUALJOINT_4", alpha_rpy*generateRandom()));
    random_map.insert(std::make_pair("VIRTUALJOINT_5", alpha_rpy*generateRandom()));
    random_map.insert(std::make_pair("VIRTUALJOINT_6", alpha_rpy*generateRandom())); 
    */ 
    
    return random_map;
}

