#include "NSPG.h"

using namespace XBot::Cartesian::Planning;

// PF
static std::default_random_engine randGenerator;
static std::uniform_real_distribution<double> randDistribution(-1.0, 1.0);

NSPG::NSPG ( PositionCartesianSolver::Ptr ik_solver, ValidityCheckContext vc_context ):
    _ik_solver(ik_solver),
    _vc_context(vc_context)
    {
        
        // PF
        auto a = std::chrono::system_clock::now();
        time_t b = std::chrono::system_clock::to_time_t(a);
        randGenerator.seed(b);
        
        _rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_ik_solver->getModel()); 
    }
    
void NSPG::setIKSolver ( PositionCartesianSolver::Ptr new_ik_solver )
{
    _ik_solver = new_ik_solver;
}

PositionCartesianSolver::Ptr NSPG::getIKSolver () const 
{
    return _ik_solver;
}

bool NSPG::sample(double timeout)    
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
    
    bool ik_solved, collision_free, balanced; 
    
    while(T < timeout)
    {
        
        std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
        
        auto tic = std::chrono::high_resolution_clock::now();
       
        ik_solved = _ik_solver->solve();
        _rspub->publishTransforms(ros::Time::now(), "/planner");
        
        if(!ik_solved) return false;
    
        collision_free = _vc_context.vc_aggregate.checkAll(); // in this form, vc_context must contain (self-)collision check only
        balanced = true; // CS
        
        if(collision_free && balanced) return true;
        
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

        
        
        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> fsec = toc-tic;
        T += fsec.count();

    }
    
    std::cout << "timeout" <<std::endl;
    return false;
}

bool NSPG::sampleOLD(double timeout, std::vector<EndEffector> activeEEsDes, Eigen::MatrixXd rCDes, Eigen::MatrixXd nCDes)    
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
    double dt = 0.1; //0.01;
    int iter = 0;
    int iterMax = 100;
    
    bool ik_solved, collision_free, balanced; 
    
    //std::cout << "rCDes = " << std::endl;
    //std::cout << rCDes << std::endl;
    
    Eigen::VectorXd cInit;
    _ik_solver->getModel()->getJointPosition(cInit);
    
    std::cout << "N = " << activeEEsDes.size() << std::endl;
    
    while(T < timeout)
    {
        
        //std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
        
        auto tic = std::chrono::high_resolution_clock::now();
       
        ik_solved = _ik_solver->solve();
        _rspub->publishTransforms(ros::Time::now(), "/planner");
        
        //if(!ik_solved) return false;
        if(!ik_solved) _ik_solver->getModel()->setJointPosition(cInit); ////////////////// TODO OCCHIO QUA
        /*
        if(!ik_solved)
        {
            std::cout << "QUAAAAAAAAAAAAAAAAAA" <<std::endl;
            Eigen::VectorXd qrand;
            qrand.setRandom(_ik_solver->getModel()->getJointNum()); // uniform in -1 < x < 1
            qrand = (qrand.array() + 1)/2.0; // uniform in 0 < x < 1
            Eigen::VectorXd _qmin(_ik_solver->getModel()->getJointNum());
            Eigen::VectorXd _qmax(_ik_solver->getModel()->getJointNum());
            _ik_solver->getModel()->getJointLimits(_qmin, _qmax);
            qrand = _qmin + qrand.cwiseProduct(_qmax - _qmin); // uniform in qmin < x < qmax
            if(_ik_solver->getModel()->isFloatingBase())
            {
                qrand.head<6>().setRandom(); // we keep virtual joints between -1 and 1 (todo: improve)
                qrand.head<6>().tail<3>() *= M_PI;
            }
            _ik_solver->getModel()->setJointPosition(qrand); 
            //iter = iterMax;
        }
        */    
        else
        {
            collision_free = _vc_context.vc_aggregate.check("collisions");
            
            Eigen::Affine3d T_CoM;
            _ik_solver->getCI()->getCurrentPose("com", T_CoM);
            Eigen::Vector3d rCoMDes = T_CoM.translation();
            Eigen::MatrixXd FC(activeEEsDes.size(), 3);
            balanced = computeCentroidalStatics(activeEEsDes, rCoMDes, rCDes, nCDes, FC);
            //if(balanced_prova) std::cout << "balanced" << std::endl;
            //else std::cout << "not balanced" << std::endl;
            //std::cout << "FC" << std::endl;
            //std::cout << FC << std::endl;
            
            //std::cout << "rCoMDes = " << rCoMDes.transpose() << std::endl;
            
            if(collision_free && balanced)
            {
                std::cout << "found" <<std::endl;
                return true;
            }
            
            /*
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
            */
            
                
            // Generate a random velocity vector for colliding chains' joints only every n iterations
            //if (iter % iterMax == 0)
            //{
                //_ik_solver->getModel()->setJointPosition(cInit); ////////////////// TODO OCCHIO QUA
                
                _ik_solver->getModel()->eigenToMap(x, joint_map);
            
                if(!collision_free)
                {
                    // Acquire colliding chains
                    auto colliding_chains = _vc_context.planning_scene->getCollidingChains();
                    random_map = generateRandomVelocitiesCollision(colliding_chains);          
                }
                if(!balanced)
                {
                    random_map = generateRandomVelocitiesBalance();     
                }
            //}
            // Update joint_map with integrated random velocities       
            for (auto i : random_map)
                joint_map[i.first] += i.second * dt;
            iter ++;
            _ik_solver->getCI()->setReferencePosture(joint_map);  
            
            Eigen::VectorXd qPostural;
            qPostural.setRandom(_ik_solver->getModel()->getJointNum());
            _ik_solver->getModel()->eigenToMap(qPostural, joint_map);
            
            std::cout << "pFB qPostural = " << qPostural.segment(0,3)<< std::endl;
        }    
        
        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> fsec = toc-tic;
        T += fsec.count();

    }
    
    std::cout << "timeout" <<std::endl;
    return false;
}


// RIADATTAMENTO MIO
bool NSPG::sample(double timeout, std::vector<EndEffector> activeEEsDes, Eigen::MatrixXd rCDes, Eigen::MatrixXd nCDes)
{
    
    
    // BE SURE THAT _ik_solver AND _vc_context HAS THE SAME MODEL
    Eigen::VectorXd x, dqlimits;
    XBot::JointNameMap chain_map, joint_map, velocity_map, random_map;
    
    _ik_solver->getCI()->setActivationState("com", XBot::Cartesian::ActivationState::Disabled); //TODO
     _ik_solver->solve(); //FIXME
    
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
    
    //FIXME maybe a solve ik here! (it is not called in the planner)
    
    bool collision_free = _vc_context.vc_aggregate.check("collisions");
            
    Eigen::Affine3d T_CoM;
    _ik_solver->getCI()->getCurrentPose("com", T_CoM);
    Eigen::Vector3d rCoMDes = T_CoM.translation();
    Eigen::MatrixXd FC(activeEEsDes.size(), 3);
    bool balanced = computeCentroidalStatics(activeEEsDes, rCoMDes, rCDes, nCDes, FC);
        
   
    //while(!_vc_context.vc_aggregate.checkAll() || counter < max_counter)
    while(!(collision_free && balanced) || counter < max_counter)
    {
        auto tic = std::chrono::high_resolution_clock::now();
        
        _ik_solver->getCI()->setActivationState("com", XBot::Cartesian::ActivationState::Disabled); //TODO
    
        
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
        _ik_solver->solve();
        
        /////////////////////////////////////////////////////////////////////////////////////
        _rspub->publishTransforms(ros::Time::now(), "/planner");
        
        collision_free = _vc_context.vc_aggregate.check("collisions");

        Eigen::Affine3d T_CoM;
        _ik_solver->getCI()->getCurrentPose("com", T_CoM);
        Eigen::Vector3d rCoMDes = T_CoM.translation();
        Eigen::MatrixXd FC(activeEEsDes.size(), 3);
        balanced = computeCentroidalStatics(activeEEsDes, rCoMDes, rCDes, nCDes, FC);
        ////////////////////////////////////////////////////////////////////////////////////
        
        //if (_vc_context.vc_aggregate.checkAll())
        if (collision_free && balanced)
            counter += 1;
        else
            counter = 0;

        //_rspub->publishTransforms(ros::Time::now(), "/NSPG");
                        
        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> fsec = toc-tic;
        T += fsec.count();
        if(T >= timeout)
        {
            std::cout << "timeout" <<std::endl;
            return false;
        }
    }
    
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
//             // Here you can add extra joints to the kinematic chains in collision.
//             if (i.getChainName() == "front_right_leg" || i.getChainName() == "front_left_leg" || i.getChainName() == "rear_right_leg" || i.getChainName() == "rear_left_leg")
//             {
//                 random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));
//                 random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
//                 random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
//             }
//             
//             if (i.getChainName() == "right_arm" || i.getChainName() == "left_arm")
//             {
//                 random_map.insert(std::make_pair("torso_yaw", 2 * generateRandom() * velocityLim_map["torso_yaw"]));  // UNCOMMENT FOR CENTAURO
//                 random_map.insert(std::make_pair("WaistYaw", 2 * generateRandom() * velocityLim_map["WaistYaw"]));   // UNCOMMENT THIS FOR COMANPLUS
//             }
//             
//             if (i.getChainName() == "arm_A" || i.getChainName() == "arm_B" || i.getChainName() == "arm_C")
//             {
//                 random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
//                 random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
//                 random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));
//             }
            
            if (i.getChainName() == "head")
            {
                random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
                random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
                random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));                
            }
            
            i.getJointPosition(chain_map);
                
            for (auto j : chain_map)
            {
                j.second = generateRandom() * velocityLim_map[j.first];
                random_map.insert(std::make_pair(j.first, j.second));
            }
            
        }
    }
    
    // Add random velocities to the floating base when the convex hull check fails
//     if (!_vc_context.vc_aggregate.check("stability"))
//     {
        random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
        random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
        random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));
//     }

    
    return random_map;
}

bool NSPG::computeCentroidalStatics(std::vector<EndEffector> activeEEsRef, Eigen::Vector3d rCoMRef, Eigen::MatrixXd rCRef, Eigen::MatrixXd nCRef, Eigen::MatrixXd &FC){

    //std::cout << "**************** CPL INVOCATION *******************" << std::endl;

    double robot_mass = _ik_solver->getModel()->getMass();
    double g = GRAVITY;
    double mu = MU_FRICTION;
    double W_CoM = COM_WEIGHT_CPL;

    std::vector<std::string> contact_name;
    std::shared_ptr<cpl::CoMPlanner> cpl;
    cpl::solver::Solution sol;

    contact_name.clear();
    std::string name;

    for(int i = 0; i < activeEEsRef.size(); i++){
        name = std::string("contact_") + std::to_string(i);
        contact_name.push_back(name);
    }

    cpl = std::make_shared<cpl::CoMPlanner>(contact_name, robot_mass);
    cpl->SetMu(mu);

    for(int i = 0; i < activeEEsRef.size(); i++){
        name = contact_name.at(i);
        cpl->SetContactPosition(name, rCRef.row(i));
        cpl->SetContactNormal(name, nCRef.row(i));
        if(activeEEsRef.at(i) == L_HAND || activeEEsRef.at(i) == R_HAND) cpl->SetForceThreshold(name, FORCE_THRES_HAND);
        if(activeEEsRef.at(i) == L_FOOT || activeEEsRef.at(i) == R_FOOT) cpl->SetForceThreshold(name, FORCE_THRES_FOOT);
    }

    cpl->SetCoMRef(rCoMRef);
    cpl->SetCoMWeight(W_CoM);

    int sol_status = cpl->Solve(sol);
    if(sol_status != 0)
        return false;

    Eigen::Vector3d rCoM = sol.com_sol;
    
    double eCoM = (rCoM - rCoMRef).norm();
    std::cout << "rCoMRef = " << rCoMRef.transpose() << std::endl;
    std::cout << "rCoM = " << rCoM.transpose() << std::endl;
    std::cout << "eCoM = " << eCoM << std::endl;
    if(eCoM > BALANCE_THRES) 
        return false;

    int i = 0;
    for (auto& elem: sol.contact_values_map){
        FC.row(i) = elem.second.force_value.transpose();
        i++;   
    }

    return true;
}

XBot::JointNameMap NSPG::generateRandomVelocitiesCollision(std::vector<XBot::ModelChain> colliding_chains) 
{
    XBot::JointNameMap random_map, chain_map, velocityLim_map;
    Eigen::VectorXd velocity_lim;
    
    _ik_solver->getModel()->getVelocityLimits(velocity_lim);    
    _ik_solver->getCI()->getReferencePosture(velocityLim_map);
    _ik_solver->getModel()->eigenToMap(velocity_lim, velocityLim_map);
 
    for (auto i:colliding_chains)
    {
        
        i.getJointPosition(chain_map);
        
        double alpha = 50.0;
    
        random_map.insert(std::make_pair("VIRTUALJOINT_1", alpha*generateRandom()));
        random_map.insert(std::make_pair("VIRTUALJOINT_2", alpha*generateRandom()));
        random_map.insert(std::make_pair("VIRTUALJOINT_3", alpha*generateRandom()));
            
        for (auto j : chain_map)
        {
            j.second = generateRandom() * velocityLim_map[j.first];
            random_map.insert(std::make_pair(j.first, j.second)); 
        }
        
    }
    
    return random_map;
}

XBot::JointNameMap NSPG::generateRandomVelocitiesBalance() 
{
    XBot::JointNameMap random_map, chain_map, velocityLim_map;
    Eigen::VectorXd velocity_lim;
    
    _ik_solver->getModel()->getVelocityLimits(velocity_lim);    
    _ik_solver->getCI()->getReferencePosture(velocityLim_map);
    _ik_solver->getModel()->eigenToMap(velocity_lim, velocityLim_map);
    
    double alpha = 50.0;
    
    /*
    double xRand = 10.0*generateRandom();
    double yRand = 1000.0*generateRandom();
    double zRand = 10.0*generateRandom();
    std::cout << "xRand = " << xRand << std::endl;
    std::cout << "yRand = " << yRand << std::endl;
    std::cout << "zRand = " << zRand << std::endl;
    random_map.insert(std::make_pair("VIRTUALJOINT_1", xRand));
    random_map.insert(std::make_pair("VIRTUALJOINT_2", yRand));
    random_map.insert(std::make_pair("VIRTUALJOINT_3", zRand));
    */
      
    random_map.insert(std::make_pair("VIRTUALJOINT_1", alpha*generateRandom()));
    random_map.insert(std::make_pair("VIRTUALJOINT_2", alpha*generateRandom()));
    random_map.insert(std::make_pair("VIRTUALJOINT_3", alpha*generateRandom()));
     
    
    //random_map.insert(std::make_pair("VIRTUALJOINT_4", alpha*generateRandom()));
    //random_map.insert(std::make_pair("VIRTUALJOINT_5", alpha*generateRandom()));
    //random_map.insert(std::make_pair("VIRTUALJOINT_6", alpha*generateRandom()));
    
    return random_map;
}

