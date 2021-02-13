#include "Planner.hpp"

// random numbers generators and distributions
static std::default_random_engine exploitationGenerator;
static std::uniform_real_distribution<double> exploitationDistribution(0.0, 1.0);
static std::default_random_engine pointGenerator;
static std::uniform_real_distribution<double> pointDistribution(-5.0, 5.0);
static std::default_random_engine integerGenerator;
static std::uniform_int_distribution<int> integerDistribution(0, 100);
static std::default_random_engine pointInWorkspaceGenerator;
static std::uniform_int_distribution<int> pointInWorkspaceDistribution(0, std::numeric_limits<int>::max());
static std::default_random_engine rotationGenerator;
static std::uniform_real_distribution<double> rotationDistribution(-1.0, 1.0);

std::string env(getenv("ROBOTOLOGY_ROOT"));

static std::ofstream foutLogMCP(env + "/soap_bar_rrt/multi_contact_planning/PlanningData/logMCP.txt", std::ofstream::trunc);

// Planner::Planner(Configuration _qInit, std::vector<EndEffector> _activeEEsInit, Configuration _qGoal, std::vector<EndEffector> _activeEEsGoal, Eigen::MatrixXd _pointCloud, Eigen::MatrixXd _pointNormals, std::vector<EndEffector> _allowedEEs, XBot::ModelInterface::Ptr _planner_model, GoalGenerator::Ptr _goal_generator, XBot::Cartesian::Planning::ValidityCheckContext _vc_context){
Planner::Planner(Configuration _qInit,
                 std::vector<EndEffector> _activeEEsInit,
                 Configuration _qGoal,
                 std::vector<EndEffector> _activeEEsGoal,
                 Eigen::MatrixXd _pointCloud,
                 Eigen::MatrixXd _pointNormals,
                 std::vector<EndEffector> _allowedEEs,
                 XBot::ModelInterface::Ptr _planner_model,
                 XBot::Cartesian::Planning::NSPG::Ptr _NSPG,
                 XBot::Cartesian::Planning::ValidityCheckContext _vc_context,
                 ros::NodeHandle& nh):
_nh(nh)
{
    // set model, goal generator and cartesian interface
    planner_model = _planner_model;
// 	goal_generator = _goal_generator;
        NSPG = _NSPG;
        ci = NSPG->getIKSolver()->getCI();
// 	ci = _goal_generator->getCartesianInterface();

    // set the environment representation
    pointCloud = _pointCloud;
    pointNormals = _pointNormals;

    // set number of dof of the robot of interest and joint limits
    n_dof = planner_model->getJointNum();
    planner_model->getJointLimits(qmin, qmax);

    // set initial configuration
    qInit = _qInit;

    // create initial stance
    Eigen::Vector3d rCoMInit = computeCoM(qInit);
    Eigen::MatrixXd rCInit;
    Eigen::MatrixXd nCInit;
    rCInit.resize(_activeEEsInit.size(), 3);
    nCInit.resize(_activeEEsInit.size(), 3);
    std::cout << "INIT POSES" << std::endl;
    for(int i = 0; i < _activeEEsInit.size(); i++){
        Eigen::Affine3d T_i = computeForwardKinematics(qInit, _activeEEsInit.at(i));
        rCInit.row(i) = T_i.translation().transpose();
        nCInit.row(i) = getNormalAtPoint(rCInit.row(i)).transpose();
        std::cout << "EE = " << _activeEEsInit.at(i) << " pos = " << T_i.translation().transpose() << std::endl;
    }
    Eigen::MatrixXd FCInit;
    FCInit.resize(_activeEEsInit.size(), 3);

    bool resCPL_Init = computeCentroidalStatics(_activeEEsInit, rCoMInit, rCInit, nCInit, rCoMInit, rCInit, FCInit);

    if(resCPL_Init) std::cout << "INIT CONFIGURATION IS BALANCED - errorCom = " << (rCoMInit - computeCoM(qInit)).norm() << std::endl;

    for(int i = 0; i < _activeEEsInit.size(); i++){
        Eigen::Affine3d T_i;
        T_i.translation() = rCInit.row(i).transpose();
        T_i.linear() = generateRotationAroundAxis(_activeEEsInit[i], nCInit.row(i).transpose());
        Eigen::Vector3d F_i = FCInit.row(i).transpose();
        Eigen::Vector3d n_i = nCInit.row(i).transpose();
        Contact* c = new Contact(_activeEEsInit.at(i), T_i, F_i, n_i);
        sigmaInit.addContact(c);
    }

    // set goal configuration
    qGoal = _qGoal;

    // create goal stance
    Eigen::Vector3d rCoMGoal = computeCoM(qGoal);
    Eigen::MatrixXd rCGoal;
    Eigen::MatrixXd nCGoal;
    rCGoal.resize(_activeEEsGoal.size(), 3);
    nCGoal.resize(_activeEEsGoal.size(), 3);
    std::cout << "GOAL POSES" << std::endl;
    for(int i = 0; i < _activeEEsGoal.size(); i++){
        Eigen::Affine3d T_i = computeForwardKinematics(qGoal, _activeEEsGoal.at(i));
        rCGoal.row(i) = T_i.translation().transpose();
        nCGoal.row(i) = getNormalAtPoint(rCGoal.row(i)).transpose();
        std::cout << "EE = " << _activeEEsInit.at(i) << " pos = " << T_i.translation().transpose() << std::endl;
    }
    Eigen::MatrixXd FCGoal;
    FCGoal.resize(_activeEEsGoal.size(), 3);

    bool resCPL_Goal = computeCentroidalStatics(_activeEEsGoal, rCoMGoal, rCGoal, nCGoal, rCoMGoal, rCGoal, FCGoal);

    if(resCPL_Goal) std::cout << "GOAL CONFIGURATION IS BALANCED - errorCom = " << (rCoMGoal - computeCoM(qGoal)).norm() << std::endl;

    for(int i = 0; i < _activeEEsGoal.size(); i++){
        Eigen::Affine3d T_i;
        T_i.translation() = rCGoal.row(i).transpose();
        T_i.linear() = generateRotationAroundAxis(_activeEEsGoal[i], nCGoal.row(i).transpose());
        Eigen::Vector3d F_i = FCGoal.row(i).transpose();
        Eigen::Vector3d n_i = nCGoal.row(i).transpose();
        Contact* c = new Contact(_activeEEsGoal.at(i), T_i, F_i, n_i);
        sigmaGoal.addContact(c);
    }

    // add to endEffectorsList all the ee that we want to consider
    for(int i = 0; i < _allowedEEs.size(); i++){
        endEffectorsList.push_back(_allowedEEs.at(i));
    }

    // create an empty tree
    tree = new Tree();

    // seed generators
    auto a = std::chrono::system_clock::now();
    time_t b = std::chrono::system_clock::to_time_t(a);
    integerGenerator.seed(b);
    pointGenerator.seed(2*b);
    exploitationGenerator.seed(3*b);
    pointInWorkspaceGenerator.seed(4*b);
    rotationGenerator.seed(5*b);

    // for collision checking (seems that it is not needed anymore)
    vc_context = _vc_context;

    _pub = _nh.advertise<multi_contact_planning::SetContactFrames>("contacts", 10, true);

    Eigen::Matrix3d rot;
    rot <<  0.0, 0.0, 1.0,
            0.0, 1.0, 0.0,
           -1.0, 0.0, 0.0;

    Eigen::Quaternion<double> quat(rot);
    foutLogMCP << "ROTATION CHECK: \n " << quat.coeffs() << std::endl;

}

Planner::~Planner(){ }

bool Planner::isGoalStance(Vertex* v){

    //if(v == nullptr) return false;

    bool c1; // condition 1: true if the i-th active ee at the goal is also active at v
    bool c2; // condition 2: true if the position of the i-th active ee at the goal is respected at v
    double dist;

    Stance sigma = v->getStance();

    for(int i = 0; i < sigmaGoal.getSize(); i++){
        c1 = false;
        for(int j = 0; j < sigma.getSize(); j++){
            if(sigma.getContact(j)->getEndEffectorName() == sigmaGoal.getContact(i)->getEndEffectorName()){
                c1 = true;
                dist = euclideanDistance(sigma.getContact(j)->getPose().translation(), sigmaGoal.getContact(i)->getPose().translation());
                if(dist < GOAL_TOLERANCE) c2 = true;
                else c2 = false;
            }
        }
        if(!c1) return false;
        if(c1 && !c2) return false;
    }

    return true;
}

Eigen::Vector3d Planner::pickRandomPoint(){
    return Eigen::Vector3d(pointDistribution(pointGenerator), pointDistribution(pointGenerator), pointDistribution(pointGenerator));
}

bool Planner::nonEmptyReachableWorkspace(EndEffector pk, Configuration q){

    double ReachableWorkspaceRadius = 0.0;
    if(pk == L_HAND || pk == R_HAND) ReachableWorkspaceRadius = WORKSPACE_RADIUS_HAND;
    else if(pk == L_FOOT || pk == R_FOOT) ReachableWorkspaceRadius = WORKSPACE_RADIUS_FOOT;

    Eigen::Affine3d T_cur = computeForwardKinematics(q, pk);
    Eigen::Vector3d p_cur = T_cur.translation();

    std::vector<Eigen::Vector3d> pointsInWorkspace;
    for(int i = 0; i < pointCloud.rows(); i++){
        Eigen::Vector3d p = pointCloud.row(i).transpose();
        double d = euclideanDistance(p_cur, p);
        if(d < ReachableWorkspaceRadius) pointsInWorkspace.push_back(p);
    }

    if(pointsInWorkspace.size() == 0) return false;
    return true;
}

Eigen::Vector3d Planner::pickPointInReachableWorkspace(EndEffector pk, Configuration q, Eigen::Vector3d rRand, int &index){

    double ReachableWorkspaceRadius = 0.0;
    if(pk == L_HAND || pk == R_HAND) ReachableWorkspaceRadius = WORKSPACE_RADIUS_HAND;
    else if(pk == L_FOOT || pk == R_FOOT) ReachableWorkspaceRadius = WORKSPACE_RADIUS_FOOT;

    Eigen::Affine3d T_cur = computeForwardKinematics(q, pk);
    Eigen::Vector3d p_cur = T_cur.translation();

    //foutLogMCP << "p_cur = " << p_cur.transpose() << std::endl;

    std::vector<Eigen::Vector3d> pointsInWorkspace;
    std::vector<int> pointsInWorkspaceIndices;
    for(int i = 0; i < pointCloud.rows(); i++){
        Eigen::Vector3d p = pointCloud.row(i).transpose();
        double d = euclideanDistance(p_cur, p);
        if(d < ReachableWorkspaceRadius){
            pointsInWorkspace.push_back(p);
            pointsInWorkspaceIndices.push_back(i);
        }
    }

    //foutLogMCP << "pointsInWorkspace.size() = " << pointsInWorkspace.size() << std::endl;

    double dMin = std::numeric_limits<double>::max();
    int iMin = -1;

    for(int i = 0; i < pointsInWorkspace.size(); i++){
        Eigen::Vector3d p = pointsInWorkspace.at(i);
        double d = euclideanDistance(p, rRand);

        if(d < dMin){
            dMin = d;
            iMin = i;
        }
    }

    Eigen::Vector3d r = pointsInWorkspace.at(iMin);

    index = pointsInWorkspaceIndices.at(iMin);

    /*
    ////////////////////////////////////////////////////////////////////////
    if(dMin < 1e-03){
        Eigen::Vector3d r = pointsInWorkspace.at(iMin);
        index = pointsInWorkspaceIndices.at(iMin);
    }
    else{
        iMin = pointInWorkspaceDistribution(pointInWorkspaceGenerator) % pointsInWorkspace.size();
        Eigen::Vector3d r = pointsInWorkspace.at(iMin);
        index = pointsInWorkspaceIndices.at(iMin);
    }
    ////////////////////////////////////////////////////////////////////////
    */

    return r;
}

EndEffector Planner::pickRandomEndEffector(){
    int eeIndex = integerDistribution(integerGenerator) % endEffectorsList.size();
    return endEffectorsList.at(eeIndex);
}

Contact* Planner::pickRandomContactFromGoalStance(){
    int eeIndex = integerDistribution(integerGenerator) % sigmaGoal.getSize();
    return sigmaGoal.getContact(eeIndex);
}

int Planner::findNearestVertexIndex(EndEffector pk, Eigen::Vector3d r){
    double dMin = std::numeric_limits<double>::max();
    int iMin = -1;

    std::vector<int> admissibleVertexes;
    std::vector<double> distAdmissibleVertexes;

    for(int i = 0; i < tree->getSize(); i++){
        Vertex* v = tree->getVertex(i);
        Configuration q = v->getConfiguration();
        Stance sigma = v->getStance();

        // evaluate conditons
        bool c1 = true; // the same vertex can be used for expansion at most a given number of times
        bool c2 = true; // at least 3 contact (after expansion)
        bool c3 = true; // non empty workspace for end effector pk if inactive at vnear
        bool c4 = true; // don't move an active ee that is already at the goal

        std::vector<EndEffector> activeEEs = sigma.retrieveActiveEndEffectors();
        std::vector<EndEffector> activeEEsCand;
        if(sigma.isActiveEndEffector(pk)){
            for(int i = 0; i < activeEEs.size(); i++) if(activeEEs.at(i) != pk) activeEEsCand.push_back(activeEEs.at(i));
        }
        else{
            for(int i = 0; i < activeEEs.size(); i++) activeEEsCand.push_back(activeEEs.at(i));
            activeEEsCand.push_back(pk);
        }

        // condition 1
        int nExp = v->getNumExpansionAttempts();
        if(nExp > MAX_NUM_EXP_PER_VERTEX) c1 = false;
        // condition 2
        if(activeEEsCand.size() < 3) c2 = false;
        // condition 3
        // NOTE: maintaining the non-active contacts as close as possible to their previous pose (when they were actually in contact with the env)
        // leads to have always an intersection between the ee ws and the env. This means that condition 3 is always verified.
        //if(!sigma.isActiveEndEffector(pk) && !nonEmptyReachableWorkspace(pk, q)) c3 = false;
        // condition 4
        if(sigma.isActiveEndEffector(pk)){
            Eigen::Vector3d rGoal = sigmaGoal.retrieveContactPose(pk).translation();
            Eigen::Vector3d rCurr = sigma.retrieveContactPose(pk).translation();
            double d = euclideanDistance(rGoal, rCurr);
            if(d < 1e-4) c4 = false;
        }

        bool allConditionsRespected = c1 && c2 && c3 && c4;

        if(allConditionsRespected){
            Eigen::Affine3d T_cur = computeForwardKinematics(q, pk);
            Eigen::Vector3d p_cur = T_cur.translation();

            double dMin_k = euclideanDistance(p_cur, r);

            admissibleVertexes.push_back(i);
            distAdmissibleVertexes.push_back(dMin_k);
        }

    }

    //foutLogMCP << "admissibleVertexes.size() = " << admissibleVertexes.size() << std::endl;
    if(admissibleVertexes.size() == 0) return -1;

    // RRT-like selection

    Eigen::VectorXd invDist(admissibleVertexes.size());
    for(int i = 0; i < admissibleVertexes.size(); i++){
        //foutLogMCP << "distAdmissibleVertexes.at(i) = " << distAdmissibleVertexes.at(i) << std::endl;
        invDist(i) = 1.0 / distAdmissibleVertexes.at(i);
    }
    double invDistSum = invDist.sum();
    double invDistPrev = 0.0;
    for(int i = 0; i < admissibleVertexes.size(); i++){
        invDist(i) = invDist(i) + invDistPrev;
        invDistPrev	= invDist(i);
    }
    Eigen::VectorXd probVector(admissibleVertexes.size());
    probVector = (1.0 / invDistSum) * invDist;
    //foutLogMCP << "probVector = " << probVector.transpose() << std::endl;

    double pr = exploitationDistribution(exploitationGenerator);
    //foutLogMCP << "pr = " << pr << std::endl;
    int index = 0;
    while(pr > probVector(index)) index++;
    //foutLogMCP << "index = " << index << std::endl;

    iMin = admissibleVertexes.at(index);

    // naive tree selection

    //int index = integerDistribution(integerGenerator) % admissibleVertexes.size();
    //iMin = admissibleVertexes.at(index);

    return iMin;

}

std::string Planner::getTaskStringName(EndEffector ee){
    std::string ee_str;

    if(ee == L_HAND) ee_str = "TCP_L";
    else if(ee == R_HAND) ee_str = "TCP_R";
    else if(ee == L_FOOT) ee_str = "l_sole";
    else if(ee == R_FOOT) ee_str = "r_sole";
    else if(ee == HEAD) ee_str = "Head";
    else ee_str = "com";

    return ee_str;
}

EndEffector Planner::getTaskEndEffectorName(std::string ee_str){
    EndEffector ee;

    if(ee_str.compare("TCP_L") == 0) ee = L_HAND;
    else if(ee_str.compare("TCP_R") == 0) ee = R_HAND;
    else if(ee_str.compare("l_sole") == 0) ee = L_FOOT;
    else if(ee_str.compare("r_sole") == 0) ee = R_FOOT;
    else if(ee_str.compare("Head") == 0) ee = HEAD;
    else ee = COM;

    return ee;
}

/*
bool Planner::computeIKSolution(Stance sigma, bool refCoM, Eigen::Vector3d rCoM, Configuration &q, Configuration qPrev){

    // build references
    Eigen::VectorXd q_postural(n_dof);
    q_postural << 0.34515884431887384, -0.06591591904073339, 0.7271543349204505, 2.772752261057329, 1.694067260637883, -2.8326452668824484, 0.02082929860422072, -1.7787860844940504, -0.036421660785962574, 0.9996204693896318, -0.6689316045377748, 0.04679752671173139, -0.0047857492997280225, -1.7034599738559666, -0.06227672563131584, 0.890601586605412, -0.6349870611535411, 0.04271151312504321, -0.02360545515374067, 0.032760740733259075, -0.707631719076811, 0.659625032411939, 0.04654837196558426, -0.9962331912723077, 0.6763772547285989, 0.44353465292278027, -0.2790720832627141, -0.6992796605078045, -0.6140390267081726, -0.10013692237630738, -0.9404489978405196, -0.6420967750257626, 0.3194200132256253, 0.17978778269015258;
    std::vector<std::string> active_tasks;
    std::vector<Eigen::Affine3d> ref_tasks;
    int i_init;
    if(refCoM){
        active_tasks.push_back("com");
        Eigen::Affine3d T_CoM_ref;
        T_CoM_ref.translation() = rCoM;
        T_CoM_ref.linear() = Eigen::Matrix3d::Identity();
        ref_tasks.push_back(T_CoM_ref);
    }
    for(int i = 0; i < sigma.getSize(); i++){
        EndEffector ee = sigma.getContact(i)->getEndEffectorName();
        active_tasks.push_back(getTaskStringName(ee));
        ref_tasks.push_back(sigma.retrieveContactPose(ee));

        foutLogMCP << "EE(stance) =\n " << getTaskStringName(ee) << " pos =" << sigma.retrieveContactPose(ee).matrix() << std::endl;
    }

    // set active links and rotations
//     multi_contact_planning::SetContactFrames contacts;
//     
//     if (refCoM)
//     {
//         contacts.action = multi_contact_planning::SetContactFrames::SET;
//         contacts.frames_in_contact = {active_tasks.begin()+1, active_tasks.end()}; 
//         i_init = 1;
//     }
//     else
//     {
//         contacts.action = multi_contact_planning::SetContactFrames::SET;
//         contacts.frames_in_contact = active_tasks;
//         i_init = 0;
//     }
// 
//     Eigen::Vector3d nC_i;
//     std::vector<geometry_msgs::Quaternion> rotations(sigma.getContacts().size());
//     for (int i = 0; i < sigma.getContacts().size(); i ++)
//     {
//         if (refCoM)
//             nC_i = getNormalAtPoint(ref_tasks[i+1].translation().transpose());   
//         else 
//             nC_i = getNormalAtPoint(ref_tasks[i].translation().transpose());
//         Eigen::Matrix3d rot = generateRotationFrictionCone(nC_i);
//         Eigen::Quaternion<double> quat(rot);
//         rotations[i].x = quat.coeffs().x();
//         rotations[i].y = quat.coeffs().y();
//         rotations[i].z = quat.coeffs().z();
//         rotations[i].w = quat.coeffs().w();
//     }
//     contacts.rotations = rotations;
//     contacts.friction_coefficient = 0.5 * sqrt(2.0);
//     contacts.optimize_torque = false;
//     
//     foutLogMCP << "contacts:" << std::endl;
//     for (auto i : contacts.frames_in_contact)
//         foutLogMCP << i << "  ";
//     foutLogMCP << "\nrotations:" << std::endl;
//     for (auto i : contacts.rotations)
//         foutLogMCP << i << "  ";
//     foutLogMCP << "\n";
//     
//     _pub.publish(contacts);
//     ros::spinOnce();
    
    // set references
    std::vector<std::string> all_tasks;
    all_tasks.push_back("com");
    all_tasks.push_back("TCP_L");
    all_tasks.push_back("TCP_R");
    all_tasks.push_back("l_sole");
    all_tasks.push_back("r_sole");

    if(!refCoM){
        ci->setActivationState(all_tasks[0], XBot::Cartesian::ActivationState::Disabled);
        i_init = 1;
    }
    else{
        ci->setActivationState(all_tasks[0], XBot::Cartesian::ActivationState::Enabled);
        i_init = 0;
    }

    for(int i = i_init; i < all_tasks.size(); i++){
        std::vector<std::string>::iterator it = std::find(active_tasks.begin(), active_tasks.end(), all_tasks[i]);

        if(it == active_tasks.end()){
            Eigen::MatrixXd wM = Eigen::MatrixXd::Identity(ci->getTask(all_tasks.at(i))->getWeight().rows(), ci->getTask(all_tasks.at(i))->getWeight().cols());
            if (all_tasks[i] == "TCP_L" || all_tasks[i] == "TCP_R")
                wM.block<3,3>(3,3) *= 0.001;
            NSPG->getIKSolver()->getCI()->getTask(all_tasks.at(i))->setWeight(wM);
            NSPG->getIKSolver()->getCI()->setPoseReference(all_tasks.at(i), computeForwardKinematics(qPrev, getTaskEndEffectorName(all_tasks.at(i))));
        }
        else{
            Eigen::MatrixXd wM = Eigen::MatrixXd::Identity(ci->getTask(all_tasks.at(i))->getWeight().rows(), ci->getTask(all_tasks.at(i))->getWeight().cols());
            if (all_tasks[i] == "TCP_L" || all_tasks[i] == "TCP_R")
                wM.block<3,3>(3,3) *= 0.001;
            NSPG->getIKSolver()->getCI()->getTask(all_tasks.at(i))->setWeight(wM);
            int index = it - active_tasks.begin();
            NSPG->getIKSolver()->getCI()->setPoseReference(all_tasks.at(i), ref_tasks[index]);
        }
    }

    Eigen::VectorXd cPrev(n_dof);
    Eigen::Vector3d posFB = qPrev.getFBPosition();
    Eigen::Vector3d rotFB = qPrev.getFBOrientation();
    cPrev.segment(0,3) = posFB;
    cPrev.segment(3,3) = rotFB;
    cPrev.tail(n_dof-6) = qPrev.getJointValues();
    NSPG->getIKSolver()->getModel()->setJointPosition(q_postural);
    NSPG->getIKSolver()->getModel()->update();

    // search IK solution
    double time_budget = GOAL_SAMPLER_TIME_BUDGET;
    Eigen::VectorXd c(n_dof);

    NSPG->getIKSolver()->solve();
    NSPG->_rspub->publishTransforms(ros::Time::now(), "/planner");

    if(!NSPG->sample(time_budget))
    {
        NSPG->getIKSolver()->getModel()->getJointPosition(c);
        q.setFBPosition(c.segment(0,3));
        q.setFBOrientation(c.segment(3,3));
        q.setJointValues(c.tail(n_dof-6));
        return false;
    }
    else
    {
        NSPG->getIKSolver()->getModel()->getJointPosition(c);
        q.setFBPosition(c.segment(0,3));
        q.setFBOrientation(c.segment(3,3));
        q.setJointValues(c.tail(n_dof-6));
        return true;
    }
}
*/
    
bool Planner::computeIKSolution(Stance sigma, bool refCoM, Eigen::Vector3d rCoM, Configuration &q, Configuration qPrev){

    // build references
    std::vector<std::string> active_tasks;
    std::vector<Eigen::Affine3d> ref_tasks;
    int i_init;
    if(refCoM){
        active_tasks.push_back("com");
        Eigen::Affine3d T_CoM_ref;
        T_CoM_ref.translation() = rCoM;
        T_CoM_ref.linear() = Eigen::Matrix3d::Identity();
        ref_tasks.push_back(T_CoM_ref);
    }
    for(int i = 0; i < sigma.getSize(); i++){
        EndEffector ee = sigma.getContact(i)->getEndEffectorName();
        active_tasks.push_back(getTaskStringName(ee));
        ref_tasks.push_back(sigma.retrieveContactPose(ee));
        //foutLogMCP << "EE(stance) =\n " << getTaskStringName(ee) << " pos =" << sigma.retrieveContactPose(ee).matrix() << std::endl;
    }
    
    // set references
    std::vector<std::string> all_tasks;
    all_tasks.push_back("com");
    all_tasks.push_back("TCP_L");
    all_tasks.push_back("TCP_R");
    all_tasks.push_back("l_sole");
    all_tasks.push_back("r_sole");

    if(!refCoM){
        ci->setActivationState(all_tasks[0], XBot::Cartesian::ActivationState::Disabled);
        i_init = 1;
    }
    else{
        ci->setActivationState(all_tasks[0], XBot::Cartesian::ActivationState::Enabled);
        i_init = 0;
    }

    for(int i = i_init; i < all_tasks.size(); i++){
        std::vector<std::string>::iterator it = std::find(active_tasks.begin(), active_tasks.end(), all_tasks[i]);
        //Eigen::MatrixXd wM = Eigen::MatrixXd::Identity(ci->getTask(all_tasks.at(i))->getWeight().rows(), ci->getTask(all_tasks.at(i))->getWeight().cols());
        //if (all_tasks[i] == "TCP_L" || all_tasks[i] == "TCP_R") wM.block<3,3>(3,3) *= 0.001;
        //NSPG->getIKSolver()->getCI()->getTask(all_tasks.at(i))->setWeight(wM);     
        if(it == active_tasks.end()){
            NSPG->getIKSolver()->getCI()->setPoseReference(all_tasks.at(i), computeForwardKinematics(qPrev, getTaskEndEffectorName(all_tasks.at(i))));
        }
        else{ //FIXME maybe here only the next two lines could be used
            int index = it - active_tasks.begin();
            NSPG->getIKSolver()->getCI()->setPoseReference(all_tasks.at(i), ref_tasks[index]);
        }
    }

    Eigen::VectorXd cPrev(n_dof);
    Eigen::Vector3d posFB = qPrev.getFBPosition();
    Eigen::Vector3d rotFB = qPrev.getFBOrientation();
    cPrev.segment(0,3) = posFB;
    cPrev.segment(3,3) = rotFB;
    cPrev.tail(n_dof-6) = qPrev.getJointValues();
    NSPG->getIKSolver()->getModel()->setJointPosition(cPrev);
    NSPG->getIKSolver()->getModel()->update();

    // search IK solution
    double time_budget = GOAL_SAMPLER_TIME_BUDGET;
    Eigen::VectorXd c(n_dof);
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////
    if(!NSPG->sample(time_budget))
    {
        NSPG->getIKSolver()->getModel()->getJointPosition(c);
        q.setFBPosition(c.segment(0,3));
        q.setFBOrientation(c.segment(3,3));
        q.setJointValues(c.tail(n_dof-6));
        return false;
    }
    else
    {
        NSPG->getIKSolver()->getModel()->getJointPosition(c);
        q.setFBPosition(c.segment(0,3));
        q.setFBOrientation(c.segment(3,3));
        q.setJointValues(c.tail(n_dof-6));
        return true;
    }
   
   
    /*
    bool solved = NSPG->getIKSolver()->solve();
    if(!solved)
    {
        NSPG->getIKSolver()->getModel()->getJointPosition(c);
        q.setFBPosition(c.segment(0,3));
        q.setFBOrientation(c.segment(3,3));
        q.setJointValues(c.tail(n_dof-6));
        return false;
    }
    else
    {
        NSPG->getIKSolver()->getModel()->getJointPosition(c);
        q.setFBPosition(c.segment(0,3));
        q.setFBOrientation(c.segment(3,3));
        q.setJointValues(c.tail(n_dof-6));
        if(refCoM){
            Eigen::Vector3d rCoMres = computeCoM(q);
            double errorCoM = (rCoM - rCoMres).norm();
            
            foutLogMCP << "rCoMres = " << rCoMres.transpose() << std::endl;
            foutLogMCP << "rCoM = " << rCoM.transpose() << std::endl;
            
            if(errorCoM > 0.001){
                foutLogMCP << "errorCoM = " << errorCoM << std::endl;
            }
            
        }
        
        return true;
    }
    */
    
     
    /*
    //if(!NSPG->getIKSolver()->solve()) return false;
    NSPG->getIKSolver()->solve();
    NSPG->_rspub->publishTransforms(ros::Time::now(), "/planner");
    
    if(!NSPG->sample(time_budget))
    {
        NSPG->getIKSolver()->getModel()->getJointPosition(c);
        q.setFBPosition(c.segment(0,3));
        q.setFBOrientation(c.segment(3,3));
        q.setJointValues(c.tail(n_dof-6));
        return false;
    }
    else
    {
        NSPG->getIKSolver()->getModel()->getJointPosition(c);
        q.setFBPosition(c.segment(0,3));
        q.setFBOrientation(c.segment(3,3));
        q.setJointValues(c.tail(n_dof-6));
        
        if(refCoM){
            Eigen::Vector3d rCoMres = computeCoM(q);
            double errorCoM = (rCoM - rCoMres).norm();
            
            foutLogMCP << "rCoMres = " << rCoMres.transpose() << std::endl;
            foutLogMCP << "rCoM = " << rCoM.transpose() << std::endl;
            
            //if(errorCoM > 0.001){
            //    foutLogMCP << "errorCoM = " << errorCoM << std::endl;
            //    return false;
            //}
                        
        }
        
        return true;
    }
    */
     
}

bool Planner::computeIKSolution(Stance sigmaNear, Stance sigmaNew, Configuration qNear, Configuration &qNew){
    
    Stance sigmaLarge, sigmaSmall;
    if(sigmaNew.getSize() > sigmaNear.getSize())
    {
        sigmaLarge = sigmaNew;
        sigmaSmall = sigmaNear;
    }
    else
    {
        sigmaLarge = sigmaNear;
        sigmaSmall = sigmaNew;
    }

    // build references
    std::vector<std::string> active_tasks;
    std::vector<Eigen::Affine3d> ref_tasks;
    for(int i = 0; i < sigmaLarge.getSize(); i++){
        EndEffector ee = sigmaLarge.getContact(i)->getEndEffectorName();
        active_tasks.push_back(getTaskStringName(ee));
        ref_tasks.push_back(sigmaLarge.retrieveContactPose(ee));
    }
    
    // set references
    std::vector<std::string> all_tasks;
    all_tasks.push_back("com");
    all_tasks.push_back("TCP_L");
    all_tasks.push_back("TCP_R");
    all_tasks.push_back("l_sole");
    all_tasks.push_back("r_sole");

    ci->setActivationState(all_tasks[0], XBot::Cartesian::ActivationState::Disabled);
    int i_init = 1;
    
    for(int i = i_init; i < all_tasks.size(); i++){
        std::vector<std::string>::iterator it = std::find(active_tasks.begin(), active_tasks.end(), all_tasks[i]);
        if(it == active_tasks.end()){
            NSPG->getIKSolver()->getCI()->setPoseReference(all_tasks.at(i), computeForwardKinematics(qNear, getTaskEndEffectorName(all_tasks.at(i))));
        }
        else{ //FIXME maybe here only the next two lines could be used
            int index = it - active_tasks.begin();
            NSPG->getIKSolver()->getCI()->setPoseReference(all_tasks.at(i), ref_tasks[index]);
        }
    }

    Eigen::VectorXd cPrev(n_dof);
    Eigen::Vector3d posFB = qNear.getFBPosition();
    Eigen::Vector3d rotFB = qNear.getFBOrientation();
    cPrev.segment(0,3) = posFB;
    cPrev.segment(3,3) = rotFB;
    cPrev.tail(n_dof-6) = qNear.getJointValues();
    NSPG->getIKSolver()->getModel()->setJointPosition(cPrev);
    NSPG->getIKSolver()->getModel()->update();
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<EndEffector> activeEEsDes = sigmaNew.retrieveActiveEndEffectors();
    Eigen::MatrixXd rCDes(activeEEsDes.size(), 3);
    Eigen::MatrixXd nCDes(activeEEsDes.size(), 3);
    for(int i = 0; i < activeEEsDes.size(); i++)
    {
        rCDes.row(i) = sigmaNew.getContact(i)->getPose().translation().transpose(); 
        nCDes.row(i) = sigmaNew.getContact(i)->getNormal().transpose();
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // search IK solution
    double time_budget = GOAL_SAMPLER_TIME_BUDGET;
    Eigen::VectorXd c(n_dof);
    
    //FIXME sample must return also FC at sigmaNew (independently of small/large)
    bool sol_found = NSPG->sample(time_budget, activeEEsDes, rCDes, nCDes);
    
    if(!sol_found) 
    {
        NSPG->getIKSolver()->getModel()->getJointPosition(c);
        qNew.setFBPosition(c.segment(0,3));
        qNew.setFBOrientation(c.segment(3,3));
        qNew.setJointValues(c.tail(n_dof-6));
        return false;
    }
    else
    {
        NSPG->getIKSolver()->getModel()->getJointPosition(c);
        qNew.setFBPosition(c.segment(0,3));
        qNew.setFBOrientation(c.segment(3,3));
        qNew.setJointValues(c.tail(n_dof-6));
        return true;
    }
     
}

bool Planner::retrieveSolution(std::vector<Stance> &sigmaList, std::vector<Configuration> &qList){
    int iEnd = tree->getSize() - 1;
    int branchSize = 1;

    Vertex* v = tree->getVertex(iEnd);
    bool solutionFound = isGoalStance(v);

    if(!solutionFound){
        sigmaList.clear();
        qList.clear();
        return false;
    }

    sigmaList.push_back(v->getStance());  
    qList.push_back(v->getConfiguration());
    if (v->getTransitionState())
    {
        sigmaList.push_back(v->getStance());
        qList.push_back(v->getTransitionConfiguration());
    }
    int parentIndex = v->getParentIndex();

    while(parentIndex > -1){
        v = tree->getVertex(parentIndex);
        sigmaList.push_back(v->getStance());
        qList.push_back(v->getConfiguration());
        if (v->getTransitionState())
        {
            sigmaList.push_back(v->getStance());
            qList.push_back(v->getTransitionConfiguration());
        }
        parentIndex = v->getParentIndex();
        branchSize++;
    }

    std::reverse(sigmaList.begin(), sigmaList.end());
    std::reverse(qList.begin(), qList.end());

    std::cout << "++++++++++++++++++++++++++++++++++branchSize++++++++++++++++++++++++++++ \n" << branchSize << std::endl;

    return true;
}

bool Planner::computeCentroidalStatics(std::vector<EndEffector> activeEEsDes, Eigen::Vector3d rCoMdes, Eigen::MatrixXd rCdes, Eigen::MatrixXd nCdes, Eigen::Vector3d &rCoM, Eigen::MatrixXd &rC, Eigen::MatrixXd &FC){

    std::cout << "**************** CPL INVOCATION *******************" << std::endl;

    double robot_mass = planner_model->getMass();
    double g = GRAVITY;
    double mu = MU_FRICTION;
    double W_CoM = COM_WEIGHT_CPL;

    if(activeEEsDes.size() == 1){
        rC.row(0) = rCdes.row(0);
        rCoM << rC(0), rC(1), rC(2)+COM_REF_HEIGHT;
        FC.row(0) << 0.0, 0.0, - g * robot_mass;

        return true;
    }

    std::vector<std::string> contact_name;
    std::shared_ptr<cpl::CoMPlanner> cpl;
    cpl::solver::Solution sol;

    contact_name.clear();
    std::string name;

    for(int i = 0; i < activeEEsDes.size(); i++){
        name = std::string("contact_") + std::to_string(i);
        contact_name.push_back(name);
    }

    cpl = std::make_shared<cpl::CoMPlanner>(contact_name, robot_mass);
    cpl->SetMu(mu);

    for(int i = 0; i < activeEEsDes.size(); i++){
        name = contact_name.at(i);
        cpl->SetContactPosition(name, rCdes.row(i));
        cpl->SetContactNormal(name, nCdes.row(i));
        if(activeEEsDes.at(i) == L_HAND || activeEEsDes.at(i) == R_HAND) cpl->SetForceThreshold(name, FORCE_THRES_HAND);
        if(activeEEsDes.at(i) == L_FOOT || activeEEsDes.at(i) == R_FOOT) cpl->SetForceThreshold(name, FORCE_THRES_FOOT);
    }

    cpl->SetCoMRef(rCoMdes);
    cpl->SetCoMWeight(W_CoM);

    int sol_status = cpl->Solve(sol);
    if(sol_status != 0)
        return false;

    rCoM = sol.com_sol;

    int i = 0;
    Eigen::Vector3d F, r, n, F_sum, Torque_sum;
    Eigen::Vector3d rError, nError, F_sumError, Torque_sumError;
    F_sum.setZero();
    Torque_sum.setZero();
    for (auto& elem: sol.contact_values_map){
        r = elem.second.position_value;
        F = elem.second.force_value;
        
        //n = elem.second.normal_value;
        //F_sum += F;
        //Torque_sum += (r - rCoM).cross(F);
        //rError = (r - rCdes.row(i).transpose());   // = 0.0 component-wise
        //nError = (n - nCdes.row(i).transpose());   // = 0.0 component-wise
        //foutLogMCP << "rError = " << rError.transpose() << std::endl;
        //foutLogMCP << "nError = " << nError.transpose() << std::endl;
        //foutLogMCP << "-F.dot(n) = " << -F.dot(n) << std::endl;
        //foutLogMCP << "(F-(n.dot(F))*n).norm() - mu*(F.dot(n)) = " << (F-(n.dot(F))*n).norm() - mu*(F.dot(n)) << std::endl;

        rC.row(i) = r.transpose();
        FC.row(i) = F.transpose();
        i++;   
    }

    //F_sumError = F_sum + Eigen::Vector3d(0.0, 0.0, robot_mass*g);
    //Torque_sumError = Torque_sum + Eigen::Vector3d(0.0, 0.0, 0.0);
    //foutLogMCP << "F_sumError = " << F_sumError.transpose() << std::endl;
    //foutLogMCP << "Torque_sumError = " << Torque_sumError.transpose() << std::endl;

    return true;
}

Eigen::Vector3d Planner::getNormalAtPoint(Eigen::Vector3d p){
    double dMin = std::numeric_limits<double>::max();
    double iMin = -1;
    for(int i = 0; i < pointCloud.rows(); i++){
        Eigen::Vector3d p_i = pointCloud.row(i).transpose();
        double d = euclideanDistance(p_i, p);
        if(d < dMin){
            dMin = d;
            iMin = i;
        }
    }

    Eigen::Vector3d nC = pointNormals.row(iMin).transpose();

    return nC;
}

Eigen::Vector3d Planner::getNormalAtPointByIndex(int index){
    Eigen::Vector3d nC = pointNormals.row(index).transpose();
    return nC;
}

Eigen::Vector3d Planner::computeCoM(Configuration q){
    Eigen::VectorXd c(n_dof);
    Eigen::Vector3d posFB = q.getFBPosition();
    Eigen::Vector3d rotFB = q.getFBOrientation();
    c.segment(0,3) = posFB;
    c.segment(3,3) = rotFB;
    c.tail(n_dof-6) = q.getJointValues();
    planner_model->setJointPosition(c);
    planner_model->update();

    Eigen::Affine3d T_COM;
    std::string link_COM = "com";
    ci->getCurrentPose(link_COM, T_COM);

    Eigen::Vector3d rCoM = T_COM.translation();

    return rCoM;
}

Eigen::Affine3d Planner::computeForwardKinematics(Configuration q, EndEffector ee)
{
    Eigen::VectorXd c(n_dof);
    Eigen::Vector3d posFB = q.getFBPosition();
    Eigen::Vector3d rotFB = q.getFBOrientation();
    c.segment(0,3) = posFB;
    c.segment(3,3) = rotFB;
    c.tail(n_dof-6) = q.getJointValues();
    planner_model->setJointPosition(c);
    planner_model->update();

    Eigen::Affine3d T;
    std::string link = getTaskStringName(ee);
    ci->getCurrentPose(link, T);

    return T;
}

/*
void Planner::run(){

    foutLogMCP << "********************************* PLANNING STARTED *********************************" << std::endl;

    tree->clear();
    Vertex* vInit = new Vertex(sigmaInit, qInit, -1);
    tree->addVertex(vInit);

    int j = 0;
    bool solutionFound = false;

    EndEffector pk;
    Eigen::Vector3d rRand;

    Vertex* vNear;
    Vertex* vNew;

    while(j < MAX_ITERATIONS && !solutionFound)
    {
        foutLogMCP << "j = " << j << std::endl;

        double pr = exploitationDistribution(exploitationGenerator);
        if(pr < EXPLOITATION_RATE)
        {
            // exploitation
            // pick a random contact from sigmaGoal and retrieve the corresponding ee and position
            foutLogMCP << "+++++++++++++++++++++++++++++ EXPLOITATION +++++++++++++++++++++++++++++" << std::endl;
            Contact* c = pickRandomContactFromGoalStance();
            pk = c->getEndEffectorName();
            rRand = c->getPose().translation();
        }
        else
        {
            // exploration
            // pick a random ee and a random point
            foutLogMCP << "+++++++++++++++++++++++++++++ EXPLORATION +++++++++++++++++++++++++++++" << std::endl;
            pk = pickRandomEndEffector();
            rRand = pickRandomPoint();
        }

        int iNear = findNearestVertexIndex(pk, rRand);
        foutLogMCP << "iNear = " << iNear << std::endl;
        foutLogMCP << "pk = " << pk << std::endl;

        if(iNear != -1)
        {
            // a vertex of the tree is available for expansion using ee pk (based on the condition specified in function findNearestVertexIndex)
            vNear = tree->getVertex(iNear);
            Stance sigmaNear = vNear->getStance();
            Configuration qNear = vNear->getConfiguration();
            vNear->increaseNumExpansionAttempts();

            std::vector<EndEffector> activeEEsNear = sigmaNear.retrieveActiveEndEffectors();
            foutLogMCP << "activeEEsNear.size() = " << activeEEsNear.size() << std::endl;
            for(int z = 0; z < activeEEsNear.size(); z++) foutLogMCP << activeEEsNear.at(z) << " ";
            foutLogMCP << std::endl;

            std::vector<EndEffector> activeEEsDes;
            Eigen::Affine3d T_k;
            Eigen::Vector3d n_k;
            if(sigmaNear.isActiveEndEffector(pk))
            {
                foutLogMCP << "REMOVING A CONTACT" << std::endl;
                for(int i = 0; i < activeEEsNear.size(); i++) if(activeEEsNear.at(i) != pk) activeEEsDes.push_back(activeEEsNear.at(i));
            }
            else
            {
                foutLogMCP << "ADDING A CONTACT" << std::endl;
                for(int i = 0; i < activeEEsNear.size(); i++) activeEEsDes.push_back(activeEEsNear.at(i));
                activeEEsDes.push_back(pk);
                int pointIndex;
                T_k.translation() = pickPointInReachableWorkspace(pk, qNear, rRand, pointIndex);
                //T_k.linear() = Eigen::Matrix3d::Identity(3,3);
                T_k.linear() = generateRotationAroundAxis(pk, getNormalAtPointByIndex(pointIndex));
                n_k = getNormalAtPointByIndex(pointIndex);
            }

            foutLogMCP << "activeEEsDes.size() = " << activeEEsDes.size() << std::endl;

            Stance sigmaNew;
            Eigen::Vector3d F_i(0.0, 0.0, 0.0);
            Eigen::Affine3d T_i;
            Eigen::Vector3d n_i;
            for(int i = 0; i < activeEEsDes.size(); i++){
                if(sigmaNear.isActiveEndEffector(activeEEsDes.at(i)))
                {
                        T_i = sigmaNear.retrieveContactPose(activeEEsDes.at(i));
                        n_i = sigmaNear.retrieveContactNormal(activeEEsDes.at(i));
                }
                else
                {
                        T_i = T_k;
                        n_i = n_k;
                }

                Contact* c = new Contact(activeEEsDes.at(i), T_i, F_i, n_i);
                sigmaNew.addContact(c);
            }

            bool similarityCheckRes = similarityCheck(sigmaNew); // true if a similar stance already exists in the tree
            bool distanceCheckRes = distanceCheck(sigmaNew); // true if L_FOOT (R_FOOT) is not too close to L_HAND (R_HAND)
            
            Stance sigmaLarge, sigmaSmall;
            if(sigmaNew.getSize() > sigmaNear.getSize())
            {
                sigmaLarge = sigmaNew;
                sigmaSmall = sigmaNear;
            }
            else
            {
                sigmaLarge = sigmaNear;
                sigmaSmall = sigmaNew;
            }
            std::vector<EndEffector> activeEEsSmall = sigmaSmall.retrieveActiveEndEffectors();          
            
            if(!similarityCheckRes && distanceCheckRes) 
            {
                Configuration qNew;
                
                // COMPUTE IK SOLUTION (NOT BALANCED)
                bool resIK = computeIKSolution(sigmaLarge, false, Eigen::Vector3d(0.0,0.0,0.0), qNew, qNear);
                if(resIK) foutLogMCP << "--------------- GS SUCCESS ---------------" << std::endl;
                else foutLogMCP << "--------------- GS FAIL ---------------" << std::endl;
                
                foutLogMCP << "rCoM_GS = " << computeCoM(qNew) << std::endl;
                
                if (resIK)
                {
                    // COMPUTE CENTROIDAL STATICS
                    Eigen::Vector3d rCoMCand = computeCoM(qNew);
                    
                    Eigen::MatrixXd rCSmall(activeEEsSmall.size(), 3);
                    Eigen::MatrixXd nCSmall(activeEEsSmall.size(), 3);
                    Eigen::Vector3d rCoMTransition;
                    Eigen::MatrixXd FCSmall(activeEEsSmall.size(), 3);
                    for(int i = 0; i < activeEEsSmall.size(); i++)
                    {
                        rCSmall.row(i) = sigmaSmall.getContact(i)->getPose().translation().transpose();
                        nCSmall.row(i) = sigmaSmall.getContact(i)->getNormal().transpose();
                    }
                    bool resCPLtransition = computeCentroidalStatics(activeEEsSmall, rCoMCand, rCSmall, nCSmall, rCoMTransition, rCSmall, FCSmall); // rC not used!!!
                    if(resCPLtransition) foutLogMCP << "--------------- CPL TRANSITION SUCCESS ---------------" << std::endl;
                    else foutLogMCP << "--------------- CPL TRANSITION FAIL ---------------" << std::endl;
                    
                    foutLogMCP << "rCoMTransition = " << rCoMTransition << std::endl;
                    
                    Eigen::MatrixXd rCDes(activeEEsDes.size(), 3);
                    Eigen::MatrixXd nCDes(activeEEsDes.size(), 3);
                    Eigen::Vector3d rCoMNew;
                    Eigen::MatrixXd FC(activeEEsDes.size(), 3);
                    for(int i = 0; i < activeEEsDes.size(); i++)
                    {
                        rCDes.row(i) = sigmaNew.getContact(i)->getPose().translation().transpose();
                        nCDes.row(i) = sigmaNew.getContact(i)->getNormal().transpose();
                    }
                    bool resCPLnew = computeCentroidalStatics(activeEEsDes, rCoMTransition, rCDes, nCDes, rCoMNew, rCDes, FC); // rC not used!!!
                    if(resCPLnew) foutLogMCP << "--------------- CPL NEW SUCCESS ---------------" << std::endl;
                    else foutLogMCP << "--------------- CPL NEW FAIL ---------------" << std::endl;
                    
                    foutLogMCP << "rCoMNew = " << rCoMNew << std::endl;
                    
                    bool transition_check; // at centroidal statics level
                    if(resCPLtransition && resCPLnew && euclideanDistance(rCoMTransition, rCoMNew) < TRANSITION_THRES) transition_check = true;
                    else transition_check = false;
                    
                    if(transition_check) foutLogMCP << "--------------- TRANSITION CHECK SUCCESS ---------------" << std::endl;
                    else foutLogMCP << "--------------- TRANSITION CHECK FAIL ---------------" << std::endl;
    
                    // COMPUTE IK SOLUTION (BALANCED)
                    if(transition_check)
                    {
                        //foutLogMCP << "FCSmall = " << FCSmall << std::endl;
                        //foutLogMCP << "FC = " << FC << std::endl;
                        
                        for(int i = 0; i < sigmaNew.getSize(); i++)
                            sigmaNew.getContact(i)->setForce(FC.row(i).transpose()); // this completes the new stance
                          
                        bool resIK_CoM = computeIKSolution(sigmaLarge, true, rCoMNew, qNew, qNear);
                        if(resIK_CoM) foutLogMCP << "--------------- GS_COM SUCCESS ---------------" << std::endl;
                        else foutLogMCP << "--------------- GS_COM FAIL ---------------" << std::endl;
                        
                        foutLogMCP << "rCoM_GS_COM = " << computeCoM(qNew) << std::endl;

                        if(resIK_CoM)
                        {
                            vNew = new Vertex(sigmaNew, qNew, iNear);
            
                            solutionFound = isGoalStance(vNew);
                            tree->addVertex(vNew);

                            foutLogMCP << "VERTEX # = " << tree->getSize()-1 << std::endl;
                            foutLogMCP << "rCoMTransition = " << rCoMTransition.transpose() << std::endl;
                            foutLogMCP << "rCoMNewRef = " << rCoMNew.transpose() << std::endl;
                            foutLogMCP << "rCoMNew = " << computeCoM(qNew).transpose() << std::endl;
                            foutLogMCP << "L_HAND = " << computeForwardKinematics(qNew, L_HAND).translation().transpose() << std::endl;
                            foutLogMCP << "R_HAND = " << computeForwardKinematics(qNew, R_HAND).translation().transpose() << std::endl;
                            foutLogMCP << "L_FOOT = " << computeForwardKinematics(qNew, L_FOOT).translation().transpose() << std::endl;
                            foutLogMCP << "R_FOOT = " << computeForwardKinematics(qNew, R_FOOT).translation().transpose() << std::endl;    
                        }
                    }
                }

            }

        }
        j++;        
    }

    std::cout << "iters = " << j << std::endl;
    std::cout << "tree size = " << tree->getSize() << std::endl;

}
*/

void Planner::run(){

    foutLogMCP << "********************************* PLANNING STARTED *********************************" << std::endl;

    tree->clear();
    Vertex* vInit = new Vertex(sigmaInit, qInit, -1);
    tree->addVertex(vInit);

    int j = 0;
    bool solutionFound = false;

    EndEffector pk;
    Eigen::Vector3d rRand;

    Vertex* vNear;
    Vertex* vNew;

    while(j < MAX_ITERATIONS && !solutionFound)
    {
        foutLogMCP << "j = " << j << std::endl;

        double pr = exploitationDistribution(exploitationGenerator);
        if(pr < EXPLOITATION_RATE)
        {
            // exploitation
            // pick a random contact from sigmaGoal and retrieve the corresponding ee and position
            foutLogMCP << "+++++++++++++++++++++++++++++ EXPLOITATION +++++++++++++++++++++++++++++" << std::endl;
            Contact* c = pickRandomContactFromGoalStance();
            pk = c->getEndEffectorName();
            rRand = c->getPose().translation();
        }
        else
        {
            // exploration
            // pick a random ee and a random point
            foutLogMCP << "+++++++++++++++++++++++++++++ EXPLORATION +++++++++++++++++++++++++++++" << std::endl;
            pk = pickRandomEndEffector();
            rRand = pickRandomPoint();
        }

        int iNear = findNearestVertexIndex(pk, rRand);
        foutLogMCP << "iNear = " << iNear << std::endl;
        foutLogMCP << "pk = " << pk << std::endl;

        if(iNear != -1)
        {
            // a vertex of the tree is available for expansion using ee pk (based on the condition specified in function findNearestVertexIndex)
            vNear = tree->getVertex(iNear);
            Stance sigmaNear = vNear->getStance();
            Configuration qNear = vNear->getConfiguration();
            vNear->increaseNumExpansionAttempts();

            std::vector<EndEffector> activeEEsNear = sigmaNear.retrieveActiveEndEffectors();
            foutLogMCP << "activeEEsNear.size() = " << activeEEsNear.size() << std::endl;
            for(int z = 0; z < activeEEsNear.size(); z++) foutLogMCP << activeEEsNear.at(z) << " ";
            foutLogMCP << std::endl;

            std::vector<EndEffector> activeEEsDes;
            Eigen::Affine3d T_k;
            Eigen::Vector3d n_k;
            if(sigmaNear.isActiveEndEffector(pk))
            {
                foutLogMCP << "REMOVING A CONTACT" << std::endl;
                for(int i = 0; i < activeEEsNear.size(); i++) if(activeEEsNear.at(i) != pk) activeEEsDes.push_back(activeEEsNear.at(i));
            }
            else
            {
                foutLogMCP << "ADDING A CONTACT" << std::endl;
                for(int i = 0; i < activeEEsNear.size(); i++) activeEEsDes.push_back(activeEEsNear.at(i));
                activeEEsDes.push_back(pk);
                int pointIndex;
                T_k.translation() = pickPointInReachableWorkspace(pk, qNear, rRand, pointIndex);
                //T_k.linear() = Eigen::Matrix3d::Identity(3,3);
                T_k.linear() = generateRotationAroundAxis(pk, getNormalAtPointByIndex(pointIndex));
                n_k = getNormalAtPointByIndex(pointIndex);
            }

            foutLogMCP << "activeEEsDes.size() = " << activeEEsDes.size() << std::endl;

            Stance sigmaNew;
            Eigen::Vector3d F_i(0.0, 0.0, 0.0);
            Eigen::Affine3d T_i;
            Eigen::Vector3d n_i;
            for(int i = 0; i < activeEEsDes.size(); i++){
                if(sigmaNear.isActiveEndEffector(activeEEsDes.at(i)))
                {
                        T_i = sigmaNear.retrieveContactPose(activeEEsDes.at(i));
                        n_i = sigmaNear.retrieveContactNormal(activeEEsDes.at(i));
                }
                else
                {
                        T_i = T_k;
                        n_i = n_k;
                }

                Contact* c = new Contact(activeEEsDes.at(i), T_i, F_i, n_i);
                sigmaNew.addContact(c);
            }

            bool similarityCheckRes = similarityCheck(sigmaNew); // true if a similar stance already exists in the tree
            bool distanceCheckRes = distanceCheck(sigmaNew); // true if L_FOOT (R_FOOT) is not too close to L_HAND (R_HAND)
            
            /*
            Stance sigmaLarge, sigmaSmall;
            if(sigmaNew.getSize() > sigmaNear.getSize())
            {
                sigmaLarge = sigmaNew;
                sigmaSmall = sigmaNear;
            }
            else
            {
                sigmaLarge = sigmaNear;
                sigmaSmall = sigmaNew;
            }
            std::vector<EndEffector> activeEEsSmall = sigmaSmall.retrieveActiveEndEffectors();          
            */
            
            if(!similarityCheckRes && distanceCheckRes) 
            {
                Configuration qNew;
                
                // COMPUTE IK SOLUTION (NOT BALANCED)
                //bool resIK = computeIKSolution(sigmaLarge, false, Eigen::Vector3d(0.0,0.0,0.0), qNew, qNear);
                bool resIK = computeIKSolution(sigmaNear, sigmaNew, qNear, qNew);
                if(resIK) foutLogMCP << "--------------- GS SUCCESS ---------------" << std::endl;
                else foutLogMCP << "--------------- GS FAIL ---------------" << std::endl;
                
                foutLogMCP << "rCoM_GS = " << computeCoM(qNew) << std::endl;
                
                if (resIK)
                {
                    vNew = new Vertex(sigmaNew, qNew, iNear);
            
                    solutionFound = isGoalStance(vNew);
                    tree->addVertex(vNew);

                    foutLogMCP << "VERTEX # = " << tree->getSize()-1 << std::endl;
                    foutLogMCP << "L_HAND = " << computeForwardKinematics(qNew, L_HAND).translation().transpose() << std::endl;
                    foutLogMCP << "R_HAND = " << computeForwardKinematics(qNew, R_HAND).translation().transpose() << std::endl;
                    foutLogMCP << "L_FOOT = " << computeForwardKinematics(qNew, L_FOOT).translation().transpose() << std::endl;
                    foutLogMCP << "R_FOOT = " << computeForwardKinematics(qNew, R_FOOT).translation().transpose() << std::endl;    
                        
                    /*
                    // COMPUTE CENTROIDAL STATICS
                    Eigen::Vector3d rCoMCand = computeCoM(qNew);
                    
                    Eigen::MatrixXd rCSmall(activeEEsSmall.size(), 3);
                    Eigen::MatrixXd nCSmall(activeEEsSmall.size(), 3);
                    Eigen::Vector3d rCoMTransition;
                    Eigen::MatrixXd FCSmall(activeEEsSmall.size(), 3);
                    for(int i = 0; i < activeEEsSmall.size(); i++)
                    {
                        rCSmall.row(i) = sigmaSmall.getContact(i)->getPose().translation().transpose();
                        nCSmall.row(i) = sigmaSmall.getContact(i)->getNormal().transpose();
                    }
                    bool resCPLtransition = computeCentroidalStatics(activeEEsSmall, rCoMCand, rCSmall, nCSmall, rCoMTransition, rCSmall, FCSmall); // rC not used!!!
                    if(resCPLtransition) foutLogMCP << "--------------- CPL TRANSITION SUCCESS ---------------" << std::endl;
                    else foutLogMCP << "--------------- CPL TRANSITION FAIL ---------------" << std::endl;
                    
                    foutLogMCP << "rCoMTransition = " << rCoMTransition << std::endl;
                    
                    Eigen::MatrixXd rCDes(activeEEsDes.size(), 3);
                    Eigen::MatrixXd nCDes(activeEEsDes.size(), 3);
                    Eigen::Vector3d rCoMNew;
                    Eigen::MatrixXd FC(activeEEsDes.size(), 3);
                    for(int i = 0; i < activeEEsDes.size(); i++)
                    {
                        rCDes.row(i) = sigmaNew.getContact(i)->getPose().translation().transpose();
                        nCDes.row(i) = sigmaNew.getContact(i)->getNormal().transpose();
                    }
                    bool resCPLnew = computeCentroidalStatics(activeEEsDes, rCoMTransition, rCDes, nCDes, rCoMNew, rCDes, FC); // rC not used!!!
                    if(resCPLnew) foutLogMCP << "--------------- CPL NEW SUCCESS ---------------" << std::endl;
                    else foutLogMCP << "--------------- CPL NEW FAIL ---------------" << std::endl;
                    
                    foutLogMCP << "rCoMNew = " << rCoMNew << std::endl;
                    
                    bool transition_check; // at centroidal statics level
                    if(resCPLtransition && resCPLnew && euclideanDistance(rCoMTransition, rCoMNew) < TRANSITION_THRES) transition_check = true;
                    else transition_check = false;
                    
                    if(transition_check) foutLogMCP << "--------------- TRANSITION CHECK SUCCESS ---------------" << std::endl;
                    else foutLogMCP << "--------------- TRANSITION CHECK FAIL ---------------" << std::endl;
    
                    // COMPUTE IK SOLUTION (BALANCED)
                    if(transition_check)
                    {
                        //foutLogMCP << "FCSmall = " << FCSmall << std::endl;
                        //foutLogMCP << "FC = " << FC << std::endl;
                        
                        for(int i = 0; i < sigmaNew.getSize(); i++)
                            sigmaNew.getContact(i)->setForce(FC.row(i).transpose()); // this completes the new stance
                          
                        bool resIK_CoM = computeIKSolution(sigmaLarge, true, rCoMNew, qNew, qNear);
                        if(resIK_CoM) foutLogMCP << "--------------- GS_COM SUCCESS ---------------" << std::endl;
                        else foutLogMCP << "--------------- GS_COM FAIL ---------------" << std::endl;
                        
                        foutLogMCP << "rCoM_GS_COM = " << computeCoM(qNew) << std::endl;

                        if(resIK_CoM)
                        {
                            vNew = new Vertex(sigmaNew, qNew, iNear);
            
                            solutionFound = isGoalStance(vNew);
                            tree->addVertex(vNew);

                            foutLogMCP << "VERTEX # = " << tree->getSize()-1 << std::endl;
                            foutLogMCP << "rCoMTransition = " << rCoMTransition.transpose() << std::endl;
                            foutLogMCP << "rCoMNewRef = " << rCoMNew.transpose() << std::endl;
                            foutLogMCP << "rCoMNew = " << computeCoM(qNew).transpose() << std::endl;
                            foutLogMCP << "L_HAND = " << computeForwardKinematics(qNew, L_HAND).translation().transpose() << std::endl;
                            foutLogMCP << "R_HAND = " << computeForwardKinematics(qNew, R_HAND).translation().transpose() << std::endl;
                            foutLogMCP << "L_FOOT = " << computeForwardKinematics(qNew, L_FOOT).translation().transpose() << std::endl;
                            foutLogMCP << "R_FOOT = " << computeForwardKinematics(qNew, R_FOOT).translation().transpose() << std::endl;    
                        }
                    }
                    */
                }

            }

        }
        j++;        
    }

    std::cout << "iters = " << j << std::endl;
    std::cout << "tree size = " << tree->getSize() << std::endl;

}

void Planner::checkSolution(std::vector<Stance> sigmaList, std::vector<Configuration> qList){

    foutLogMCP << "********************************* CHECKING THE SOLUTION *********************************" << std::endl;
    
    Configuration qPrev, qCurr;
    Stance sigmaPrev, sigmaCurr;
    for(int i = 1; i < qList.size(); i++){
        std::cout << "i = " << i << std::endl;
        
        qPrev = qList.at(i-1);
        qCurr = qList.at(i);
        sigmaPrev = sigmaList.at(i-1);
        sigmaCurr = sigmaList.at(i);
        
        Eigen::Vector3d rCoMCurr = computeCoM(qCurr);             
        
        std::vector<EndEffector> activeEEsPrev = sigmaPrev.retrieveActiveEndEffectors();
        std::cout << "activeEEsPrev.size() = " << activeEEsPrev.size() << std::endl;
        std::vector<EndEffector> activeEEsCurr = sigmaCurr.retrieveActiveEndEffectors();
        std::cout << "activeEEsCurr.size() = " << activeEEsCurr.size() << std::endl;
        
        Eigen::MatrixXd rCPrev(activeEEsPrev.size(), 3);
        Eigen::MatrixXd nCPrev(activeEEsPrev.size(), 3);
        Eigen::Vector3d rCoMPrevStar;
        Eigen::MatrixXd FCPrev(activeEEsPrev.size(), 3);
        for(int i = 0; i < activeEEsPrev.size(); i++)
        {
            rCPrev.row(i) = sigmaPrev.getContact(i)->getPose().translation().transpose();
            nCPrev.row(i) = sigmaPrev.getContact(i)->getNormal().transpose();
        }
        bool resCPLPrev = computeCentroidalStatics(activeEEsPrev, rCoMCurr, rCPrev, nCPrev, rCoMPrevStar, rCPrev, FCPrev);
        
        double dCoMcpl = euclideanDistance(rCoMCurr, rCoMPrevStar);
        
        std::cout << "rCoMCurr = " << rCoMCurr.transpose() << std::endl;
        std::cout << "rCoMPrevStar = " << rCoMPrevStar.transpose() << std::endl;
        std::cout << "dCoMcpl = " << dCoMcpl << std::endl;
        
    }

}


int Planner::getTreeSize(){
    return tree->getSize();
}


double Planner::computeHrange(Configuration q){
    Eigen::VectorXd c = q.getJointValues();
    int n = n_dof-6;
    Eigen::VectorXd cMin = qmin.tail(n);
    Eigen::VectorXd cMax = qmax.tail(n);
    Eigen::VectorXd cBar = 0.5*(cMin + cMax);

    double sum = 0.0;
    for(int i = 0; i < n; i++){
        sum += std::pow((c(i) - cBar(i)) / (cMax(i) - cMin(i)), 2.0);
    }

    return 1.0/ (2.0*(double)n);
}

double Planner::computeHtorso(Configuration q){
    Eigen::Vector3d eTorsoCur = q.getFBOrientation();
    Eigen::VectorXd qhome;
    planner_model->getRobotState("home", qhome);
    Eigen::Vector3d eTorsoDes = qhome.segment(3,3);

    Eigen::Vector3d d = eTorsoCur - eTorsoDes;
    return fabs(d(0)) + fabs(d(1)) + fabs(d(2));
}

Eigen::Matrix3d Planner::generateRotationAroundAxis(EndEffector pk, Eigen::Vector3d axis){
        Eigen::Matrix3d rot;

        bool vertical = false;
        Eigen::Vector3d aux = axis - Eigen::Vector3d(0.0, 0.0, 1.0);
    if(abs(aux(0)) < 1e-3 && abs(aux(1)) < 1e-3 && abs(aux(2)) < 1e-3) vertical = true;

    if(pk == L_HAND || pk == R_HAND){
        if(vertical){
                rot << -1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, -1.0;
        }
        else{
                rot <<  0.0, 0.0, 1.0,
                        0.0, 1.0, 0.0,
                        1.0, 0.0, 0.0;
        }
    }
        else{
        if(vertical){
                rot <<  1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0;
        }
        else{
                rot <<  0.0, 0.0, -1.0,
                        0.0, 1.0, 0.0,
                        1.0, 0.0, 0.0;
        }
    }

        return rot;
}

Eigen::Matrix3d Planner::generateRotationFrictionCone(Eigen::Vector3d axis)
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
            rot <<  0.0, 0.0, 1.0,
                    0.0, 1.0, 0.0,
                   -1.0, 0.0, 0.0;
    }

    return rot;
}

bool Planner::similarityCheck(Stance sigmaNew)
{
    Vertex* v;
    Stance sigma;

    for(int i = 0; i < tree->getSize(); i++){
        v = tree->getVertex(i);
        sigma = v->getStance();

        if(sigmaNew.getSize() == sigma.getSize()){

            bool similar = true;

            for(int j = 0; j < sigmaNew.getSize(); j++){
                Contact* cNew = sigmaNew.getContact(j);
                EndEffector eeNew = cNew->getEndEffectorName();
                Eigen::Vector3d posNew = cNew->getPose().translation();

                Eigen::Vector3d pos = sigma.retrieveContactPose(eeNew).translation();

                double error_norm = (posNew - pos).norm();
                if(error_norm > 1e-03) similar = false;
            }

            if(similar) return true;
        }
    }

    return false;
}


bool Planner::distanceCheck(Stance sigmaNew) 
{
    Eigen::Vector3d pLFoot = sigmaNew.retrieveContactPose(L_FOOT).translation();
    Eigen::Vector3d pLHand = sigmaNew.retrieveContactPose(L_HAND).translation();
    Eigen::Vector3d pRFoot = sigmaNew.retrieveContactPose(R_FOOT).translation();
    Eigen::Vector3d pRHand = sigmaNew.retrieveContactPose(R_HAND).translation();
    
    /*
    if(sigmaNew.isActiveEndEffector(L_FOOT) && sigmaNew.isActiveEndEffector(L_HAND))   
        if(euclideanDistance(pLFoot, pLHand) < DIST_THRES)
            return false;
    
    if(sigmaNew.isActiveEndEffector(R_FOOT) && sigmaNew.isActiveEndEffector(R_HAND))   
        if(euclideanDistance(pRFoot, pRHand) < DIST_THRES) return false;
    */
    
    if(sigmaNew.isActiveEndEffector(L_FOOT) && sigmaNew.isActiveEndEffector(L_HAND))   
        if(euclideanDistance(pLFoot, pLHand) < 0.6 || euclideanDistance(pLFoot, pLHand) > 1.8) return false; 
    
    if(sigmaNew.isActiveEndEffector(R_FOOT) && sigmaNew.isActiveEndEffector(R_HAND))   
        if(euclideanDistance(pRFoot, pRHand) < 0.6 || euclideanDistance(pRFoot, pRHand) > 1.8) return false;
    
    if(sigmaNew.isActiveEndEffector(L_HAND) && sigmaNew.isActiveEndEffector(R_HAND))   
        if(euclideanDistance(pLHand, pRHand) > 1.0) return false;
    
    return true;
}



