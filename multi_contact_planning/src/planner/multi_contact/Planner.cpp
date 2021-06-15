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
    NSPG = _NSPG;
    ci = NSPG->getIKSolver()->getCI();
    
    // set the environment representation
    pointCloud = _pointCloud;
    pointNormals = _pointNormals;

    // set number of dof of the robot of interest and joint limits
    n_dof = planner_model->getJointNum();
    planner_model->getJointLimits(qmin, qmax);
    
    // create an empty tree
    tree = std::make_shared<Tree>();

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

    std::vector<std::string> links = {"r_sole", "l_sole", "TCP_R", "TCP_L", "l_ball_tip_d", "r_ball_tip_d"};
    //_cs = std::unique_ptr<XBot::Cartesian::Planning::CentroidalStatics>(new XBot::Cartesian::Planning::CentroidalStatics(NSPG->getIKSolver()->getModel(), links, MU_FRICTION*sqrt(2), true, Eigen::Vector2d(-0.1, 0.1), Eigen::Vector2d(-0.05, 0.05)));
    
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
    _cs = std::unique_ptr<XBot::Cartesian::Planning::CentroidalStatics>(new XBot::Cartesian::Planning::CentroidalStatics(NSPG->getIKSolver()->getModel(), links, MU_FRICTION*sqrt(2), true, CoP_xlim, CoP_ylim));
    
    // set initial configuration
    qInit = _qInit;
    
    // create initial stance
    foutLogMCP << "create initial stance" << std::endl;
    for(int i = 0; i < _activeEEsInit.size(); i++){
        Eigen::Affine3d T_i = computeForwardKinematics(qInit, _activeEEsInit.at(i));
        Eigen::Vector3d F_i(0.0, 0.0, 0.0);
        Eigen::Vector3d n_i = getNormalAtPoint(T_i.translation());
        std::shared_ptr<Contact> c = std::make_shared<Contact>(_activeEEsInit.at(i), T_i, F_i, n_i);
        sigmaInit.addContact(c);
    }
    retrieveContactForces(qInit, sigmaInit);  
    
    // set goal configuration
    qGoal = _qGoal;

    // create goal stance
    foutLogMCP << "create goal stance" << std::endl;
    for(int i = 0; i < _activeEEsGoal.size(); i++){
        Eigen::Affine3d T_i = computeForwardKinematics(qGoal, _activeEEsGoal.at(i));
        Eigen::Vector3d F_i(0.0, 0.0, 0.0);
        Eigen::Vector3d n_i = getNormalAtPoint(T_i.translation());
        std::shared_ptr<Contact> c = std::make_shared<Contact>(_activeEEsGoal.at(i), T_i, F_i, n_i);
        sigmaGoal.addContact(c);
    }
    retrieveContactForces(qGoal, sigmaGoal);  

    // add to endEffectorsList all the ee that we want to consider
    for(int i = 0; i < _allowedEEs.size(); i++){
        endEffectorsList.push_back(_allowedEEs.at(i));
    }

}

Planner::~Planner(){ }

bool Planner::isGoalStance(std::shared_ptr<Vertex> v){

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
    if(pk == L_HAND_C || pk == R_HAND_C || pk == L_HAND_D || pk == R_HAND_D) ReachableWorkspaceRadius = WORKSPACE_RADIUS_HAND;
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

Eigen::Vector3d Planner::pickPointInGrowingReachableWorkspace(EndEffector pk, Configuration q, Eigen::Vector3d rRand, int &index){

    double ReachableWorkspaceRadius = 0.0;
    if(pk == L_HAND_C || pk == R_HAND_C || pk == L_HAND_D || pk == R_HAND_D) ReachableWorkspaceRadius = WORKSPACE_RADIUS_HAND;
    else if(pk == L_FOOT || pk == R_FOOT) ReachableWorkspaceRadius = WORKSPACE_RADIUS_FOOT;

    Eigen::Affine3d T_cur = computeForwardKinematics(q, pk);
    Eigen::Vector3d p_cur = T_cur.translation();

    std::vector<Eigen::Vector3d> pointsInWorkspace;
    std::vector<int> pointsInWorkspaceIndices;
    
    double deltaRadius = 0.1;
    while(pointsInWorkspace.size() == 0){    
        for(int i = 0; i < pointCloud.rows(); i++){
            Eigen::Vector3d p = pointCloud.row(i).transpose();
            double d = euclideanDistance(p_cur, p);
            if(d < ReachableWorkspaceRadius){
                pointsInWorkspace.push_back(p);
                pointsInWorkspaceIndices.push_back(i);
            }
        }
        ReachableWorkspaceRadius+=deltaRadius;
    }
    
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

    return r;
}

Eigen::Vector3d Planner::pickPointInReachableWorkspace(EndEffector pk, Configuration q, Eigen::Vector3d rRand, int &index){

    double ReachableWorkspaceRadius = 0.0;
    if(pk == L_HAND_C || pk == R_HAND_C || pk == L_HAND_D || pk == R_HAND_D) ReachableWorkspaceRadius = WORKSPACE_RADIUS_HAND;
    else if(pk == L_FOOT || pk == R_FOOT) ReachableWorkspaceRadius = WORKSPACE_RADIUS_FOOT;

    Eigen::Affine3d T_cur = computeForwardKinematics(q, pk);
    Eigen::Vector3d p_cur = T_cur.translation();

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

    return r;
}

EndEffector Planner::pickRandomEndEffector(){
    int eeIndex = integerDistribution(integerGenerator) % endEffectorsList.size();
    return endEffectorsList.at(eeIndex);
}

std::shared_ptr<Contact> Planner::pickRandomContactFromGoalStance(){
    int eeIndex = integerDistribution(integerGenerator) % sigmaGoal.getSize();
    return sigmaGoal.getContact(eeIndex);
}

int Planner::findNearestVertexIndex(EndEffector pk, Eigen::Vector3d r){
    double dMin = std::numeric_limits<double>::max();
    int iMin = -1;

    std::vector<int> admissibleVertexes;
    std::vector<double> distAdmissibleVertexes;

    for(int i = 0; i < tree->getSize(); i++){
        std::shared_ptr<Vertex> v = tree->getVertex(i);
        Configuration q = v->getConfiguration();
        Stance sigma = v->getStance();

        // evaluate conditons
        bool c1 = true; // the same vertex can be used for expansion at most a given number of times
        bool c2 = true; // at least 3 contact (after expansion)
        bool c3 = true; // non empty workspace for end effector pk if inactive at vnear
        bool c4 = true; // don't move an active ee that is already at the goal
        bool c5 = true; // don't remove an active ee that has been just added

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

        // condition 5
        int index_parent = v->getParentIndex();
        if(index_parent != -1){
            std::shared_ptr<Vertex> v_parent = tree->getVertex(index_parent);
            Stance sigma_parent = v_parent->getStance();
            if( sigma_parent.getSize() < sigma.getSize() && sigma.getContact(sigma.getSize()-1)->getEndEffectorName() == pk) c5 = false;
        }
        
        bool allConditionsRespected = c1 && c2 && c3 && c4 && c5;

        if(allConditionsRespected){
            Eigen::Affine3d T_cur = computeForwardKinematics(q, pk);
            Eigen::Vector3d p_cur = T_cur.translation();

            double dMin_k = euclideanDistance(p_cur, r);

            admissibleVertexes.push_back(i);
            distAdmissibleVertexes.push_back(dMin_k);
        }

    }

    if(admissibleVertexes.size() == 0) return -1;

    // RRT-like selection

    Eigen::VectorXd invDist(admissibleVertexes.size());
    for(int i = 0; i < admissibleVertexes.size(); i++){
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
    
    double pr = exploitationDistribution(exploitationGenerator);
    int index = 0;
    while(pr > probVector(index)) index++;
    
    iMin = admissibleVertexes.at(index);


    return iMin;

}

bool Planner::retrieveSolution(std::vector<Stance> &sigmaList, std::vector<Configuration> &qList){
    int iEnd = tree->getSize() - 1;
    int branchSize = 1;

    std::shared_ptr<Vertex> v = tree->getVertex(iEnd);
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
        if(activeEEsDes.at(i) == L_HAND_C || activeEEsDes.at(i) == R_HAND_C || activeEEsDes.at(i) == L_HAND_D || activeEEsDes.at(i) == R_HAND_D) cpl->SetForceThreshold(name, FORCE_THRES_HAND);
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
        n = elem.second.normal_value;
        F_sum += F;
        Torque_sum += (r - rCoM).cross(F);

        rError = (r - rCdes.row(i).transpose());   // = 0.0 component-wise
        nError = (n - nCdes.row(i).transpose());   // = 0.0 component-wise

        //foutLogMCP << "rError = " << rError.transpose() << std::endl;
        //foutLogMCP << "nError = " << nError.transpose() << std::endl;
        //foutLogMCP << "-F.dot(n) = " << -F.dot(n) << std::endl;
        //foutLogMCP << "(F-(n.dot(F))*n).norm() - mu*(F.dot(n)) = " << (F-(n.dot(F))*n).norm() - mu*(F.dot(n)) << std::endl;

// 		for(int j = 0; j < 3; j++) if(abs(rError(j)) > 1e-4 || abs(nError(j)) > 1e-4) return false;
// 		if( -F.dot(n) > 1e-4 ) return false;
// 		if( (F-(n.dot(F))*n).norm() - mu*(F.dot(n)) > 1e-4 ) return false;

        rC.row(i) = r.transpose();
        FC.row(i) = F.transpose();
        i++;
    }

    F_sumError = F_sum + Eigen::Vector3d(0.0, 0.0, robot_mass*g);
    Torque_sumError = Torque_sum + Eigen::Vector3d(0.0, 0.0, 0.0);

    //foutLogMCP << "F_sumError = " << F_sumError.transpose() << std::endl;
    //foutLogMCP << "Torque_sumError = " << Torque_sumError.transpose() << std::endl;

// 	for(int j = 0; j < 3; j++) if(abs(F_sumError(j)) > 1e-4 || abs(Torque_sumError(j)) > 1e-4) return false;

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

void Planner::run(){

    foutLogMCP << "********************************* PLANNING STARTED *********************************" << std::endl;
    
    float timeIKandCS = 0.0;
    float timeTotal = 0.0;
    
    auto tic_timeTotal = std::chrono::high_resolution_clock::now();

    tree->clear();
    std::shared_ptr<Vertex> vInit = std::make_shared<Vertex>(sigmaInit, qInit, -1);
    tree->addVertex(vInit);
    
    int j = 0;
    bool solutionFound = false;

    EndEffector pk;
    Eigen::Vector3d rRand;

    //std::shared_ptr<Vertex> vNear;
    //std::shared_ptr<Vertex> vNew;
    
    bool adding;

    std::vector<Configuration> qListVertex;
    std::vector<Stance> sigmaListVertex;

    while(j < MAX_ITERATIONS && !solutionFound)
        {
            foutLogMCP << "j = " << j << std::endl;

            double pr = exploitationDistribution(exploitationGenerator);
            if(pr < EXPLOITATION_RATE)
            {
                // exploitation
                // pick a random contact from sigmaGoal and retrieve the corresponding ee and position
                foutLogMCP << "+++++++++++++++++++++++++++++ EXPLOITATION +++++++++++++++++++++++++++++" << std::endl;
                std::shared_ptr<Contact> c = pickRandomContactFromGoalStance();
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
                std::shared_ptr<Vertex> vNear = tree->getVertex(iNear);
                Stance sigmaNear = vNear->getStance();
                Configuration qNear = vNear->getConfiguration();

                std::vector<EndEffector> activeEEsNear = sigmaNear.retrieveActiveEndEffectors();
                foutLogMCP << "activeEEsNear.size() = " << activeEEsNear.size() << std::endl;
                for(int z = 0; z < activeEEsNear.size(); z++) foutLogMCP << activeEEsNear.at(z) << std::endl;

                std::vector<EndEffector> activeEEsDes;
                Eigen::Affine3d T_k;
                Eigen::Vector3d n_k;
                if(sigmaNear.isActiveEndEffector(pk))
                {
                    foutLogMCP << "REMOVING A CONTACT" << std::endl;
                    adding = false;
                    for(int i = 0; i < activeEEsNear.size(); i++){
                        if(activeEEsNear.at(i) != pk) activeEEsDes.push_back(activeEEsNear.at(i));
                    }
                }   
                else
                {
                    foutLogMCP << "ADDING A CONTACT" << std::endl;
                    adding = true;
                    for(int i = 0; i < activeEEsNear.size(); i++) activeEEsDes.push_back(activeEEsNear.at(i));
                    activeEEsDes.push_back(pk);
                    int pointIndex;
                    //T_k.translation() = pickPointInReachableWorkspace(pk, qNear, rRand, pointIndex);
                    T_k.translation() = pickPointInGrowingReachableWorkspace(pk, qNear, rRand, pointIndex);
                    T_k.linear() = generateRotationAroundAxis(pk, getNormalAtPointByIndex(pointIndex));
                    n_k = getNormalAtPointByIndex(pointIndex);
                }

                foutLogMCP << "activeEEsDes.size() = " << activeEEsDes.size() << std::endl;
                for(int i = 0; i < activeEEsDes.size(); i++){
                    EndEffector ee = activeEEsDes.at(i);
                    std::string ee_str = getTaskStringName(ee);
                    foutLogMCP << ee_str << std::endl;
                }
                
                // generate sigmaNew 
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
                    
                    std::shared_ptr<Contact> c = std::make_shared<Contact>(activeEEsDes.at(i), T_i, F_i, n_i);
                    sigmaNew.addContact(c);
                }
                
                bool similarityCheckRes = similarityCheck(sigmaNew); // true if a similar stance already exists in the tree
                bool distanceCheckRes = distanceCheck(sigmaNew); // true if distance between ees is in range 

                if(!similarityCheckRes && distanceCheckRes) 
                {
                    Configuration qNew;
                    
                    vNear->increaseNumExpansionAttempts();   
                    
                    // determine sigmaLarge and sigmaSmall
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
                    Eigen::MatrixXd rCSmall(activeEEsSmall.size(), 3);
                    Eigen::MatrixXd nCSmall(activeEEsSmall.size(), 3);
                    Eigen::MatrixXd FCSmall(activeEEsSmall.size(), 3);
                    for(int i = 0; i < activeEEsSmall.size(); i++)
                    {
                        rCSmall.row(i) = sigmaSmall.getContact(i)->getPose().translation().transpose();
                        nCSmall.row(i) = sigmaSmall.getContact(i)->getNormal().transpose();
                    }
                    
                    // generate qNew (feasible)
                    
                    auto tic = std::chrono::high_resolution_clock::now();

                    //if(SCENARIO == 2) adding = false;
                    adding = adding && FREE_YAW_ROTATION;
                    bool resIKCS = computeIKandCS(sigmaSmall, sigmaLarge, qNear, qNew, adding);
                    if(resIKCS) foutLogMCP << "--------------- GS SUCCESS ---------------" << std::endl;
                    else foutLogMCP << "--------------- GS FAIL ---------------" << std::endl;
                    
                    auto toc = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<float> fsec = toc-tic;
                    float t_fsec = fsec.count();
                    foutLogMCP << "GS TIME = " << t_fsec << std::endl;
                    timeIKandCS += t_fsec;
                    
                    if(resIKCS){
                        // adjust orientations in sigmaNew 
                        retrieveContactPoses(qNew, sigmaNew);
                            
                        // set forces in sigmaNew
                        retrieveContactForces(qNew, sigmaNew);
                    
                        // add new vertex    
                        std::shared_ptr<Vertex> vNew = std::make_shared<Vertex>(sigmaNew, qNew, iNear);
                        tree->addVertex(vNew);
                        solutionFound = isGoalStance(vNew);

                        foutLogMCP << "VERTEX # = " << tree->getSize()-1 << std::endl;                        
                    }
                }
            }

            j++;

    }

    std::cout << "iters = " << j << std::endl;
    std::cout << "tree size = " << tree->getSize() << std::endl;
    
    auto toc_timeTotal = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> fsec_timeTotal = toc_timeTotal-tic_timeTotal;
    timeTotal += fsec_timeTotal.count();
    
    foutLogMCP << "---------- DATA ----------" << std::endl;
    foutLogMCP << "timeTotal = " << timeTotal << std::endl;
    foutLogMCP << "timeIKandCS = " << timeIKandCS << std::endl;
    foutLogMCP << "iters = " << j << std::endl;
    foutLogMCP << "tree size =  = " << tree->getSize() << std::endl;
    
}


int Planner::getTreeSize(){
    return tree->getSize();
}

bool Planner::similarityCheck(Stance sigmaNew)
{
    std::shared_ptr<Vertex> v;
    Stance sigma;

    for(int i = 0; i < tree->getSize(); i++){
        v = tree->getVertex(i);
        sigma = v->getStance();

        if(sigmaNew.getSize() == sigma.getSize()){

            bool similar = true;

            for(int j = 0; j < sigmaNew.getSize(); j++){
                std::shared_ptr<Contact> cNew = sigmaNew.getContact(j);
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
    Eigen::Vector3d pRFoot = sigmaNew.retrieveContactPose(R_FOOT).translation();
    Eigen::Vector3d pLHandC = sigmaNew.retrieveContactPose(L_HAND_C).translation();
    Eigen::Vector3d pRHandC = sigmaNew.retrieveContactPose(R_HAND_C).translation();
    Eigen::Vector3d pLHandD = sigmaNew.retrieveContactPose(L_HAND_D).translation();
    Eigen::Vector3d pRHandD = sigmaNew.retrieveContactPose(R_HAND_D).translation();
    
    if(SCENARIO == 1 && INIT_INDEX > 0){ 
        if(sigmaNew.isActiveEndEffector(L_FOOT) && sigmaNew.isActiveEndEffector(L_HAND_C))   
            if(euclideanDistance(pLFoot, pLHandC) < DIST_THRES_MIN || euclideanDistance(pLFoot, pLHandC) > DIST_THRES_MAX) return false; 
        
        if(sigmaNew.isActiveEndEffector(R_FOOT) && sigmaNew.isActiveEndEffector(R_HAND_C))   
            if(euclideanDistance(pRFoot, pRHandC) < DIST_THRES_MIN || euclideanDistance(pRFoot, pRHandC) > DIST_THRES_MAX) return false;
            
        if(sigmaNew.isActiveEndEffector(L_FOOT) && sigmaNew.isActiveEndEffector(L_HAND_D))   
            if(euclideanDistance(pLFoot, pLHandD) < DIST_THRES_MIN || euclideanDistance(pLFoot, pLHandD) > DIST_THRES_MAX) return false; 
        
        if(sigmaNew.isActiveEndEffector(R_FOOT) && sigmaNew.isActiveEndEffector(R_HAND_D))   
            if(euclideanDistance(pRFoot, pRHandD) < DIST_THRES_MIN || euclideanDistance(pRFoot, pRHandD) > DIST_THRES_MAX) return false;
    }
        
//     if(sigmaNew.isActiveEndEffector(L_HAND_C) && sigmaNew.isActiveEndEffector(R_HAND_C))   
//         if(euclideanDistance(pLHandC, pRHandC) > DIST_HANDS_THRES_MAX) return false;

    if(SCENARIO == 2 && INIT_INDEX == 0){
        if(sigmaNew.isActiveEndEffector(L_HAND_C) && sigmaNew.isActiveEndEffector(R_HAND_C))
            if(euclideanDistance(pLHandC, pRHandC) < DIST_HANDS_THRES_MAX) return false;
    }

    if(SCENARIO == 2 && INIT_INDEX > 0){
        if(sigmaNew.isActiveEndEffector(L_FOOT) && sigmaNew.isActiveEndEffector(L_HAND_C))   
             if(fabs(pLFoot(2) - pLHandC(2)) < 5.0*WORKSPACE_RADIUS_FOOT || fabs(pLFoot(2) - pLHandC(2)) > 8.0*WORKSPACE_RADIUS_FOOT) return false;
        if(sigmaNew.isActiveEndEffector(R_FOOT) && sigmaNew.isActiveEndEffector(R_HAND_C))   
             if(fabs(pRFoot(2) - pRHandC(2)) < 5.0*WORKSPACE_RADIUS_HAND || fabs(pRFoot(2) - pRHandC(2)) > 8.0*WORKSPACE_RADIUS_HAND) return false;
    }

    if(SCENARIO == 3){
        if(sigmaNew.isActiveEndEffector(L_FOOT) && sigmaNew.isActiveEndEffector(R_FOOT))   
             if(fabs(pLFoot(2) - pRFoot(2)) > WORKSPACE_RADIUS_FOOT) return false;
        if(sigmaNew.isActiveEndEffector(L_HAND_D) && sigmaNew.isActiveEndEffector(R_HAND_D))   
             if(fabs(pLHandD(2) - pRHandD(2)) > WORKSPACE_RADIUS_HAND) return false;
             
        if(sigmaNew.isActiveEndEffector(L_FOOT) && sigmaNew.isActiveEndEffector(L_HAND_D))   
             if(fabs(pLFoot(2) - pLHandD(2)) < 4.0*WORKSPACE_RADIUS_FOOT) return false;
        if(sigmaNew.isActiveEndEffector(R_FOOT) && sigmaNew.isActiveEndEffector(R_HAND_D))   
             if(fabs(pRFoot(2) - pRHandD(2)) < 4.0*WORKSPACE_RADIUS_HAND) return false;
    }
        
    
    return true;
}

void Planner::checkSolutionCS(std::vector<Stance> sigmaList, std::vector<Configuration> qList){
    
    foutLogMCP << "STABILITY CHECK" << std::endl;
    
    Configuration qCurr;
    Stance sigmaCurr;
    for(int i = 0; i < qList.size(); i++){
        foutLogMCP << "************************************ i = " << i << std::endl;
        
        qCurr = qList.at(i);
        sigmaCurr = sigmaList.at(i);
        Eigen::Vector3d rCoMCurr = computeCoM(qCurr);
    
        foutLogMCP << "rCoM = " << rCoMCurr.transpose() << std::endl;
        
        Eigen::VectorXd cNew(n_dof);
        Eigen::Vector3d posFB = qCurr.getFBPosition();
        Eigen::Vector3d rotFB = qCurr.getFBOrientation();
        cNew.segment(0,3) = posFB;
        cNew.segment(3,3) = rotFB;
        cNew.tail(n_dof-6) = qCurr.getJointValues();
        NSPG->getIKSolver()->getModel()->setJointPosition(cNew);
        NSPG->getIKSolver()->getModel()->update();
        
        std::vector<std::string> active_links;
        std::vector<Eigen::Affine3d> ref_tasks;
        std::vector<Eigen::Vector3d> normals;
        for(int i = 0; i < sigmaCurr.getSize(); i++)
        {
            EndEffector ee = sigmaCurr.getContact(i)->getEndEffectorName();
            std::string contact_link = getTaskStringName(ee);
            Eigen::Vector3d nC_i = sigmaCurr.getContact(i)->getNormal();
            active_links.push_back(contact_link);
            normals.push_back(nC_i);
        }
        
        _cs->setContactLinks(active_links);
        
        //if(sigmaCurr.getSize() == 2) _cs->setOptimizeTorque(true);
        //else _cs->setOptimizeTorque(false);
        
        _cs->init(false);  
        
        for (int i = 0; i < sigmaCurr.getContacts().size(); i ++)
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
        
        if (_cs->checkStability(5*1e-2)) foutLogMCP << "CS CHECK TRUE" << std::endl;
        else foutLogMCP << "CS CHECK FALSE" << std::endl;  
        
        /////////////////////// TRANSITION CHECK
        if(i == 0) foutLogMCP << "TRANSITION CHECK TRUE" << std::endl;
        else{
            Stance sigmaPrev = sigmaList.at(i-1);
            Stance sigmaLarge, sigmaSmall;
            if(sigmaCurr.getSize() > sigmaPrev.getSize())
            {
                sigmaLarge = sigmaCurr;
                sigmaSmall = sigmaPrev;
            }
            else
            {
                sigmaLarge = sigmaPrev;
                sigmaSmall = sigmaCurr;
            }
            
            std::vector<std::string> active_links;
            std::vector<Eigen::Affine3d> ref_tasks;
            std::vector<Eigen::Vector3d> normals;
            for(int i = 0; i < sigmaSmall.getSize(); i++)
            {
                EndEffector ee = sigmaSmall.getContact(i)->getEndEffectorName();
                std::string contact_link = getTaskStringName(ee);
                Eigen::Vector3d nC_i = sigmaSmall.getContact(i)->getNormal();
                active_links.push_back(contact_link);
                normals.push_back(nC_i);
            }
            
            _cs->setContactLinks(active_links);
            
            //if(sigmaSmall.getSize() == 2) _cs->setOptimizeTorque(true);
            //else _cs->setOptimizeTorque(false);
            
            _cs->init(false);  
            
            for (int i = 0; i < sigmaSmall.getContacts().size(); i ++)
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
            
            if (_cs->checkStability(5*1e-2)) foutLogMCP << "TRANSITION CHECK TRUE" << std::endl;
            else foutLogMCP << "TRANSITION CHECK FALSE" << std::endl; 
            
        }
    }
}


/*
void Planner::checkSolutionCS(std::vector<Stance> sigmaList, std::vector<Configuration> qList){
    
    foutLogMCP << "STABILITY CHECK" << std::endl;
    
    Configuration qCurr;
    Stance sigmaCurr;
    for(int i = 0; i < qList.size(); i++){
        foutLogMCP << "************************************ i = " << i << std::endl;
        
        qCurr = qList.at(i);
        sigmaCurr = sigmaList.at(i);
        Eigen::Vector3d rCoMCurr = computeCoM(qCurr);
    
        foutLogMCP << "rCoM = " << rCoMCurr.transpose() << std::endl;
        
        Eigen::VectorXd cNew(n_dof);
        Eigen::Vector3d posFB = qCurr.getFBPosition();
        Eigen::Vector3d rotFB = qCurr.getFBOrientation();
        cNew.segment(0,3) = posFB;
        cNew.segment(3,3) = rotFB;
        cNew.tail(n_dof-6) = qCurr.getJointValues();
        NSPG->getIKSolver()->getModel()->setJointPosition(cNew);
        NSPG->getIKSolver()->getModel()->update();

        std::vector<std::string> active_links;
        std::vector<Eigen::Affine3d> ref_tasks;
        for(int i = 0; i < sigmaCurr.getSize(); i++)
        {
            EndEffector ee = sigmaCurr.getContact(i)->getEndEffectorName();
            active_links.push_back(getTaskStringName(ee));
            ref_tasks.push_back(sigmaCurr.retrieveContactPose(ee));
            
            foutLogMCP << sigmaCurr.retrieveContactPose(ee).translation().transpose() << std::endl;
        }

        _cs->setContactLinks(active_links);
        
        if(sigmaCurr.getSize() == 2) _cs->setOptimizeTorque(true);
        else _cs->setOptimizeTorque(false);
        
        _cs->init(false); 

        for (int i = 0; i < sigmaCurr.getContacts().size(); i ++)
        {
            auto nC_i = getNormalAtPoint(ref_tasks[i].translation().transpose());
            Eigen::Matrix3d rot = generateRotationFrictionCone(nC_i);
            _cs->setContactRotationMatrix(active_links[i], rot);
        }

        if (_cs->checkStability(5*1e-2)) foutLogMCP << "CS CHECK TRUE" << std::endl;
        else foutLogMCP << "CS CHECK FALSE" << std::endl;  
        
        /////////////////////// TRANSITION CHECK
        if(i == 0) foutLogMCP << "TRANSITION CHECK TRUE" << std::endl;
        else{
            Stance sigmaPrev = sigmaList.at(i-1);
            Stance sigmaLarge, sigmaSmall;
            if(sigmaCurr.getSize() > sigmaPrev.getSize())
            {
                sigmaLarge = sigmaCurr;
                sigmaSmall = sigmaPrev;
            }
            else
            {
                sigmaLarge = sigmaPrev;
                sigmaSmall = sigmaCurr;
            }
            
            std::vector<std::string> active_links;
            std::vector<Eigen::Affine3d> ref_tasks;
            for(int i = 0; i < sigmaSmall.getSize(); i++)
            {
                EndEffector ee = sigmaSmall.getContact(i)->getEndEffectorName();
                active_links.push_back(getTaskStringName(ee));
                ref_tasks.push_back(sigmaSmall.retrieveContactPose(ee));
                
                //foutLogMCP << sigmaSmall.retrieveContactPose(ee).translation().transpose() << std::endl;
            }

            _cs->setContactLinks(active_links);
            
            if(sigmaSmall.getSize() == 2) _cs->setOptimizeTorque(true);
            else _cs->setOptimizeTorque(false);

            _cs->init(false);  

            for (int i = 0; i < sigmaSmall.getContacts().size(); i ++)
            {
                auto nC_i = getNormalAtPoint(ref_tasks[i].translation().transpose());
                Eigen::Matrix3d rot = generateRotationFrictionCone(nC_i);
                _cs->setContactRotationMatrix(active_links[i], rot);
            }

            if (_cs->checkStability(5*1e-2)) foutLogMCP << "TRANSITION CHECK TRUE" << std::endl;
            else foutLogMCP << "TRANSITION CHECK FALSE" << std::endl; 
            
        }
    }
}
*/

bool Planner::computeIKandCS(Stance sigmaSmall, Stance sigmaLarge, Configuration qNear, Configuration &qNew, bool adding){
    
    std::string added_task = getTaskStringName(sigmaLarge.getContact(sigmaLarge.getSize()-1)->getEndEffectorName());
    
    // build references
    std::vector<std::string> active_tasks;
    std::vector<Eigen::Affine3d> ref_tasks;
    for(int i = 0; i < sigmaLarge.getSize(); i++){
        EndEffector ee = sigmaLarge.getContact(i)->getEndEffectorName();
        active_tasks.push_back(getTaskStringName(ee));
        ref_tasks.push_back(sigmaLarge.retrieveContactPose(ee));
    }

    // set references
    std::vector<std::string> all_tasks = {"r_sole", "l_sole", "TCP_R", "TCP_L", "l_ball_tip_d", "r_ball_tip_d"};
    ci->setActivationState("com", XBot::Cartesian::ActivationState::Disabled); //FIXME useless if CoM not in stack
    
    //FIXME /////////////////////////////////////////////////////////////////////////////////
    ci->setActivationState("l_foot_upper_right_link", XBot::Cartesian::ActivationState::Disabled);
    ci->setActivationState("l_foot_upper_left_link", XBot::Cartesian::ActivationState::Disabled);
    ci->setActivationState("l_foot_lower_right_link", XBot::Cartesian::ActivationState::Disabled);
    ci->setActivationState("l_foot_lower_left_link", XBot::Cartesian::ActivationState::Disabled);
    ci->setActivationState("r_foot_upper_right_link", XBot::Cartesian::ActivationState::Disabled);
    ci->setActivationState("r_foot_upper_left_link", XBot::Cartesian::ActivationState::Disabled);
    ci->setActivationState("r_foot_lower_right_link", XBot::Cartesian::ActivationState::Disabled);
    ci->setActivationState("r_foot_lower_left_link", XBot::Cartesian::ActivationState::Disabled);
    /////////////////////////////////////////////////////////////////////////////////////////
    
    for(int i = 0; i < all_tasks.size(); i++){
        std::vector<std::string> subtasks = getSubtasksStringName(all_tasks[i]);
        std::vector<std::string>::iterator it = std::find(active_tasks.begin(), active_tasks.end(), all_tasks[i]);
        if(it == active_tasks.end()){
            ci->setActivationState(subtasks[0], XBot::Cartesian::ActivationState::Disabled); 
            ci->setActivationState(subtasks[1], XBot::Cartesian::ActivationState::Disabled); 
        }
        else{ 
            ci->setActivationState(subtasks[0], XBot::Cartesian::ActivationState::Enabled); 
            if(adding && added_task.compare(all_tasks[i]) == 0) ci->setActivationState(subtasks[1], XBot::Cartesian::ActivationState::Disabled); 
            else ci->setActivationState(subtasks[1], XBot::Cartesian::ActivationState::Enabled); 
            int index = it - active_tasks.begin();
            ci->setPoseReference(all_tasks[i], ref_tasks[index]);
        } 
    }
    
    // set initial guess (and starting postural in NSPG as well)
    Eigen::VectorXd cPrev(n_dof);
    Eigen::Vector3d posFB = qNear.getFBPosition();
    Eigen::Vector3d rotFB = qNear.getFBOrientation();
    cPrev.segment(0,3) = posFB;
    cPrev.segment(3,3) = rotFB;
    cPrev.tail(n_dof-6) = qNear.getJointValues();
    
    if(SCENARIO == 2 || SCENARIO == 3) NSPG->getIKSolver()->getModel()->getRobotState("home", cPrev);
    
    NSPG->getIKSolver()->getModel()->setJointPosition(cPrev);
    NSPG->getIKSolver()->getModel()->update();
    
    // search IK solution (joint limits) --> qNominal
    double time_budget = GOAL_SAMPLER_TIME_BUDGET;
    Eigen::VectorXd c(n_dof);
    if (!NSPG->getIKSolver()->solve()){
        foutLogMCP << "STOP BEFORE NSPG" << std::endl;
        return false;
    }
    
    // refine IK solution (balance and self-collisions)
    NSPG->_rspub->publishTransforms(ros::Time::now(), "/planner");
   
    if(!NSPG->sample(time_budget, sigmaSmall, sigmaLarge))
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

void Planner::retrieveContactForces(Configuration q, Stance &sigma){
    
    Eigen::VectorXd c(n_dof);
    Eigen::Vector3d posFB = q.getFBPosition();
    Eigen::Vector3d rotFB = q.getFBOrientation();
    c.segment(0,3) = posFB;
    c.segment(3,3) = rotFB;
    c.tail(n_dof-6) = q.getJointValues();
    NSPG->getIKSolver()->getModel()->setJointPosition(c);
    NSPG->getIKSolver()->getModel()->update();
    
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
    
    //if(sigma.getSize() == 2) _cs->setOptimizeTorque(true);
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
    
    if (_cs->checkStability(CS_THRES))
    {
        foutLogMCP << "----------STABILITY CHECK PASSED----------" << std::endl;
        
        std::map<std::string, Eigen::Vector6d> FCmap = _cs->getForces();
        for(int i = 0; i < sigma.getSize(); i++)
        {
            EndEffector ee = sigma.getContact(i)->getEndEffectorName();
            Eigen::Affine3d Tee;
            NSPG->getIKSolver()->getCI()->getCurrentPose(getTaskStringName(ee), Tee);
            
            Eigen::Vector6d wrench;
            wrench.setZero();
            std::string contact_link = getTaskStringName(ee);
            wrench = FCmap.find(contact_link)->second;
            sigma.getContact(i)->setForce(wrench);
        }
    }
}

/*
void Planner::retrieveContactForces(Configuration q, Stance &sigma){
    
    Eigen::VectorXd c(n_dof);
    Eigen::Vector3d posFB = q.getFBPosition();
    Eigen::Vector3d rotFB = q.getFBOrientation();
    c.segment(0,3) = posFB;
    c.segment(3,3) = rotFB;
    c.tail(n_dof-6) = q.getJointValues();
    NSPG->getIKSolver()->getModel()->setJointPosition(c);
    NSPG->getIKSolver()->getModel()->update();
    
    std::vector<std::string> active_links;
    std::vector<Eigen::Affine3d> ref_tasks;
    for(int i = 0; i < sigma.getSize(); i++)
    {
        EndEffector ee = sigma.getContact(i)->getEndEffectorName();
        active_links.push_back(getTaskStringName(ee));
        ref_tasks.push_back(sigma.retrieveContactPose(ee));
        
//         // TODO delete this in the following, it s for debug
//         foutLogMCP << "pose from ref_tasks" << std::endl;
//         foutLogMCP << ref_tasks[i].translation() << std::endl;
//         foutLogMCP << ref_tasks[i].linear() << std::endl;
//         Eigen::Affine3d pose;
//         NSPG->getIKSolver()->getCI()->getCurrentPose(active_links[i], pose);
//         foutLogMCP << "pose from CI" << std::endl;
//         foutLogMCP << pose.translation() << std::endl;
//         foutLogMCP << pose.linear() << std::endl;
        
    }

    _cs->setContactLinks(active_links);
    
    if(sigma.getSize() == 2) _cs->setOptimizeTorque(true);
    else _cs->setOptimizeTorque(false);
    
    _cs->init(false);  
    
    for (int i = 0; i < sigma.getContacts().size(); i ++)
    {
        auto nC_i = getNormalAtPoint(ref_tasks[i].translation().transpose());
        Eigen::Matrix3d rot = generateRotationFrictionCone(nC_i);
        _cs->setContactRotationMatrix(active_links[i], rot);
    }
    
    if (_cs->checkStability(CS_THRES))
    {
        foutLogMCP << "----------STABILITY CHECK PASSED----------" << std::endl;
        for (auto i : _cs->getContactLinks()) foutLogMCP << i << ": \n" << _cs->getContactFrame(i) << std::endl;
        
        std::map<std::string, Eigen::Vector6d> FCmap = _cs->getForces();
        for(int i = 0; i < sigma.getSize(); i++)
        {
            EndEffector ee = sigma.getContact(i)->getEndEffectorName();
            std::string ee_str = getTaskStringName(ee);
            Eigen::Vector6d FC = FCmap.find(ee_str)->second;
            sigma.getContact(i)->setForce(FC.head(3));
            
            foutLogMCP << "ee_str = " << ee_str << std::endl;
            foutLogMCP << "FC = " << FC.transpose() << std::endl;
        }
    }
    
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
//     Eigen::Affine3d T1;
//     NSPG->getIKSolver()->getCI()->getCurrentPose("l_sole", T1);
//     foutLogMCP << "POSE l_sole" << std::endl;
//     foutLogMCP << T1.translation() << std::endl;
//     foutLogMCP << T1.linear() << std::endl;
//     Eigen::Affine3d T2;
//     NSPG->getIKSolver()->getCI()->getCurrentPose("l_foot_upper_left_link", T2);
//     foutLogMCP << "POSE l_upper_left" << std::endl;
//     foutLogMCP << T2.translation() << std::endl;
//     foutLogMCP << T2.linear() << std::endl;
    
    active_links.clear();
    ref_tasks.clear();
    Eigen::Affine3d T;
    for(int i = 0; i < sigma.getSize(); i++)
    {
        EndEffector ee = sigma.getContact(i)->getEndEffectorName();
        std::vector<std::string> contact_links = getContactLinks(ee);
        foutLogMCP << "pose end effector" << std::endl;
        foutLogMCP << sigma.retrieveContactPose(ee).translation() << std::endl;
        foutLogMCP << sigma.retrieveContactPose(ee).linear() << std::endl;
        for(int j = 0; j < contact_links.size(); j++){
            active_links.push_back(contact_links[j]);
            NSPG->getIKSolver()->getCI()->getCurrentPose(contact_links[j], T);
            ref_tasks.push_back(T);
            
            foutLogMCP << "pose contact link" << std::endl;
            foutLogMCP << T.translation() << std::endl;
            foutLogMCP << T.linear() << std::endl;
                
        }
    }
    
    _cs->setContactLinks(active_links);
    
    if(sigma.getSize() == 2) _cs->setOptimizeTorque(true);
    else _cs->setOptimizeTorque(false);
    
    _cs->init(false);  
    
    for (int i = 0; i < sigma.getContacts().size(); i ++)
    {
        auto nC_i = getNormalAtPoint(ref_tasks[i].translation().transpose());
        Eigen::Matrix3d rot = generateRotationFrictionCone(nC_i);
        _cs->setContactRotationMatrix(active_links[i], rot);
    }
    
    if (_cs->checkStability(CS_THRES))
    {
        foutLogMCP << "----------STABILITY CHECK SURFACE PASSED----------" << std::endl;
        for (auto i : _cs->getContactLinks()) foutLogMCP << i << ": \n" << _cs->getContactFrame(i) << std::endl;
        
        std::map<std::string, Eigen::Vector6d> FCmap = _cs->getForces();
        for(int i = 0; i < sigma.getSize(); i++)
        {
            EndEffector ee = sigma.getContact(i)->getEndEffectorName();
            Eigen::Affine3d Tee;
            NSPG->getIKSolver()->getCI()->getCurrentPose(getTaskStringName(ee), Tee);
            
            Eigen::Vector6d wrench;
            wrench.setZero();
            std::vector<std::string> contact_links = getContactLinks(ee);
            for(int j = 0; j < contact_links.size(); j++){
                Eigen::Affine3d Tp;
                NSPG->getIKSolver()->getCI()->getCurrentPose(contact_links[j], Tp);
            
                Eigen::Vector6d FC = FCmap.find(contact_links[j])->second;
                
                Eigen::Vector3d force = FC.head(3);
                Eigen::Vector3d torque = (Tp.translation() - Tee.translation()).cross(force);
                
                wrench.head(3) += force;
                wrench.tail(3) += torque;                    
            }
            
            foutLogMCP << "ee_str = " << getTaskStringName(ee) << std::endl;
            foutLogMCP << "FC = " << wrench.transpose() << std::endl;
        
        }
    }
    else foutLogMCP << "----------STABILITY CHECK SURFACE NOT PASSED----------" << std::endl;
    
}
*/

void Planner::retrieveContactPoses(Configuration q, Stance &sigma){
    for(int i = 0; i < sigma.getSize(); i++)
    {
        EndEffector ee = sigma.getContact(i)->getEndEffectorName();
        Eigen::Affine3d T = computeForwardKinematics(q, ee);
        sigma.getContact(i)->setPose(T.translation(), T.linear());
    }
}

