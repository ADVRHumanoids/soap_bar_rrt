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

static std::ofstream foutLogMCP("/home/paolo/catkin_ws/external/src/soap_bar_rrt/multi_contact_planning/PlanningData/logMCP.txt", std::ofstream::trunc);

//Planner::Planner(Configuration _qInit, std::vector<Eigen::Affine3d> _poseActiveEEsInit, std::vector<EndEffector> _activeEEsInit, std::vector<Eigen::Affine3d> _poseActiveEEsGoal, std::vector<EndEffector> _activeEEsGoal, Eigen::MatrixXd _pointCloud, Eigen::MatrixXd _pointNormals, std::vector<EndEffector> _allowedEEs, XBot::ModelInterface::Ptr _planner_model, GoalGenerator::Ptr _goal_generator){
Planner::Planner(Configuration _qInit, std::vector<Eigen::Affine3d> _poseActiveEEsInit, std::vector<EndEffector> _activeEEsInit, std::vector<Eigen::Affine3d> _poseActiveEEsGoal, std::vector<EndEffector> _activeEEsGoal, Eigen::MatrixXd _pointCloud, Eigen::MatrixXd _pointNormals, std::vector<EndEffector> _allowedEEs, XBot::ModelInterface::Ptr _planner_model, GoalGenerator::Ptr _goal_generator, XBot::Cartesian::Planning::ValidityCheckContext _vc_context){

	// set model, goal generator and cartesian interface
	planner_model = _planner_model;
	goal_generator = _goal_generator;
	ci = _goal_generator->getCartesianInterface();

	// set the environment representation
	pointCloud = _pointCloud;
	pointNormals = _pointNormals;

	// set number of dof of the robot of interest
	n_dof = planner_model->getJointNum();

	// set initial configuration
	qInit = _qInit;	

	// create initial stance 
	Eigen::Vector3d rCoMInit = computeCoM(qInit); 
	Eigen::MatrixXd rCInit;
	Eigen::MatrixXd nCInit;
	rCInit.resize(_activeEEsInit.size(), 3);
	nCInit.resize(_activeEEsInit.size(), 3);
	for(int i = 0; i < _activeEEsInit.size(); i++){
		rCInit.row(i) = _poseActiveEEsInit.at(i).translation().transpose();
		nCInit.row(i) = getNormalAtPoint(rCInit.row(i)).transpose();						
	}
	Eigen::MatrixXd FCInit;
	FCInit.resize(_activeEEsInit.size(), 3);	

	bool resCPL = computeCentroidalStatics(_activeEEsInit, rCoMInit, rCInit, nCInit, rCoMInit, rCInit, FCInit); 

	for(int i = 0; i < _activeEEsInit.size(); i++){
		Eigen::Affine3d p_i = _poseActiveEEsInit.at(i);
		Eigen::Vector3d F_i = FCInit.row(i).transpose();
		Eigen::Vector3d n_i = nCInit.row(i).transpose();
		Contact* c = new Contact(_activeEEsInit.at(i), p_i, F_i, n_i);
		sigmaInit.addContact(c);
	}
	
	// create goal stance 
	for(int i = 0; i < _activeEEsGoal.size(); i++){
		Eigen::Affine3d p_i = _poseActiveEEsGoal.at(i);
		Eigen::Vector3d F_i(0.0,0.0,0.0);
		Eigen::Vector3d n_i(0.0,0.0,0.0);
		Contact* c = new Contact(_activeEEsGoal.at(i), p_i, F_i, n_i); 		
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

	// needed for collision checking
	vc_context = _vc_context;

	//std::cout << "pointCloud size = " << pointCloud.rows() << " X " << pointCloud.cols() << std::endl;
	//std::cout << "pointNormals size = " << pointNormals.rows() << " X " << pointNormals.cols() << std::endl;
		
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

	foutLogMCP << "p_cur = " << p_cur.transpose() << std::endl;

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

	foutLogMCP << "pointsInWorkspace.size() = " << pointsInWorkspace.size() << std::endl;

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

Eigen::Vector3d Planner::pickPointInReachableWorkspace(EndEffector pk, Configuration q, Eigen::Vector3d rRand){

	double ReachableWorkspaceRadius = 0.0;
	if(pk == L_HAND || pk == R_HAND) ReachableWorkspaceRadius = WORKSPACE_RADIUS_HAND;
	else if(pk == L_FOOT || pk == R_FOOT) ReachableWorkspaceRadius = WORKSPACE_RADIUS_FOOT;

	Eigen::Affine3d T_cur = computeForwardKinematics(q, pk);
	Eigen::Vector3d p_cur = T_cur.translation();

	foutLogMCP << "p_cur = " << p_cur.transpose() << std::endl;

	std::vector<Eigen::Vector3d> pointsInWorkspace;
	for(int i = 0; i < pointCloud.rows(); i++){
		Eigen::Vector3d p = pointCloud.row(i).transpose();
		double d = euclideanDistance(p_cur, p);
		if(d < ReachableWorkspaceRadius) pointsInWorkspace.push_back(p);
	}

	foutLogMCP << "pointsInWorkspace.size() = " << pointsInWorkspace.size() << std::endl;

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
	
	return r;
}

Eigen::Vector3d Planner::pickRandomPointInReachableWorkspace(EndEffector pk, Configuration q){ // NOT USED

	double ReachableWorkspaceRadius = 0.0;
	if(pk == L_HAND || pk == R_HAND) ReachableWorkspaceRadius = WORKSPACE_RADIUS_HAND;
	else if(pk == L_FOOT || pk == R_FOOT) ReachableWorkspaceRadius = WORKSPACE_RADIUS_FOOT;

	Eigen::Affine3d T_cur = computeForwardKinematics(q, pk);
	Eigen::Vector3d p_cur = T_cur.translation();

	foutLogMCP << "p_cur = " << p_cur.transpose() << std::endl;

	std::vector<Eigen::Vector3d> pointsInWorkspace;
	for(int i = 0; i < pointCloud.rows(); i++){
		Eigen::Vector3d p = pointCloud.row(i).transpose();
		double d = euclideanDistance(p_cur, p);
		if(d < ReachableWorkspaceRadius) pointsInWorkspace.push_back(p);
	}

	foutLogMCP << "pointsInWorkspace.size() = " << pointsInWorkspace.size() << std::endl;

	int pointIndex = pointInWorkspaceDistribution(pointInWorkspaceGenerator) % pointsInWorkspace.size();	

	Eigen::Vector3d r = pointsInWorkspace.at(pointIndex);
	
	return r;
}

Eigen::Vector3d Planner::pickRandomPointInReachableWorkspace(Configuration q){ // NOT USED

	Eigen::Vector3d pFB = q.getFBPosition();
	std::vector<Eigen::Vector3d> pointsInWorkspace;
	for(int i = 0; i < pointCloud.rows(); i++){
		Eigen::Vector3d p = pointCloud.row(i).transpose();
		double d = euclideanDistance(pFB, p);
		if(d < WORKSPACE_RADIUS) pointsInWorkspace.push_back(p);
	}

	int pointIndex = pointInWorkspaceDistribution(pointInWorkspaceGenerator) % pointsInWorkspace.size();	

	Eigen::Vector3d r = pointsInWorkspace.at(pointIndex);
	
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
		//bool c2 = true; // at least one contact (after expansion)
		bool c2 = true; // at least 3 contact (after expansion)
		bool c3 = true; // one contact allowed only if it involves one of the feet
		bool c4 = true; // two contacts can not involve only hands
		bool c5 = true;	// the foot and the knee of the same leg can not be simultaneously in contact
		bool c6 = true; // non empty workspace for end effector pk if inactive at vnear
		bool c7 = true; // don't move an active ee that is already at the goal

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
		//if(activeEEsCand.size() == 0) c2 = false;
		if(activeEEsCand.size() < 3) c2 = false;
		// condition 3
		if(activeEEsCand.size() == 1 && activeEEsCand.at(0) != L_FOOT && activeEEsCand.at(0) != R_FOOT) c3 = false;
		// condition 4
		if(activeEEsCand.size() == 2 && ((activeEEsCand.at(0) == L_HAND && activeEEsCand.at(1) == R_HAND) || (activeEEsCand.at(0) == R_HAND && activeEEsCand.at(1) == L_HAND))) c4 = false;
		// condition 5
		bool pLF = false;
		bool pRF = false;
		bool pLK = false;
		bool pRK = false;
		for(int j = 0; j < activeEEsCand.size(); j++){
			if(activeEEsCand.at(j) == L_FOOT) pLF = true;
			if(activeEEsCand.at(j) == R_FOOT) pRF = true;
			if(activeEEsCand.at(j) == L_KNEE) pLK = true;
			if(activeEEsCand.at(j) == R_KNEE) pRK = true;
		}
		if((pLF && pLK) || (pRF && pRK)) c5 = false;	
		// condition 6
		if(!sigma.isActiveEndEffector(pk) && !nonEmptyReachableWorkspace(pk, q)) c6 = false;	
		// condition 7
		if(sigma.isActiveEndEffector(pk)){
			Eigen::Vector3d rGoal = sigmaGoal.retrieveContactPose(pk).translation();
			Eigen::Vector3d rCurr = sigma.retrieveContactPose(pk).translation();
			double d = euclideanDistance(rGoal, rCurr);
			if(d < 1e-4) c7 = false;	
		}
			 
		//bool allConditionsRespected = c1 && c2 && c3 && c4 && c5 && c6 && c7;
		bool allConditionsRespected = c1 && c2 && c7; 

		  
		/*	
		if(allConditionsRespected){
			Eigen::Vector3d rCoM = computeCoM(q);	
			double d = euclideanDistance(rCoM, r);

			admissibleVertexes.push_back(i);
			distAdmissibleVertexes.push_back(d);	 
		}
		*/

		/*
		if(allConditionsRespected){
			double dMin_k = std::numeric_limits<double>::max();
			for(int j = 0; j < sigma.getSize(); j++){
				Eigen::Affine3d T_j = sigma.getContact(j)->getPose();			
				Eigen::Vector3d r_j = T_j.translation();
				double d = euclideanDistance(r_j, r);
				if(d < dMin_k) dMin_k = d;
			}

			admissibleVertexes.push_back(i);
			distAdmissibleVertexes.push_back(dMin_k);			 
		}
		*/

		if(allConditionsRespected){
			//Eigen::Affine3d T_cur = computeForwardKinematics(q, pk);
			//Eigen::Vector3d p_cur = T_cur.translation();
			
			 
			Eigen::Affine3d T_cur;
			Eigen::Vector3d p_cur;
			if(sigma.isActiveEndEffector(pk)){
				T_cur = computeForwardKinematics(q, pk);
				p_cur = T_cur.translation();
			}
			else{
				int iParent = v->getParentIndex();
				v = tree->getVertex(iParent);
				q = v->getConfiguration();
				T_cur = computeForwardKinematics(q, pk);
				p_cur = T_cur.translation();	
			}	
			 

			double dMin_k = euclideanDistance(p_cur, r);

			admissibleVertexes.push_back(i);
			distAdmissibleVertexes.push_back(dMin_k);			 
		}		
	
	}

	foutLogMCP << "admissibleVertexes.size() = " << admissibleVertexes.size() << std::endl;

	if(admissibleVertexes.size() == 0) return -1;

	// RRT-like selection

	Eigen::VectorXd invDist(admissibleVertexes.size());
	for(int i = 0; i < admissibleVertexes.size(); i++){
		foutLogMCP << "distAdmissibleVertexes.at(i) = " << distAdmissibleVertexes.at(i) << std::endl; 
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
	foutLogMCP << "probVector = " << probVector.transpose() << std::endl; 

	double pr = exploitationDistribution(exploitationGenerator);
	foutLogMCP << "pr = " << pr << std::endl; 
	int index = 0;
	while(pr > probVector(index)) index++;
	foutLogMCP << "index = " << index << std::endl; 
	
	iMin = admissibleVertexes.at(index);
		
	// naive tree selection

	//int index = integerDistribution(integerGenerator) % admissibleVertexes.size();	
	//iMin = admissibleVertexes.at(index);

	return iMin;	

}

bool Planner::isGoalContactReachable(EndEffector pk, Configuration q){ // NOT USED	

	double ReachableWorkspaceRadius = 0.0;
	if(pk == L_HAND || pk == R_HAND) ReachableWorkspaceRadius = WORKSPACE_RADIUS_HAND;
	else if(pk == L_FOOT || pk == R_FOOT) ReachableWorkspaceRadius = WORKSPACE_RADIUS_FOOT;

	Eigen::Affine3d T_cur = computeForwardKinematics(q, pk);
	Eigen::Vector3d p_cur = T_cur.translation();
		
	Eigen::Vector3d p = sigmaGoal.retrieveContactPose(pk).translation();	
	
	double d = euclideanDistance(p_cur, p);
	if(d < ReachableWorkspaceRadius) return true;
	
	return false; 
}

std::string Planner::getTaskStringName(EndEffector ee){
	std::string ee_str;

	if(ee == L_HAND) ee_str = "TCP_L";
	else if(ee == R_HAND) ee_str = "TCP_R";
	else if(ee == L_FOOT) ee_str = "l_sole";
	else if(ee == R_FOOT) ee_str = "r_sole";
	else if(ee == HEAD) ee_str = "Head";
	else ee_str = "Com";

	return ee_str;
}
			

bool Planner::computeIKSolutionWithoutCoM(Stance sigma, Configuration &q, Configuration q_ref){

	//std::cout << "**************** GS INVOCATION WITHOUT COM *******************" << std::endl;
	
	// build references
	std::vector<std::string> active_tasks;
	std::vector<Eigen::Affine3d> ref_tasks;
	for(int i = 0; i < sigma.getSize(); i++){
    	EndEffector ee = sigma.getContact(i)->getEndEffectorName();
    	active_tasks.push_back(getTaskStringName(ee));
		ref_tasks.push_back(sigma.retrieveContactPose(ee));
	}

	// build postural 
	Eigen::VectorXd c_ref(n_dof);
	Eigen::Vector3d posFB = q_ref.getFBPosition();
	Eigen::Vector3d rotFB = q_ref.getFBOrientation();
	c_ref.segment(0,3) = posFB;
	c_ref.segment(3,3) = rotFB;
	c_ref.tail(n_dof-6) = q_ref.getJointValues();

	// set references
    std::vector<std::string> all_tasks;
    //all_tasks.push_back("Com");
    all_tasks.push_back("TCP_L");
    all_tasks.push_back("TCP_R");
    all_tasks.push_back("l_sole");
    all_tasks.push_back("r_sole");

	//ci->setActivationState(all_tasks[0], XBot::Cartesian::ActivationState::Disabled);
    //for(int i = 1; i < all_tasks.size(); i++){
    for(int i = 0; i < all_tasks.size(); i++){
        int index = -1;
        for(int j = 0; j < active_tasks.size(); j++) if(active_tasks[j] == all_tasks[i]) index = j;
  
        //if(index == -1) ci->setActivationState(all_tasks[i], XBot::Cartesian::ActivationState::Disabled);
        //if(index == -1) ci->getTask(all_tasks.at(i))->setLambda(0.1);
    	if(index == -1) ci->getTask(all_tasks.at(i))->setWeight(0.1*Eigen::MatrixXd::Identity(ci->getTask(all_tasks.at(i))->getWeight().rows(), ci->getTask(all_tasks.at(i))->getWeight().cols()));
        else{
            //ci->setActivationState(all_tasks[i], XBot::Cartesian::ActivationState::Enabled);
            //ci->getTask(all_tasks.at(i))->setLambda(1.0);  
            ci->getTask(all_tasks.at(i))->setWeight(Eigen::MatrixXd::Identity(ci->getTask(all_tasks.at(i))->getWeight().rows(), ci->getTask(all_tasks.at(i))->getWeight().cols()));
            ci->setPoseReference(all_tasks[i], ref_tasks[index]);
        }
    }
 
 	/* 
    // set collision checking 
    vc_context.planning_scene->acm.setEntry("RBall", "<octomap>", false);   
    vc_context.planning_scene->acm.setEntry("LBall", "<octomap>", false);     
    vc_context.planning_scene->acm.setEntry("LFoot", "<octomap>", false);    
    vc_context.planning_scene->acm.setEntry("RFoot", "<octomap>", false);    
    for (auto i : active_tasks) {
        if (i == "Com") continue;
        else if (i == "TCP_R") vc_context.planning_scene->acm.setEntry("RBall", "<octomap>", true);   
        else if (i == "TCP_L") vc_context.planning_scene->acm.setEntry("LBall", "<octomap>", true);     
        else if (i == "l_sole") vc_context.planning_scene->acm.setEntry("LFoot", "<octomap>", true);    
        else if (i == "r_sole") vc_context.planning_scene->acm.setEntry("RFoot", "<octomap>", true);    
    } 
	*/ 

	/*
	// set references 
    std::vector<std::string> all_tasks;
    all_tasks.push_back("Com");
    all_tasks.push_back("TCP_L");
    all_tasks.push_back("TCP_R");
    all_tasks.push_back("l_sole");
    all_tasks.push_back("r_sole");
    for(int i = 0; i < all_tasks.size(); i++) ci->setActivationState(all_tasks.at(i), XBot::Cartesian::ActivationState::Disabled);
    Eigen::Affine3d T_ref;
    for(int i = 0; i < active_tasks.size(); i++){
        ci->setActivationState(active_tasks.at(i), XBot::Cartesian::ActivationState::Enabled);
        T_ref = ref_tasks.at(i);
        ci->setPoseReference(active_tasks.at(i), T_ref);
    }
	*/

    // set postural
    XBot::JointNameMap jmap;
    planner_model->eigenToMap(c_ref, jmap);
    //ci->setReferencePosture(jmap); 

    // search IK solution
    double time_budget = GOAL_SAMPLER_TIME_BUDGET;
    Eigen::VectorXd c;
    if(!goal_generator->sample(c, time_budget)) return false;
    else{
    	q.setFBPosition(c.segment(0,3));
        q.setFBOrientation(c.segment(3,3));
        q.setJointValues(c.tail(n_dof-6));
        return true;
    }

}

bool Planner::computeIKSolutionWithCoM(Stance sigma, Eigen::Vector3d rCoM, Configuration &q, Configuration q_ref){

	//std::cout << "**************** GS INVOCATION WITH COM *******************" << std::endl;
	
	// build references
	std::vector<std::string> active_tasks;
	std::vector<Eigen::Affine3d> ref_tasks;
	active_tasks.push_back("Com");
	Eigen::Affine3d T_CoM_ref;
    T_CoM_ref.translation() = rCoM;
    T_CoM_ref.linear() = Eigen::Matrix3d::Identity(3,3);
    ref_tasks.push_back(T_CoM_ref);
    for(int i = 0; i < sigma.getSize(); i++){
    	EndEffector ee = sigma.getContact(i)->getEndEffectorName();
    	active_tasks.push_back(getTaskStringName(ee));
		ref_tasks.push_back(sigma.retrieveContactPose(ee));
	}

	// build postural
	Eigen::VectorXd c_ref(n_dof);
	Eigen::Vector3d posFB = q_ref.getFBPosition();
	Eigen::Vector3d rotFB = q_ref.getFBOrientation();
	c_ref.segment(0,3) = posFB;
	c_ref.segment(3,3) = rotFB;
	c_ref.tail(n_dof-6) = q_ref.getJointValues();

	// set references
    std::vector<std::string> all_tasks;
    all_tasks.push_back("Com");
    all_tasks.push_back("TCP_L");
    all_tasks.push_back("TCP_R");
    all_tasks.push_back("l_sole");
    all_tasks.push_back("r_sole");
    for(int i = 0; i < all_tasks.size(); i++){
        int index = -1;
        for(int j = 0; j < active_tasks.size(); j++) if(active_tasks[j] == all_tasks[i]) index = j;
    
        //if(index == -1) ci->setActivationState(all_tasks[i], XBot::Cartesian::ActivationState::Disabled);
        //if(index == -1) ci->getTask(all_tasks.at(i))->setLambda(0.1);
        if(index == -1) ci->getTask(all_tasks.at(i))->setWeight(0.1*Eigen::MatrixXd::Identity(ci->getTask(all_tasks.at(i))->getWeight().rows(), ci->getTask(all_tasks.at(i))->getWeight().cols()));
    	else{
            //ci->setActivationState(all_tasks[i], XBot::Cartesian::ActivationState::Enabled);
            //ci->getTask(all_tasks.at(i))->setLambda(1.0);
            ci->getTask(all_tasks.at(i))->setWeight(Eigen::MatrixXd::Identity(ci->getTask(all_tasks.at(i))->getWeight().rows(), ci->getTask(all_tasks.at(i))->getWeight().cols()));
            ci->setPoseReference(all_tasks[i], ref_tasks[index]);
        }
    }

    //ci->getTask(all_tasks.at(0))->setWeight(0.1*Eigen::MatrixXd::Identity(ci->getTask(all_tasks.at(0))->getWeight().rows(), ci->getTask(all_tasks.at(0))->getWeight().cols()));
    //ci->setActivationState(all_tasks[0], XBot::Cartesian::ActivationState::Disabled);

    /*
    // set collision checking 
    vc_context.planning_scene->acm.setEntry("RBall", "<octomap>", false);   
    vc_context.planning_scene->acm.setEntry("LBall", "<octomap>", false);     
    vc_context.planning_scene->acm.setEntry("LFoot", "<octomap>", false);    
    vc_context.planning_scene->acm.setEntry("RFoot", "<octomap>", false);    
    for (auto i : active_tasks) {
        if (i == "Com") continue;
        else if (i == "TCP_R") vc_context.planning_scene->acm.setEntry("RBall", "<octomap>", true);   
        else if (i == "TCP_L") vc_context.planning_scene->acm.setEntry("LBall", "<octomap>", true);     
        else if (i == "l_sole") vc_context.planning_scene->acm.setEntry("LFoot", "<octomap>", true);    
        else if (i == "r_sole") vc_context.planning_scene->acm.setEntry("RFoot", "<octomap>", true);    
    } 
	*/

	/*
	// set references 
    std::vector<std::string> all_tasks;
    all_tasks.push_back("Com");
    all_tasks.push_back("TCP_L");
    all_tasks.push_back("TCP_R");
    all_tasks.push_back("l_sole");
    all_tasks.push_back("r_sole");
    for(int i = 0; i < all_tasks.size(); i++) ci->setActivationState(all_tasks.at(i), XBot::Cartesian::ActivationState::Disabled);
    Eigen::Affine3d T_ref;
    for(int i = 0; i < active_tasks.size(); i++){
        ci->setActivationState(active_tasks.at(i), XBot::Cartesian::ActivationState::Enabled);
        T_ref = ref_tasks.at(i);
        ci->setPoseReference(active_tasks.at(i), T_ref);
    }
	*/

    // set postural
    XBot::JointNameMap jmap;
    planner_model->eigenToMap(c_ref, jmap);
    //ci->setReferencePosture(jmap);  

    // build initial guess
	Eigen::VectorXd c_init(n_dof);
	Eigen::Vector3d posFB_init = q.getFBPosition();
	Eigen::Vector3d rotFB_init = q.getFBOrientation();
	c_init.segment(0,3) = posFB_init;
	c_init.segment(3,3) = rotFB_init;
	c_init.tail(n_dof-6) = q.getJointValues();

    // search IK solution
    double time_budget = GOAL_SAMPLER_TIME_BUDGET_COM;
    Eigen::VectorXd c;
    //if(!goal_generator->sample(c, time_budget)) return false;
    if(!goal_generator->sample(c, time_budget, c_init)) return false;
    else{
        q.setFBPosition(c.segment(0,3));
        q.setFBOrientation(c.segment(3,3));
        q.setJointValues(c.tail(n_dof-6));
        return true;
    }

}

bool Planner::retrieveSolution(std::vector<Stance> &sigmaList, std::vector<Configuration> &qList){
	int iEnd = tree->getSize() - 1;
	
	Vertex* v = tree->getVertex(iEnd);
	bool solutionFound = isGoalStance(v);

	if(!solutionFound){
		sigmaList.clear();
		qList.clear();
		return false;
	}
		
	sigmaList.push_back(v->getStance());
	qList.push_back(v->getConfiguration());		
	int parentIndex = v->getParentIndex();	
	
	while(parentIndex > -1){
		v = tree->getVertex(parentIndex);
		sigmaList.push_back(v->getStance());
		qList.push_back(v->getConfiguration());		
		parentIndex = v->getParentIndex();	
	}

	std::reverse(sigmaList.begin(), sigmaList.end());	
	std::reverse(qList.begin(), qList.end());	
	
	return true;	
} 

bool Planner::retrieveSolution1stStage(std::vector<Stance> &sigmaList, std::vector<Configuration> &qList){
	int iEnd = tree->getSize() - 1;
	
	Vertex* v = tree->getVertex(iEnd);
	bool solutionFound = isGoalStance(v);

	if(!solutionFound){
		sigmaList.clear();
		qList.clear();
		return false;
	}
		
	sigmaList.push_back(v->getStance());
	qList.push_back(v->getConfiguration());		
	int parentIndex = v->getParentIndex();	
	
	while(parentIndex > -1){
		v = tree->getVertex(parentIndex);
		sigmaList.push_back(v->getStance());
		qList.push_back(v->getConfiguration());		
		parentIndex = v->getParentIndex();	
	}

	std::reverse(sigmaList.begin(), sigmaList.end());	
	std::reverse(qList.begin(), qList.end());	
	
	return true;	
} 

bool Planner::retrieveSolution2ndStage(std::vector<Stance> &sigmaList, std::vector<Configuration> &qList){
	/*
	if(sol_len > sigmaList2ndStage.size()){
		sigmaList.clear();
		qList.clear();
		return false;	
	}
	
	for(int i = 0; i < sigmaList2ndStage.size(); i++) sigmaList.push_back(sigmaList2ndStage.at(i));
	for(int i = 0; i < qList2ndStage.size(); i++) qList.push_back(qList2ndStage.at(i));

	return true;		
	*/

	for(int i = 0; i < sigmaList2ndStage.size(); i++) sigmaList.push_back(sigmaList2ndStage.at(i));
	for(int i = 0; i < qList2ndStage.size(); i++) qList.push_back(qList2ndStage.at(i));
	if(sol_len > sigmaList2ndStage.size()) return false;	
	return true;	
}

bool Planner::computeCentroidalStatics(std::vector<EndEffector> activeEEsDes, Eigen::Vector3d rCoMdes, Eigen::MatrixXd rCdes, Eigen::MatrixXd nCdes, Eigen::Vector3d &rCoM, Eigen::MatrixXd &rC, Eigen::MatrixXd &FC){

	std::cout << "**************** CPL INVOCATION *******************" << std::endl;

	double robot_mass = ROBOT_MASS;
    double g = GRAVITY;
    double mu = MU_FRICTION;
	double F_thres = FORCE_THRES;
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
		//cpl->SetForceThreshold(name, F_thres);
		if(activeEEsDes.at(i) == L_HAND || activeEEsDes.at(i) == R_HAND) cpl->SetForceThreshold(name, FORCE_THRES_HAND);
		if(activeEEsDes.at(i) == L_FOOT || activeEEsDes.at(i) == R_FOOT) cpl->SetForceThreshold(name, FORCE_THRES_FOOT); 
	}	
	 
	cpl->SetCoMRef(rCoMdes);
	cpl->SetCoMWeight(W_CoM);
        
    sol = cpl->Solve();
    
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
		
		foutLogMCP << "rError = " << rError.transpose() << std::endl;
		foutLogMCP << "nError = " << nError.transpose() << std::endl; 
		foutLogMCP << "-F.dot(n) = " << -F.dot(n) << std::endl;		
		foutLogMCP << "(F-(n.dot(F))*n).norm() - mu*(F.dot(n)) = " << (F-(n.dot(F))*n).norm() - mu*(F.dot(n)) << std::endl;		
		
		for(int j = 0; j < 3; j++) if(abs(rError(j)) > 1e-4 || abs(nError(j)) > 1e-4) return false;
		if( -F.dot(n) > 1e-4 ) return false; 
		if( (F-(n.dot(F))*n).norm() - mu*(F.dot(n)) > 1e-4 ) return false;
	
		rC.row(i) = r.transpose();
		FC.row(i) = F.transpose();   
		i++;
    }

	F_sumError = F_sum + Eigen::Vector3d(0.0, 0.0, robot_mass*g);	
	Torque_sumError = Torque_sum + Eigen::Vector3d(0.0, 0.0, 0.0);	

	foutLogMCP << "F_sumError = " << F_sumError.transpose() << std::endl; 
	foutLogMCP << "Torque_sumError = " << Torque_sumError.transpose() << std::endl; 
			
	for(int j = 0; j < 3; j++) if(abs(F_sumError(j)) > 1e-4 || abs(Torque_sumError(j)) > 1e-4) return false;

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

Eigen::Vector3d Planner::generateRandomCoM(Configuration qNear){
	Eigen::Vector3d rCoM = computeCoM(qNear); 
	Eigen::Vector3d rRand = pickRandomPoint();

	double eta = 0.30;

	Eigen::Vector3d delta = rRand - rCoM;
	double delta_norm = delta.norm();

	Eigen::Vector3d rCoMRef = rCoM + (eta/delta_norm) * delta;

	return rCoMRef;
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
	std::string link_COM = "Com";        
    ci->getCurrentPose(link_COM, T_COM);
    
	Eigen::Vector3d rCoM = T_COM.translation();
	
	return rCoM;
}

Eigen::Affine3d Planner::computeForwardKinematics(Configuration q, EndEffector ee){
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

Eigen::Matrix3d Planner::generateRandomRotationAroundAxis(Eigen::Vector3d axis){
	Eigen::Matrix3d rot;
    Eigen::Vector3d r1, r2, r3; 
    r3 = axis;

    double r1_x, r1_y, r1_z;

    if(abs(r3(0)) > 1e-3){
        r1_y = rotationDistribution(rotationGenerator);
        r1_z = rotationDistribution(rotationGenerator);
        r1_x = -(r3(1)*r1_y + r3(2)*r1_z) / r3(0);             
    }
    else if(abs(r3(1)) > 1e-3){
        r1_x = rotationDistribution(rotationGenerator);
        r1_z = rotationDistribution(rotationGenerator);
        r1_y = -(r3(0)*r1_x + r3(2)*r1_z) / r3(1);             
    }
    else{
        r1_x = rotationDistribution(rotationGenerator);
        r1_y = rotationDistribution(rotationGenerator);
        r1_z = -(r3(0)*r1_x + r3(1)*r1_y) / r3(2);             
    }

    r1 << r1_x, r1_y, r1_z;
    double r1_norm = r1.norm();
    r1 = 1.0/r1_norm * r1;

    r2 = r1.cross(r3);
    double r2_norm = r2.norm();
    r2 = 1.0/r2_norm * r2;

    rot.col(0) = r1.transpose();
    rot.col(1) = r2.transpose();
    rot.col(2) = r3.transpose();
    
	return rot;    
}

Eigen::Matrix3d Planner::generateRotationAroundAxis(EndEffector pk, Eigen::Vector3d axis){
	Eigen::Matrix3d rot;

	bool vertical = false;
	Eigen::Vector3d aux = axis - Eigen::Vector3d(0.0, 0.0, 1.0); 
    if(abs(aux(0)) < 1e-3 && abs(aux(1)) < 1e-3 && abs(aux(2)) < 1e-3) vertical = true;

    if(pk == L_HAND || pk == R_HAND){
    	if(vertical){
    		rot << 	0.0, 1.0, 0.0,
					1.0, 0.0, 0.0,
					0.0, 0.0, -1.0;
    	}
    	else{
    		rot << 	0.0, 0.0, 1.0,
					1.0, 0.0, 0.0,
					0.0, 1.0, 0.0;
    	}		
    }
	else{
    	if(vertical){
    		rot << 	1.0, 0.0, 0.0,
					0.0, 1.0, 0.0,
					0.0, 0.0, 1.0;
    	}
    	else{
    		rot << 	0.0, 0.0, -1.0,
					0.0, 1.0, 0.0,
					1.0, 0.0, 0.0;
    	}		
    }

	return rot;    
}

Eigen::Vector3d Planner::getNormalAtContact(EndEffector ee, Eigen::Matrix3d rot){
	if(ee == L_HAND || ee == R_HAND) return -rot.col(2);
	return rot.col(2); // this is ok for feet, what about knees???
}

bool Planner::similarityTest(Configuration qNew){

	Eigen::VectorXd cNew(n_dof);
	cNew.segment(0,3) = qNew.getFBPosition();
	cNew.segment(3,3) = qNew.getFBOrientation();
    cNew.tail(n_dof-6) = qNew.getJointValues();

    Vertex* v;
    Configuration q; 
	Eigen::VectorXd c(n_dof);
	Eigen::VectorXd diff(n_dof);
	for(int i = 0; i < tree->getSize(); i++){
		v = tree->getVertex(i);
		q = v->getConfiguration();

		c.segment(0,3) = q.getFBPosition();
		c.segment(3,3) = q.getFBOrientation();
	    c.tail(n_dof-6) = q.getJointValues();

	    diff = cNew - c;

	    bool similar = true;
	    for(int j = 0; j < n_dof; j++) if(abs(diff(j) > 1e-03)) similar = false;

	    if(similar) return true;	
	}

	return false;
}

bool Planner::similarityTest(Configuration qNew, Stance sigmaNew){

	Eigen::VectorXd cNew(n_dof);
	cNew.segment(0,3) = qNew.getFBPosition();
	cNew.segment(3,3) = qNew.getFBOrientation();
    cNew.tail(n_dof-6) = qNew.getJointValues();

    Vertex* v;
    Configuration q; 
    Stance sigma;
	Eigen::VectorXd c(n_dof);
	Eigen::VectorXd diff(n_dof);
	for(int i = 0; i < tree->getSize(); i++){
		v = tree->getVertex(i);
		q = v->getConfiguration();
		sigma = v->getStance();	

		if(sigmaNew.getSize() == sigma.getSize()){
			c.segment(0,3) = q.getFBPosition();
			c.segment(3,3) = q.getFBOrientation();
		    c.tail(n_dof-6) = q.getJointValues();

		    diff = cNew - c;

		    bool similar = true;
		    for(int j = 0; j < n_dof; j++) if(abs(diff(j) > 1e-03)) similar = false;

		    if(similar) return true;
		}	
	}

	return false;
}

bool Planner::similarityTest(Stance sigmaNew){

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

void Planner::updateEndEffectorsList(Configuration qNew, Stance sigmaNew){

	std::vector<EndEffector> endEffectorsListAux;

	for(int i = 0; i < endEffectorsList.size(); i++){
		EndEffector ee = endEffectorsList.at(i);

		endEffectorsListAux.push_back(ee);

		if(sigmaNew.isActiveEndEffector(ee) && sigmaGoal.isActiveEndEffector(ee)){
			Eigen::Vector3d p = computeForwardKinematics(qNew, ee).translation();
			Eigen::Vector3d pGoal = sigmaGoal.retrieveContactPose(ee).translation();

			double d = euclideanDistance(p, pGoal);
			if(d < GOAL_TOLERANCE) endEffectorsListAux.pop_back(); 
		}
	}

	endEffectorsList.clear();
	for(int i = 0; i < endEffectorsListAux.size(); i++){
		endEffectorsList.push_back(endEffectorsListAux.at(i));
	}	
}

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
	
	while(j < MAX_ITERATIONS && !solutionFound){

		foutLogMCP << "j = " << j << std::endl;
		std::cout << "j = " << j << std::endl;
		
		double pr = exploitationDistribution(exploitationGenerator);
		if(pr < EXPLOITATION_RATE){
			// exploitation
			// pick a random contact from sigmaGoal and retrieve the corresponding ee and position
			foutLogMCP << "+++++++++++++++++++++++++++++ EXPLOITATION +++++++++++++++++++++++++++++" << std::endl;
			Contact* c = pickRandomContactFromGoalStance();
			pk = c->getEndEffectorName(); 
			rRand = c->getPose().translation();
		}
		else{
			// exploration
			// pick a random ee and a random point
			foutLogMCP << "+++++++++++++++++++++++++++++ EXPLORATION +++++++++++++++++++++++++++++" << std::endl;
			pk = pickRandomEndEffector(); 
			rRand = pickRandomPoint();
		}

		int iNear = findNearestVertexIndex(pk, rRand);   
		foutLogMCP << "iNear = " << iNear << std::endl; 
		foutLogMCP << "pk = " << pk << std::endl; 

		if(iNear != -1){ // a vertex of the tree is available for expansion using ee pk (based on the condition specified in function findNearestVertexIndex)
			vNear = tree->getVertex(iNear);
			Stance sigmaNear = vNear->getStance();
			Configuration qNear = vNear->getConfiguration();
			vNear->increaseNumExpansionAttempts();
			
			std::vector<EndEffector> activeEEsNear = sigmaNear.retrieveActiveEndEffectors();
			foutLogMCP << "activeEEsNear.size() = " << activeEEsNear.size() << std::endl; 
			for(int z = 0; z < activeEEsNear.size(); z++) foutLogMCP << activeEEsNear.at(z) << std::endl;	

			std::vector<EndEffector> activeEEsDes;
			Eigen::Affine3d T_k;
			Eigen::Vector3d n_k;	
			if(sigmaNear.isActiveEndEffector(pk)){
				foutLogMCP << "REMOVING A CONTACT" << std::endl;
				for(int i = 0; i < activeEEsNear.size(); i++) if(activeEEsNear.at(i) != pk) activeEEsDes.push_back(activeEEsNear.at(i));						
			}
			else{
				foutLogMCP << "ADDING A CONTACT" << std::endl;					
				for(int i = 0; i < activeEEsNear.size(); i++) activeEEsDes.push_back(activeEEsNear.at(i));
				activeEEsDes.push_back(pk);
				int pointIndex;  
				T_k.translation() = pickPointInReachableWorkspace(pk, qNear, rRand, pointIndex);	
				T_k.linear() = Eigen::Matrix3d::Identity(3,3);
				n_k = getNormalAtPointByIndex(pointIndex);							
			}	

			foutLogMCP << "activeEEsDes.size() = " << activeEEsDes.size() << std::endl; 

			Stance sigmaNew;
			Eigen::Vector3d F_i(0.0, 0.0, 0.0);
			Eigen::Affine3d T_i;
			Eigen::Vector3d n_i;
			for(int i = 0; i < activeEEsDes.size(); i++){
				if(sigmaNear.isActiveEndEffector(activeEEsDes.at(i))){
					T_i = sigmaNear.retrieveContactPose(activeEEsDes.at(i));
					n_i = sigmaNear.retrieveContactNormal(activeEEsDes.at(i));
				}
				else{
					T_i = T_k;
					n_i = n_k;	
				}
    			
    			Contact* c = new Contact(activeEEsDes.at(i), T_i, F_i, n_i);
				sigmaNew.addContact(c);
			}

			bool similar = similarityTest(sigmaNew);

			if(!similar){ 
				Configuration qNew;
				bool resIK = computeIKSolutionWithoutCoM(sigmaNew, qNew, qNear);

				/* 
				bool resIK;
				if(sigmaNew.getSize() == 4) resIK = computeIKSolutionWithoutCoM(sigmaNew, qNew, qNear);
				else{
					resIK = true;
					qNew = qNear;
				}
				*/ 
				
				if(resIK) foutLogMCP << "--------------- GS SUCCESS ---------------" << std::endl;
				else foutLogMCP << "--------------- GS FAIL ---------------" << std::endl;

				if(resIK){
					foutLogMCP << "L_HAND = " << computeForwardKinematics(qNew, L_HAND).translation().transpose() << std::endl;
					foutLogMCP << "R_HAND = " << computeForwardKinematics(qNew, R_HAND).translation().transpose() << std::endl;
					foutLogMCP << "L_FOOT = " << computeForwardKinematics(qNew, L_FOOT).translation().transpose() << std::endl;
					foutLogMCP << "R_FOOT = " << computeForwardKinematics(qNew, R_FOOT).translation().transpose() << std::endl;
						 

					//bool similar = similarityTest(qNew, sigmaNew);

					//if(!similar){
						vNew = new Vertex(sigmaNew, qNew, iNear);  
						tree->addVertex(vNew);
									
						foutLogMCP << "VERTEX # = " << tree->getSize()-1 << std::endl;
						Eigen::VectorXd c(n_dof);
						c.segment(0,3) = qNew.getFBPosition();
						c.segment(3,3) = qNew.getFBOrientation();
						c.tail(n_dof-6) = qNew.getJointValues();
						for(int z = 0; z < c.rows(); z++) foutLogMCP << c(z) << ", ";
						foutLogMCP << ";" << std::endl;								

						solutionFound = isGoalStance(vNew);		
					//}
					 
					
					
				}
			}

		}		
		
		j++;
		
	}

	std::cout << "iters = " << j << std::endl;
	std::cout << "tree size = " << tree->getSize() << std::endl;
	
}

void Planner::run1stStage(){

	foutLogMCP << "********************************* 1ST STAGE *********************************" << std::endl;

	tree->clear();
	Vertex* vInit = new Vertex(sigmaInit, qInit, -1);
	tree->addVertex(vInit);

	int j = 0;
	bool solutionFound = false;

	EndEffector pk;
	Eigen::Vector3d rRand;

	Vertex* vNear;
	Vertex* vNew;
	
	while(j < MAX_ITERATIONS && !solutionFound){

		foutLogMCP << "j = " << j << std::endl;
		
		double pr = exploitationDistribution(exploitationGenerator);
		if(pr < EXPLOITATION_RATE){
			// exploitation
			// pick a random contact from sigmaGoal and retrieve the corresponding ee and position
			foutLogMCP << "+++++++++++++++++++++++++++++ EXPLOITATION +++++++++++++++++++++++++++++" << std::endl;
			Contact* c = pickRandomContactFromGoalStance();
			pk = c->getEndEffectorName(); 
			rRand = c->getPose().translation();
		}
		else{
			// exploration
			// pick a random ee and a random point
			foutLogMCP << "+++++++++++++++++++++++++++++ EXPLORATION +++++++++++++++++++++++++++++" << std::endl;
			pk = pickRandomEndEffector(); 
			rRand = pickRandomPoint();
		}

		int iNear = findNearestVertexIndex(pk, rRand);   
		foutLogMCP << "iNear = " << iNear << std::endl; 
		foutLogMCP << "pk = " << pk << std::endl; 

		if(iNear != -1){ // a vertex of the tree is available for expansion using ee pk (based on the condition specified in function findNearestVertexIndex)
			vNear = tree->getVertex(iNear);
			Stance sigmaNear = vNear->getStance();
			Configuration qNear = vNear->getConfiguration();
			vNear->increaseNumExpansionAttempts();
			
			std::vector<EndEffector> activeEEsNear = sigmaNear.retrieveActiveEndEffectors();
			foutLogMCP << "activeEEsNear.size() = " << activeEEsNear.size() << std::endl; 
			for(int z = 0; z < activeEEsNear.size(); z++) foutLogMCP << activeEEsNear.at(z) << std::endl;	

			std::vector<EndEffector> activeEEsDes;
			Eigen::Affine3d T_k;
			Eigen::Vector3d n_k;	
			if(sigmaNear.isActiveEndEffector(pk)){
				foutLogMCP << "REMOVING A CONTACT" << std::endl;
				for(int i = 0; i < activeEEsNear.size(); i++) if(activeEEsNear.at(i) != pk) activeEEsDes.push_back(activeEEsNear.at(i));						
			}
			else{
				foutLogMCP << "ADDING A CONTACT" << std::endl;					
				for(int i = 0; i < activeEEsNear.size(); i++) activeEEsDes.push_back(activeEEsNear.at(i));
				activeEEsDes.push_back(pk);
				int pointIndex;  
				T_k.translation() = pickPointInReachableWorkspace(pk, qNear, rRand, pointIndex);	
				T_k.linear() = Eigen::Matrix3d::Identity(3,3);
				n_k = getNormalAtPointByIndex(pointIndex);							
			}	

			foutLogMCP << "activeEEsDes.size() = " << activeEEsDes.size() << std::endl; 

			Stance sigmaNew;
			Eigen::Vector3d F_i(0.0, 0.0, 0.0);
			Eigen::Affine3d T_i;
			Eigen::Vector3d n_i;
			for(int i = 0; i < activeEEsDes.size(); i++){
				if(sigmaNear.isActiveEndEffector(activeEEsDes.at(i))){
					T_i = sigmaNear.retrieveContactPose(activeEEsDes.at(i));
					n_i = sigmaNear.retrieveContactNormal(activeEEsDes.at(i));
				}
				else{
					T_i = T_k;
					n_i = n_k;	
				}
    			
    			Contact* c = new Contact(activeEEsDes.at(i), T_i, F_i, n_i);
				sigmaNew.addContact(c);
			}

			Configuration qNew;
			bool resIK = computeIKSolutionWithoutCoM(sigmaNew, qNew, qNear);
			
			/*
			bool resIK;
			if(sigmaNew.getSize() == 4) resIK = computeIKSolutionWithoutCoM(sigmaNew, qNew, qNear);
			else{
				resIK = true;
				qNew = qNear;
			}
			*/	
			
			if(resIK) foutLogMCP << "--------------- GS SUCCESS ---------------" << std::endl;
			else foutLogMCP << "--------------- GS FAIL ---------------" << std::endl;

			if(resIK){
				foutLogMCP << "L_HAND = " << computeForwardKinematics(qNew, L_HAND).translation().transpose() << std::endl;
				foutLogMCP << "R_HAND = " << computeForwardKinematics(qNew, R_HAND).translation().transpose() << std::endl;
				foutLogMCP << "L_FOOT = " << computeForwardKinematics(qNew, L_FOOT).translation().transpose() << std::endl;
				foutLogMCP << "R_FOOT = " << computeForwardKinematics(qNew, R_FOOT).translation().transpose() << std::endl;
					 
				//bool similar = similarityTest(qNew);
				//bool similar = false;
				bool similar = similarityTest(qNew, sigmaNew);

				if(!similar){
					vNew = new Vertex(sigmaNew, qNew, iNear);  
					tree->addVertex(vNew);
								
					foutLogMCP << "VERTEX # = " << tree->getSize()-1 << std::endl;
					Eigen::VectorXd c(n_dof);
					c.segment(0,3) = qNew.getFBPosition();
					c.segment(3,3) = qNew.getFBOrientation();
					c.tail(n_dof-6) = qNew.getJointValues();
					for(int z = 0; z < c.rows(); z++) foutLogMCP << c(z) << ", ";
					foutLogMCP << ";" << std::endl;								

					solutionFound = isGoalStance(vNew);	
				}
			}

		}		
		
		j++;
		
	}

	std::cout << "iters = " << j << std::endl;
	std::cout << "tree size = " << tree->getSize() << std::endl;
	
}

/* 
void Planner::run2ndStage(std::vector<Stance> sigmaList, std::vector<Configuration> qList){

	foutLogMCP << "********************************* 2ND STAGE *********************************" << std::endl;

	Eigen::VectorXi succ(sigmaList.size());
	succ.setZero();
	succ(0) = 1;

	sol_len = sigmaList.size();

	sigmaList2ndStage.clear();
	qList2ndStage.clear();

	sigmaList2ndStage.push_back(sigmaList.at(0));
	qList2ndStage.push_back(qList.at(0));
	
	Stance sigma;
	Configuration q;

	for(int i = 1; i < sigmaList.size(); i++){
		
		foutLogMCP << "+++++ i = " << i << std::endl;

		sigma = sigmaList.at(i);
		q = qList.at(i);	

		Eigen::Vector3d rCoMdes = computeCoM(q);
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//rCoMdes = (computeForwardKinematics(q, L_FOOT).translation() + computeForwardKinematics(q, R_FOOT).translation()) / 2.0;
		//rCoMdes(2) = COM_REF_HEIGHT; 
		//rCoMdes(2) -= 0.10; 
		//if(computeForwardKinematics(q, L_HAND).translation())
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		std::vector<EndEffector> activeEEsDes = sigma.retrieveActiveEndEffectors();
		std::cout << "activeEEsDes.size() = " << activeEEsDes.size() << std::endl;
		Eigen::MatrixXd rCdes(activeEEsDes.size(), 3);
		Eigen::MatrixXd nCdes(activeEEsDes.size(), 3);
		Eigen::Vector3d rCoM;
		Eigen::MatrixXd rC(activeEEsDes.size(), 3);
		Eigen::MatrixXd FC(activeEEsDes.size(), 3);

		for(int j = 0; j < activeEEsDes.size(); j++){
			rCdes.row(j) = sigma.getContact(j)->getPose().translation().transpose();
			nCdes.row(j) = sigma.getContact(j)->getNormal().transpose();
		}

		foutLogMCP << "rCdes" << std::endl;	
		foutLogMCP << rCdes << std::endl;	
		foutLogMCP << "nCdes" << std::endl;	
		foutLogMCP << nCdes << std::endl;	
		foutLogMCP << "rCoMdes" << std::endl;	
		foutLogMCP << rCoMdes.transpose() << std::endl;
				
		bool resCPL = computeCentroidalStatics(activeEEsDes, rCoMdes, rCdes, nCdes, rCoM, rC, FC); 

		foutLogMCP << "rC" << std::endl;	
		foutLogMCP << rC << std::endl;	
		foutLogMCP << "FC" << std::endl;	
		foutLogMCP << FC << std::endl;	
		foutLogMCP << "rCoM" << std::endl;	
		foutLogMCP << rCoM.transpose() << std::endl;	

		foutLogMCP << "eCoMnorm = " << (rCoMdes - rCoM).norm() << std::endl;

				
		if(resCPL) foutLogMCP << "--------------- CPL SUCCESS ---------------" << std::endl;
		else foutLogMCP << "--------------- CPL FAIL ---------------" << std::endl;	

		if(resCPL){
			Stance sigmaBar;
			for(int j = 0; j < sigma.getSize(); j++){
				Contact* c_j_bar = sigma.getContact(j);
				Eigen::Vector3d F_j = FC.row(j).transpose();
				Contact* c_j = new Contact(c_j_bar->getEndEffectorName(), c_j_bar->getPose(), F_j, c_j_bar->getNormal());
				sigmaBar.addContact(c_j);	
			}
			
			Eigen::Vector3d rCoMError = rCoMdes - rCoM; 
			foutLogMCP << "rCoMErrorA = " << rCoMError.transpose() << std::endl;
			
			bool resIK_CoM = computeIKSolutionWithCoM(sigmaBar, rCoM, q, qList.at(i-1));
			
			foutLogMCP << "L_HAND = " << computeForwardKinematics(q, L_HAND).translation().transpose() << std::endl;
			foutLogMCP << "R_HAND = " << computeForwardKinematics(q, R_HAND).translation().transpose() << std::endl;
			foutLogMCP << "L_FOOT = " << computeForwardKinematics(q, L_FOOT).translation().transpose() << std::endl;
			foutLogMCP << "R_FOOT = " << computeForwardKinematics(q, R_FOOT).translation().transpose() << std::endl;
			Eigen::Vector3d rCoMErrorB = computeCoM(q) - rCoM; 
			foutLogMCP << "rCoMErrorB = " << rCoMErrorB.transpose() << std::endl;

			if(resIK_CoM) foutLogMCP << "--------------- GS COM SUCCESS ---------------" << std::endl;
			else foutLogMCP << "--------------- GS COM FAIL ---------------" << std::endl;
			
			if(resIK_CoM){
				sigmaList2ndStage.push_back(sigmaBar);
				qList2ndStage.push_back(q);
				
				foutLogMCP << "CONFIGURATION # = " << i << std::endl;
				Eigen::VectorXd c(n_dof);
				c.segment(0,3) = q.getFBPosition();
				c.segment(3,3) = q.getFBOrientation();
				c.tail(n_dof-6) = q.getJointValues();
				for(int z = 0; z < c.rows(); z++) foutLogMCP << c(z) << ", ";
				foutLogMCP << ";" << std::endl;			

				succ(i) = 1;
			}
			 
		}
				
	}	

	//std::cout << "qList2ndStage.size() = " << qList2ndStage.size() << std::endl;
	for(int i = 0; i < sigmaList.size(); i++){
		if(succ(i) == 1) std::cout << i << " SUCCESS" << std::endl;
		else std::cout << i << " FAILURE" << std::endl;
	}
}
*/ 


void Planner::run2ndStage(std::vector<Stance> sigmaList, std::vector<Configuration> qList){

	foutLogMCP << "********************************* 2ND STAGE *********************************" << std::endl;

	sol_len = sigmaList.size();

	sigmaList2ndStage.clear();
	qList2ndStage.clear();

	sigmaList2ndStage.push_back(sigmaList.at(0));
	qList2ndStage.push_back(qList.at(0));
	
	Stance sigma;
	Configuration q;

	for(int i = 1; i < sigmaList.size(); i++){
		
		foutLogMCP << "+++++ i = " << i << std::endl;

		sigma = sigmaList.at(i);
		q = qList.at(i);	

		for(int k = 0; k < 10; k++){

			bool resIK;	
			if(k = 0) resIK = true;
			else resIK = computeIKSolutionWithoutCoM(sigma, q, qList.at(i-1)); 

			//bool resIK = computeIKSolutionWithoutCoM(sigma, q, qList.at(i-1));

			if(resIK) foutLogMCP << "--------------- GS SUCCESS ---------------" << std::endl;
			else foutLogMCP << "--------------- GS FAIL ---------------" << std::endl;	

			if(resIK){
				Eigen::Vector3d rCoMdes = computeCoM(q);
				std::vector<EndEffector> activeEEsDes = sigma.retrieveActiveEndEffectors();
				std::cout << "activeEEsDes.size() = " << activeEEsDes.size() << std::endl;
				Eigen::MatrixXd rCdes(activeEEsDes.size(), 3);
				Eigen::MatrixXd nCdes(activeEEsDes.size(), 3);
				Eigen::Vector3d rCoM;
				Eigen::MatrixXd rC(activeEEsDes.size(), 3);
				Eigen::MatrixXd FC(activeEEsDes.size(), 3);

				for(int j = 0; j < activeEEsDes.size(); j++){
					rCdes.row(j) = sigma.getContact(j)->getPose().translation().transpose();
					nCdes.row(j) = sigma.getContact(j)->getNormal().transpose();
				}

				foutLogMCP << "rCdes" << std::endl;	
				foutLogMCP << rCdes << std::endl;	
				foutLogMCP << "nCdes" << std::endl;	
				foutLogMCP << nCdes << std::endl;	
				foutLogMCP << "rCoMdes" << std::endl;	
				foutLogMCP << rCoMdes.transpose() << std::endl;
						
				bool resCPL = computeCentroidalStatics(activeEEsDes, rCoMdes, rCdes, nCdes, rCoM, rC, FC); 

				foutLogMCP << "rC" << std::endl;	
				foutLogMCP << rC << std::endl;	
				foutLogMCP << "FC" << std::endl;	
				foutLogMCP << FC << std::endl;	
				foutLogMCP << "rCoM" << std::endl;	
				foutLogMCP << rCoM.transpose() << std::endl;	
						
				if(resCPL) foutLogMCP << "--------------- CPL SUCCESS ---------------" << std::endl;
				else foutLogMCP << "--------------- CPL FAIL ---------------" << std::endl;	

				if(resCPL){
					Stance sigmaBar;
					for(int j = 0; j < sigma.getSize(); j++){
						Contact* c_j_bar = sigma.getContact(j);
						Eigen::Vector3d F_j = FC.row(j).transpose();
						Contact* c_j = new Contact(c_j_bar->getEndEffectorName(), c_j_bar->getPose(), F_j, c_j_bar->getNormal());
						sigmaBar.addContact(c_j);	
					}
					
					Eigen::Vector3d rCoMError = rCoMdes - rCoM; 
					foutLogMCP << "rCoMErrorA = " << rCoMError.transpose() << std::endl;
					
					//bool resIK_CoM = computeIKSolutionWithCoM(sigmaBar, rCoM, q, qList.at(i-1));
					bool resIK_CoM;
					if(sigmaBar.getSize() == 4) resIK_CoM = computeIKSolutionWithCoM(sigmaList.at(i), rCoM, q, qList.at(i-1));
					else resIK_CoM = computeIKSolutionWithCoM(sigmaList.at(i-1), rCoM, q, qList.at(i-1));
					
					foutLogMCP << "L_HAND = " << computeForwardKinematics(q, L_HAND).translation().transpose() << std::endl;
					foutLogMCP << "R_HAND = " << computeForwardKinematics(q, R_HAND).translation().transpose() << std::endl;
					foutLogMCP << "L_FOOT = " << computeForwardKinematics(q, L_FOOT).translation().transpose() << std::endl;
					foutLogMCP << "R_FOOT = " << computeForwardKinematics(q, R_FOOT).translation().transpose() << std::endl;
					Eigen::Vector3d rCoMErrorB = computeCoM(q) - rCoM; 
					foutLogMCP << "rCoMErrorB = " << rCoMErrorB.transpose() << std::endl;

					if(resIK_CoM) foutLogMCP << "--------------- GS COM SUCCESS ---------------" << std::endl;
					else foutLogMCP << "--------------- GS COM FAIL ---------------" << std::endl;
			
					 		
					if(resIK_CoM){
						sigmaList2ndStage.push_back(sigmaBar);
						qList2ndStage.push_back(q);
						
						foutLogMCP << "CONFIGURATION # = " << i << std::endl;
						Eigen::VectorXd c(n_dof);
						c.segment(0,3) = q.getFBPosition();
						c.segment(3,3) = q.getFBOrientation();
						c.tail(n_dof-6) = q.getJointValues();
						for(int z = 0; z < c.rows(); z++) foutLogMCP << c(z) << ", ";
						foutLogMCP << ";" << std::endl;

						break;								
					}
					 
				}
				

			}

		}

	}	
}


int Planner::getTreeSize(){
	return tree->getSize();
}
