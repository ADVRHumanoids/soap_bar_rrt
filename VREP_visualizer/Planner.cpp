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

Planner::Planner(Configuration _qInit, Eigen::MatrixXd _posActiveEEsInit, Eigen::MatrixXd _contactForcesInit, std::vector<EndEffector> _activeEEsInit, Eigen::MatrixXd _posActiveEEsGoal, std::vector<EndEffector> _activeEEsGoal, Eigen::MatrixXd _pointCloud, Eigen::MatrixXd _pointNormals){
		
	// set initial configuration
	qInit = _qInit;	
	// create initial stance 
	for(int i = 0; i < _activeEEsInit.size(); i++){
		Eigen::Vector3d p_i = _posActiveEEsInit.row(i).transpose();
		Eigen::Vector3d F_i = _contactForcesInit.row(i).transpose();
		Contact* c = new Contact(_activeEEsInit.at(i), p_i, F_i);
		sigmaInit.addContact(c);
	}

	// create goal stance 
	for(int i = 0; i < _activeEEsGoal.size(); i++){
		Eigen::Vector3d p_i = _posActiveEEsGoal.row(i).transpose();
		Eigen::Vector3d F_i(0.0,0.0,0.0);
		Contact* c = new Contact(_activeEEsGoal.at(i), p_i, F_i); 		
		sigmaGoal.addContact(c);
	}

	// set the environment representation
	pointCloud = _pointCloud;
	pointNormals = _pointNormals;
	
	// create an empty tree
	tree = new Tree();

	// add to endEffectorsList all the ee that we want to consider
	endEffectorsList.push_back(L_HAND);
	endEffectorsList.push_back(R_HAND);
	//endEffectorsList.push_back(L_KNEE);
	//endEffectorsList.push_back(R_KNEE);
	endEffectorsList.push_back(L_FOOT);
	endEffectorsList.push_back(R_FOOT);
	
	// seed generators
	auto a = std::chrono::system_clock::now();
    time_t b = std::chrono::system_clock::to_time_t(a);
	integerGenerator.seed(b);
	pointGenerator.seed(2*b);
	exploitationGenerator.seed(3*b);
	pointInWorkspaceGenerator.seed(4*b);


}
 
Planner::~Planner(){ }

bool Planner::isGoalStance(Vertex* v){
	bool c1; // condition 1: true if the i-th active ee at the goal is also active at v
	bool c2; // condition 2: true if the position of the i-th active ee at the goal is respected at v
	double dist;

	Stance sigma = v->getStance();

	for(int i = 0; i < sigmaGoal.getSize(); i++){
		c1 = false;
		for(int j = 0; j < sigma.getSize(); j++){
			c2 = false;
			if(sigma.getContact(j)->getEndEffectorName() == sigmaGoal.getContact(i)->getEndEffectorName()){
				c1 = true;
				dist = euclideanDistance(sigma.getContact(j)->getPosition(), sigmaGoal.getContact(i)->getPosition());				
				if(dist < GOAL_TOLERANCE) c2 = true;
			}
		}

		if( c1 && c2 ) return true;
	}
	
	return false;	
}

Eigen::Vector3d Planner::pickRandomPoint(){
	return Eigen::Vector3d(pointDistribution(pointGenerator), pointDistribution(pointGenerator), pointDistribution(pointGenerator));
}

Eigen::Vector3d Planner::pickRandomPointInReachableWorkspace(Configuration q, Eigen::MatrixXd w){
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

	for(int i = 0; i < tree->getSize(); i++){
		Vertex* v = tree->getVertex(i);
		Configuration q = v->getConfiguration();
		Stance sigma = v->getStance();
		
		// evaluate conditons
		bool c1 = true; // the same vertex can be used for expansion at most a given number of times 
		bool c2 = true; // at least one contact (after expansion)
		bool c3 = true; // one contact allowed only if it involves one of the feet
		bool c4 = true; // two contacts can not involve only hands
		bool c5 = true;	// the foot and the knee of the same leg can not be simultaneously in contact

		std::vector<EndEffector> activeEEs = sigma.retrieveActiveEndEffectors();		
		std::vector<EndEffector> activeEEsCand;
		//if(isActiveEndEffector(pk, activeEEs)){
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
		if(activeEEsCand.size() == 0) c2 = false;
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
		 
		bool allConditionsRespected = c1 && c2 && c3 && c4 && c5; 

		if(allConditionsRespected){
			Eigen::Vector3d rFB = q.getFBPosition(); // TODO try to compute d using the position of the ee pk instead of the fb (kinematics needed here!)
			double d = euclideanDistance(rFB, r);
			if(d < dMin){	
				dMin = d;
				iMin = i;
			}
		}
	}

	return iMin;	

}

Eigen::MatrixXd Planner::computeReachableWorkspace(EndEffector pk, Configuration q){
	Eigen::MatrixXd w(3,2); // defines a cube: each row contains the bounds for the corresponding component (x,y,z) 

	w.setZero(); // TODO

	return w;
}

bool Planner::isGoalContactReachable(EndEffector pk, Configuration q, Eigen::MatrixXd w){
	
	Eigen::Vector3d pFB = q.getFBPosition();
	
	Eigen::Vector3d p = sigmaGoal.retrieveContactPosition(pk);	
	
	double d = euclideanDistance(pFB, p);
	if(d < WORKSPACE_RADIUS) return true;
	
	return false;
}
		
bool Planner::computeIKSolution(Stance sigma, Eigen::Vector3d rCoM, Configuration &q){
	// TODO goal sampler
	// coumputes a configuration q compatible with the stance sigma and returns true if such q exists, false otherwise

	q.setFBPosition(rCoM); // must be the FB not the COM!	

	return true; 
}

bool Planner::retrieveSolution(std::vector<Stance> &sigmaList, std::vector<Configuration> &qList){
	// if a plan has been found returns true and fills the two vectors (args) with the sequences of stances and configurations respectively
	// otherwise returns false

	int iEnd = tree->getSize() - 1;
	
	Vertex* v = tree->getVertex(iEnd);
	bool solutionFound = isGoalStance(v);

	if(!solutionFound) return false;
		
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

bool Planner::computeCentroidalStatics(std::vector<EndEffector> activeEEsDes, Eigen::Vector3d rCoMdes, Eigen::MatrixXd rCdes, Eigen::MatrixXd nCdes, Eigen::Vector3d &rCoM, Eigen::MatrixXd &rC, Eigen::MatrixXd &FC){

	std::cout << "**************** CPL INVOCATION *******************" << std::endl;

	double robot_mass = 10.0;
    double g = -9.81;
    double mu = 0.5;
	double F_thres = 20.0;
	double W_CoM = 1000.0;
 
	std::vector<std::string> contact_name;
    std::shared_ptr<cpl::CoMPlanner> cpl;
	cpl::solver::Solution sol;
	
	contact_name.clear();
	std::string name; 

	for(int i = 0; i < activeEEsDes.size(); i++){
		name = std::string("contact_") + std::to_string(i); 
		contact_name.push_back(name);	

		//std::cout << "name = " << name << std::endl;  
	}	

	cpl = std::make_shared<cpl::CoMPlanner>(contact_name, robot_mass);
    cpl->SetMu(mu);

	for(int i = 0; i < activeEEsDes.size(); i++){
		name = contact_name.at(i);
		cpl->SetContactPosition(name, rCdes.row(i));
		cpl->SetContactNormal(name, nCdes.row(i));
		cpl->SetForceThreshold(name, F_thres); 
	}	
	 
	cpl->SetCoMRef(rCoMdes);
	cpl->SetCoMWeight(W_CoM);
        
    sol = cpl->Solve();
    
	//std::cout << sol << std::endl;

	rCoM = sol.com_sol;
	if(activeEEsDes.size() == 1) rCoM(2) = rCoMdes(2);
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
		//std::cout << "r = " << r.transpose() << std::endl;
		//std::cout << "rCdes = " << rCdes << std::endl;
				
		//std::cout << "rError = " << rError.transpose() << std::endl;
		//std::cout << "nError = " << nError.transpose() << std::endl; 
		//std::cout << "-F.dot(n) = " << -F.dot(n) << std::endl;		
		//std::cout << "(F-(n.dot(F))*n).norm() - mu*(F.dot(n)) = " << (F-(n.dot(F))*n).norm() - mu*(F.dot(n)) << std::endl;		
		for(int j = 0; j < 3; j++) if(abs(rError(j)) > 1e-4 || abs(nError(j)) > 1e-4) return false;
		if( -F.dot(n) > 0.0 ) return false; 
		if( (F-(n.dot(F))*n).norm() - mu*(F.dot(n)) > 0.0 ) return false;
	
		rC.row(i) = r.transpose();
		FC.row(i) = F.transpose();   
		i++;
    }

	F_sumError = F_sum + Eigen::Vector3d(0.0, 0.0, robot_mass*g);	
	Torque_sumError = Torque_sum + Eigen::Vector3d(0.0, 0.0, 0.0);	
	//std::cout << "F_sumError = " << F_sumError.transpose() << std::endl; 
	//std::cout << "Torque_sumError = " << Torque_sumError.transpose() << std::endl; 
			
	for(int j = 0; j < 3; j++) if(abs(F_sumError(j)) > 1e-4 || abs(Torque_sumError(j)) > 1e-4) return false;

	return true;
}

Eigen::Vector3d Planner::getNormalAtPoint(Eigen::Vector3d p){

	/*
	// fitting a plane on k nearest points to p 
	std::map<double, Eigen::Vector3d> pointsOrderedByDistance;
	for(int i = 0; i < pointCloud.rows(); i++){
		Eigen::Vector3d p_i = pointCloud.row(i).transpose();
		double d = euclideanDistance(p_i, p);
		pointsOrderedByDistance.insert(std::pair<double, Eigen::Vector3d>(d, p_i)); 
	}

	int k = 100;
	Eigen::MatrixXd pointsNear(k,3);
	std::map<double, Eigen::Vector3d>::iterator itr; 
    itr = pointsOrderedByDistance.begin();
	for(int i = 0; i < k; i++) { 
		double d = itr->first;
		Eigen::Vector3d p_i = itr->second;
		pointsNear.row(i) = p_i.transpose();	
		itr++;
    } 
	Eigen::Vector3d centroid(pointsNear.col(0).mean(), pointsNear.col(1).mean(), pointsNear.col(2).mean());
	pointsNear.col(0).array() -= centroid(0);
	pointsNear.col(1).array() -= centroid(1);
	pointsNear.col(2).array() -= centroid(2);
	auto svd = pointsNear.transpose().jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
	
	// computing the normal in p	
	Eigen::Vector3d normal = svd.matrixU().rightCols<1>();
	if((Eigen::Vector3d(0.0, 0.0, 2.0)-p).transpose()*normal < 0.0) normal = -normal;	
	
	return normal;
	*/

	//Eigen::Vector3d nC(0.0, 0.0, 1.0);
	/*
	Eigen::Vector3d nF;
	for(int i = 0; i < pointCloud.rows(); i++){
		Eigen::Vector3d p_i = pointCloud.row(i).transpose();
		double d = euclideanDistance(p_i, p);
		if(d < 1e-4){
			nF = pointNormals.row(i).transpose();
			break; 	
		} 
	}
	std::cout << "nF = " << nF.transpose() << std::endl;
	*/

	Eigen::Vector3d nC;
	if(p(0) <= 1.506 && p(0) >= 1.494) nC << -1.0, 0.0, 0.0;
	else nC << 0.0, 0.0, 1.0; 
	//std::cout << "nC = " << nC.transpose() << std::endl;
	
	return nC;
}

void Planner::run(){

	std::vector<EndEffector> activeEEsGoal = sigmaGoal.retrieveActiveEndEffectors();
	
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

		double pr = exploitationDistribution(exploitationGenerator);
		if(pr < EXPLOITATION_RATE){
			// exploitation
			// pick a random contact from sigmaGoal and retrieve the corresponding ee and position
			Contact* c = pickRandomContactFromGoalStance();
			pk = c->getEndEffectorName(); 
			rRand = c->getPosition();
		}
		else{
			// exploration
			// pick a random ee and a random point
			pk = pickRandomEndEffector(); 
			rRand = pickRandomPoint();
		}

		int iNear = findNearestVertexIndex(pk, rRand);    

		if(iNear != -1){ // a vertex of the tree is available for expansion using ee pk (based on the condition specified in function findNearestVertexIndex)
			vNear = tree->getVertex(iNear);
			Stance sigmaNear = vNear->getStance();
			Configuration qNear = vNear->getConfiguration();
			vNear->increaseNumExpansionAttempts();
			
			std::vector<EndEffector> activeEEsNear = sigmaNear.retrieveActiveEndEffectors();

			std::vector<EndEffector> activeEEsDes;
			Eigen::Vector3d rCDes_k;
			Eigen::MatrixXd rCdes;
			Eigen::MatrixXd nCdes;
			
			if(sigmaNear.isActiveEndEffector(pk)){
				rCdes.resize(activeEEsNear.size()-1, 3);
				nCdes.resize(activeEEsNear.size()-1, 3);
				int h = 0;	
				for(int i = 0; i < activeEEsNear.size(); i++){
					if(activeEEsNear.at(i) != pk){	
						activeEEsDes.push_back(activeEEsNear.at(i));
						rCdes.row(h) = sigmaNear.getContact(i)->getPosition().transpose();
						nCdes.row(h) = getNormalAtPoint(rCdes.row(h)).transpose();
						h++;
					}
				}
			}
			else{
				rCdes.resize(activeEEsNear.size()+1, 3);
				nCdes.resize(activeEEsNear.size()+1, 3);
				for(int i = 0; i < activeEEsNear.size(); i++){
					activeEEsDes.push_back(activeEEsNear.at(i));
					rCdes.row(i) = sigmaNear.getContact(i)->getPosition().transpose();
					nCdes.row(i) = getNormalAtPoint(rCdes.row(i)).transpose();						
				}
				activeEEsDes.push_back(pk);
			
				Eigen::MatrixXd reachableWorkspace = computeReachableWorkspace(pk, qNear); // now we are using a sphere centered at the FB so this is useless for the moment

				if(sigmaGoal.isActiveEndEffector(pk) && isGoalContactReachable(pk, qNear, reachableWorkspace)){
					std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
					Eigen::Vector3d rGoal = sigmaGoal.retrieveContactPosition(pk); 
					rCDes_k = rGoal;
				}				
				else{
					rCDes_k = pickRandomPointInReachableWorkspace(qNear, reachableWorkspace);   
				}

				rCdes.row(rCdes.rows()-1) = rCDes_k.transpose();
				nCdes.row(rCdes.rows()-1) = getNormalAtPoint(rCDes_k).transpose();						
						
			}		

			Eigen::Vector3d rCoMdes = qNear.getFBPosition(); // must be the CoM not the FB!
			
			Eigen::Vector3d rCoM;
			Eigen::MatrixXd rC;
			Eigen::MatrixXd FC;
			
			rC.resize(activeEEsDes.size(), 3);
			FC.resize(activeEEsDes.size(), 3);
			
			bool resCPL = computeCentroidalStatics(activeEEsDes, rCoMdes, rCdes, nCdes, rCoM, rC, FC); 
			
			if(resCPL) std::cout << "--------------------------------------------------------------------------CPL SUCCESS" << std::endl;
			else std::cout << "--------------------------------------------------------------------------CPL FAIL" << std::endl;

			if(resCPL){
			
				// create the new stance

				Stance sigmaNew;
				for(int i = 0; i < activeEEsDes.size(); i++){
					Eigen::Vector3d p_i = rCdes.row(i).transpose();
					Eigen::Vector3d F_i = FC.row(i).transpose();
					Contact* c = new Contact(activeEEsDes.at(i), p_i, F_i);
					sigmaNew.addContact(c);
				}
				
				// create the new configuration
				
				Configuration qNew;
				bool resIK = computeIKSolution(sigmaNew, rCoM, qNew);

				if(resIK){
					vNew = new Vertex(sigmaNew, qNew, iNear);  
					tree->addVertex(vNew);
				}
		
				solutionFound = isGoalStance(vNew);
			}
		}		
		
		j++;
		
	}

	std::cout << "iters = " << j << std::endl;
	std::cout << "tree size = " << tree->getSize() << std::endl;

}

int Planner::getTreeSize(){
	return tree->getSize();
}
