#include "Planner.hpp"
// DA VEDERE COSA INCLUDERE
#include <stdlib.h>   
#include <algorithm>
#include <chrono>
#include <fstream>
#include <random>
#include <thread>

// random numbers generators and distributions
static std::default_random_engine exploitationGenerator;
static std::uniform_real_distribution<double> exploitationDistribution(0.0, 1.0);
static std::default_random_engine pointGenerator;
static std::uniform_real_distribution<double> pointDistribution(-5.0, 5.0);
static std::default_random_engine integerGenerator;
static std::uniform_int_distribution<int> integerDistribution(0, 100);

Planner::Planner(Configuration _qInit, Eigen::MatrixXd _contactForcesInit, std::vector<EndEffector> _activeEEsInit, Eigen::MatrixXd _posActiveEEsGoal, std::vector<EndEffector> _activeEEsGoal, std::vector<Superquadric*> _superquadricsList){
		
	// _contactForcesInit is a matrix nC x 3 where each row contains the i-th contact force at qInit, with nC the number of active contacts at qInit
	// _posActiveEEsGoal is a matrix mC x 3 where each row contains the position of the i-th contact at the goal stance, with mC the number of active contacts at the goal stance  

	qInit = _qInit;
	contactForcesInit = _contactForcesInit;	
	for(int i = 0; i < _activeEEsInit.size(); i++) activeEEsInit.push_back(_activeEEsInit.at(i));
	posActiveEEsGoal = _posActiveEEsGoal;	
	for(int i = 0; i < _activeEEsGoal.size(); i++) activeEEsGoal.push_back(_activeEEsGoal.at(i));

	// create initial stance
	for(int i = 0; i < activeEEsInit.size(); i++){
		Eigen::Vector3d posEE_i = forwardKinematics(qInit, activeEEsInit.at(i)).head(3);
		Contact* c = new Contact(activeEEsInit.at(i), posEE_i, contactForcesInit.row(i)); 		
		sigmaInit.addContact(c);
	}

	// create goal stance 
	Eigen::Vector3d contactForceGoal;
	contactForceGoal.setZero(); // any values are ok here since the goal stance does not specify the contact forces (but only positions). I'm keeping this for completeness
	for(int i = 0; i < activeEEsGoal.size(); i++){
		Eigen::Vector3d posEE_i = posActiveEEsGoal.row(i);
		Contact* c = new Contact(activeEEsGoal.at(i), posEE_i, contactForceGoal); 		
		sigmaGoal.addContact(c);
	}

	// set the environment representation
	for(int i = 0; i < _superquadricsList.size(); i++) superquadricsList.push_back(_superquadricsList.at(i));
	
	// create an empty tree
	tree = new Tree();

	// add to endEffectorsList all the ee that we want to consider
	endEffectorsList.push_back(L_HAND);
	endEffectorsList.push_back(R_HAND);
	endEffectorsList.push_back(L_KNEE);
	endEffectorsList.push_back(R_KNEE);
	endEffectorsList.push_back(L_FOOT);
	endEffectorsList.push_back(R_FOOT);
	
	// seed generators
	auto a = std::chrono::system_clock::now();
    time_t b = std::chrono::system_clock::to_time_t(a);
	integerGenerator.seed(b);
	pointGenerator.seed(2*b);
	exploitationGenerator.seed(3*b);

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

		if(!c1 || (c1 && !c2)) return false;

	}
	
	return true;
}

Eigen::Vector3d Planner::pickRandomPoint(){
	return Eigen::Vector3d(pointDistribution(pointGenerator), pointDistribution(pointGenerator), pointDistribution(pointGenerator));
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
	double dMin = numeric_limits<double>::max();
	int iMin = -1;

	for(int i = 0; i < tree.getSize(); i++){
		Vertex* v = tree->getVertex(i);
		Configuration q = v->getConfiguration();
		Stance sigma = v->getStance();
		
		// evaluate conditons
		bool c1 = false; // true if the current vertex has not been already expanded the maximum number of times
		int nExp = v->getNumExpansionAttempts();
		if(nExp < MAX_NUM_EXP_PER_VERTEX) c1 = true;
	
		/*
		// possibly add other conditions; for example: don't put in contact at the same time the left foot and the left knee;
		// this kind of conditions can be evaluated on the set activeEEsCand (the set of active ee's when expanding the current vertex v using the ee pk) constructed in the following
		std::vector<EndEffector> activeEEs = sigma.retrieveActiveEndEffectors();		
		std::vector<EndEffector> activeEEsCand;
		if(isActiveEndEffector(pk, activeEEs)){
			for(int i = 0; i < activeEEs.size(); i++) if(activeEEs.at(i) != pk) activeEEsCand.push_back(activeEEs.at(i));
		}
		else{
			for(int i = 0; i < activeEEs.size(); i++) activeEEsCand.push_back(activeEEs.at(i));
			activeEEsCand.push_back(pk);
		}
		*/
		
		bool allConditionsRespected = c1; // in general c1 && c2 && ... 

		if(allConditionsRespected){
			Eigen::Vector3d rk = forwardKinematics(q, pk).head(3);
			double d = euclideanDistance(rk, r);
			if(d < dMin){	
				dMin = d;
				iMin = i;
			}
		}
	}

	return iMin;

}

bool Planner::isActiveEndEffector(EndEffector pk, std::vector<EndEffector> activeEEs){
	for(int i = 0; i < activeEEs.size(); i++) if(pk == activeEEs.at(i)) return true;
	return false;	
}

Eigen::MatrixXd Planner::computeReachableWorkspace(EndEffector pk, Configuration q){
	Eigen::MatrixXd w(3,2); // defines a cube: each row contains the bounds for the corresponding component (x,y,z) 

	w.setZero(); // TODO

	return w;
}

Eigen::Vector3d Planner::retrieveContactPosition(EndEffector pk, Stance sigma){
	//note: this function returns the position of the ee pk as described in the stance sigma, assuming that sigma actually contains a contact involving pk
	// this assumption must always be verified before calling this function!

	Eigen::Vector3d r;

	for(int i = 0; i < sigma.getSize(); i++){
		if(sigma.getContact(i)->getEndEffectorName() == pk) r = sigma.getContact(i)->getPosition();
	}

	return r;	
} 

bool Planner::isGoalContactReachable(EndEffector pk, Eigen::MatrixXd w){
	//note: this function returns true if the point r (specified by the position of the ee pk as described in the goal stance sigmaGoal) 
	// is contained within the cube defined by w (the reachable workspace), assuming that sigmaGoal actually contains a contact involving pk
	// this assumption must always be verified before calling this function!

	bool reachable = false;

	Eigen::Vector3d r = retrieveContactPosition(pk, sigmaGoal);
	if( w(0,0) < r(0) && r(0) < w(0,1) && w(1,0) < r(1) && r(1) < w(1,1) && w(2,0) < r(2) && r(2) < w(2,1) ) reachable = true;

	return reachable;
}

bool Planner::checkSuperquadricPointIntersection(Superquadric sq, Eigen::Vector3d  r){
	// return true if the point r lies on the superquadric sq (here I am using the word "intersect" just for analogy with function checkSuperquadricWorkspaceIntersection)
	// TODO
}	

int Planner::retrieveSuperquadricIndex(Eigen::Vector3d r){
	// returns the index (in the list of the superquadrics superquadricsList) of the superquadric where point r lies on
	for(int i = 0; i < superquadricsList.size(); i++){
		bool lie = checkSuperquadricPointIntersection(superquadricsList.at(i), r);		
		if(lie) return i;
	}

	// what follows is just for completeness since this function will be called only for points r involved in the goal stance that,
	// by definition, lie on some superquadrics (function will always return in the for loop)
	return -1;
}		

bool Planner::checkSuperquadricWorkspaceIntersection(Superquadric sq, Eigen::MatrixXd w){
	// return true if the superquadric sq intersect with the workspace w
	// TODO
}

std::vector<int> Planner::retrieveReachableSuperquadricsIndices(Eigen::MatrixXd w){
	// returns the indices (in the list of the superquadrics superquadricsList) of the superquadrics that intersect with the workspace defined by w
	std::vector<int> indices;
	
	for(int i = 0; i < superquadricsList.size(); i++){
		bool intersect = checkSuperquadricWorkspaceIntersection(superquadricsList.at(i), w);
		if(intersect) indices.push_back(i)
	}

	return indices;
}

int Planner::pickRandomSuperquadricIndex(std::vector<int> reachableSuperquadricsIndices){
	int index = integerDistribution(integerGenerator) % reachableSuperquadricsIndices.size();			
	return index;
}

Stance Planner::computeCandidateContacts(Eigen::Vector3d rCoMDes, Eigen::MatrixXd rCDes, std::vector<EndEffector> activeEEsDes, int superquadricIndexDes, EndEfector pk, Action f){
	// TODO from RAL
}
		
Configuration Planner::computeIKSolution(Stance sigma, bool &qFound){
	// TODO goal sampler
	// coumputes a configuration q compatible with the stance sigma and sets qFound to true if such q exists, otherwise sets qFound to false 
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


void Planner::run(){
	
	tree->clear();
	Vertex* vInit = new Vertex(sigmaInit, qInit, -1);
	tree->addVertex(vInit);
	
	int j = 0;
	bool solutionFound = false;

	EndEffector pk;
	Eigen::Vector3d rRand;

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
			Vertex* vNear = tree->getVertex(iNear);
			Stance sigmaNear = vNear->getStance();
			Configuration qNear = vNear->getConfiguration();
			vNear->increaseNumExpansionAttempts();
			
			std::vector<EndEffector> activeEEsNear = sigmaNear.retrieveActiveEndEffectors();

			Action f;
			std::vector<EndEffector> activeEEsDes;
			int superquadricIndexDes;
			Eigen::Vector3d rDes;
			
			if(isActiveEndEffector(pk, activeEEsNear)){
				f = REMOVE; 
				for(int i = 0; i < activeEEsNear.size(); i++) if(activeEEsNear.at(i) != pk) activeEEsDes.push_back(activeEEsNear.at(i));

				superquadricIndexDes = -1;
				rDes = forwardKinematics(qNear, pk).head(3);
				
			}
			else{
				f = ADD; 
				for(int i = 0; i < activeEEsNear.size(); i++) activeEEsDes.push_back(activeEEsNear.at(i));
				activeEEsDes.push_back(pk);

				Eigen::MatrixXd reachableWorkspace = computeReachableWorkspace(pk, qNear);

				if(isActiveEndEffector(pk, activeEEsGoal) && isGoalContactReachable(pk, reachableWorkspace)){
					Eigen::Vector3d rGoal = retrieveContactPosition(pk, sigmaGoal); 
					superquadricIndexDes = retrieveSuperquadricIndex(rGoal);
					rDes = rGoal;
				}				
				else{
					std::vector<int> reachableSuperquadricsIndices = retrieveReachableSuperquadricsIndices(reachableWorkspace);
					superquadricIndexDes = pickRandomSuperquadricIndex(reachableSuperquadricsIndices);
					rDes = rRand;
				}		
			}		

			Eigen::Vector3d rCoMDes = forwardKinematics(qNear, COM).head(3);
			Eigen::MatrixXd rCDes(endEffectorsList.size(), 3); // each row contains the desired position (x,y,z) of the i-th ee (following the order in endEffectorsList for the end-effectors)
			for(int i = 0; i < endEffectorsList.size(); i++){
				EndEffector pi = endEffectorsList.at(i); 
				if(f == ADD && pi == pk) rCDes.row(i) = rDes;
				else{
					Eigen::Vector3d rCDes_i = forwardKinematics(qNear, pi).head(3);
					rCDes.row(i) = rCDes_i;
				}
			}
			

			Stance sigmaNew = computeCandidateContacts(rCoMDes, rCDes, activeEEsDes, superquadricIndexDes, pk, f);
			bool qFound;		
			Configuration qNew = computeIKSolution(sigmaNew, qFound);

			if(qFound){
				Vertex* vNew = new Vertex(sigmaNew, qNew, iNear){  
				tree->addVertex(vNew);
			}
	
			solutionFound = isGoalStance(vNew);
		}		
		
		j++;
		
	}

}

int Planner::getTreeSize(){
	return tree->getSize();
}
