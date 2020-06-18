#include "Stance.hpp"

Stance::Stance(){
	contactsList.clear();
}

Contact* Stance::getContact(int i){
	return contactsList.at(i);
}

void Stance::addContact(Contact* c){
	contactsList.push_back(c);
}

int Stance::getSize(){
	return contactsList.size();
}

void Stance::clear(){
	contactsList.clear();
}

std::vector<EndEffector> Stance::retrieveActiveEndEffectors(){
    std::vector<EndEffector> activeEEs;

	for(int i = 0; i < contactsList.size(); i++){
		activeEEs.push_back(contactsList.at(i)->getEndEffectorName());		
	}

	return activeEEs;
}

bool Stance::isActiveEndEffector(EndEffector pk){
	std::vector<EndEffector> activeEEs;

	for(int i = 0; i < contactsList.size(); i++){
		if(contactsList.at(i)->getEndEffectorName() == pk) return true;		
	}
	
	return false;	
}

Eigen::Vector3d Stance::retrieveContactPosition(EndEffector pk){
	//note: this function returns the position of the ee pk as described in the stance, assuming that it actually contains a contact involving pk
	// this assumption must always be verified before calling this function!

	for(int i = 0; i < contactsList.size(); i++){
		if(contactsList.at(i)->getEndEffectorName() == pk) return contactsList.at(i)->getPosition();
	}

} 

