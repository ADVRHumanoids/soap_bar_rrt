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

std::vector<Contact::EndEffector> Stance::retrieveActiveEndEffectors(){
    std::vector<Contact::EndEffector> activeEEs;

	for(int i = 0; i < contactsList.size(); i++){
		activeEEs.push_back(contactsList.at(i)->getEndEffectorName());		
	}

	return activeEEs;
}
