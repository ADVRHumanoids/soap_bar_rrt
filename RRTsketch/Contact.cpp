#include "Contact.hpp"

Contact::Contact(EndEffector _name, Eigen::Vector3d _position, Eigen::Vector3d _force){
	name = _name;
	position = _position;
	force = _force;
}

~Contact::Contact(){};

EndEffector Contact::getEndEffectorName(){
	return name;
}

Eigen::Vector3d Contact::getPosition(){
	return position;
}

Eigen::Vector3d Contact::getForce(){
	return force;
}
