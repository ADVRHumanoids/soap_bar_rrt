#include "Contact.hpp"

Contact::Contact(EndEffector _name, Eigen::Affine3d _pose, Eigen::Vector3d _force, Eigen::Vector3d _normal){
	name = _name;
	pose = _pose;
	force = _force;
	normal = _normal;
}

EndEffector Contact::getEndEffectorName(){
	return name;
}

Eigen::Affine3d Contact::getPose(){
	return pose;
}

Eigen::Vector3d Contact::getForce(){
	return force;
}

Eigen::Vector3d Contact::getNormal(){
	return normal;
}