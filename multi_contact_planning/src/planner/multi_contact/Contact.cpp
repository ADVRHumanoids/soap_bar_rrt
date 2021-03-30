#include "Contact.hpp"

Contact::Contact(EndEffector _name, Eigen::Affine3d _pose, Eigen::VectorXd _force, Eigen::Vector3d _normal){
    name = _name;
    pose = _pose;
    force = _force;
    normal = _normal;
}

Contact::~Contact(){}

EndEffector Contact::getEndEffectorName(){
    return name;
}

Eigen::Affine3d Contact::getPose(){
    return pose;
}

void Contact::setPose(Eigen::VectorXd pos, Eigen::Matrix3d rot){
    pose.translation() = pos;
    pose.linear() = rot;
}

Eigen::VectorXd Contact::getForce(){
    return force;
}

Eigen::Vector3d Contact::getNormal(){
    return normal;
}

void Contact::setForce(Eigen::VectorXd _force){
    force = _force;
}
