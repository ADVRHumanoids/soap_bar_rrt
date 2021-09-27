#include "Configuration.hpp"

Configuration::Configuration(){ }

Configuration::Configuration(int dof){
    q_jnt.resize(dof);
}

Configuration::~Configuration(){ }

Configuration& Configuration::operator= (const Configuration& c)
{
    this->setFBPosition(c.q_fb.segment(0,3));
    this->setFBOrientation(c.q_fb.segment(3,3));
    this->setJointValues(c.q_jnt);

    return *this;
}

Eigen::Vector3d Configuration::getFBPosition(){
	Eigen::Vector3d p = q_fb.head(3);
	return p;
}

Eigen::Vector3d Configuration::getFBOrientation(){
	Eigen::Vector3d o = q_fb.tail(3);
	return o;
}

void Configuration::setFBPosition(Eigen::Vector3d p){
	q_fb.head(3) = p;	
}

void Configuration::setFBOrientation(Eigen::Vector3d o){
	q_fb.tail(3) = o;	
}

double Configuration::getJointComponent(int i){
	return q_jnt(i);
}

void Configuration::setJointComponent(int i, double val){
	q_jnt(i) = val;
}

Eigen::VectorXd Configuration::getJointValues(){
	return q_jnt;
}

void Configuration::setJointValues(Eigen::VectorXd _q_jnt){
	q_jnt.resize(_q_jnt.size());
	q_jnt = _q_jnt; 
}

void Configuration::print()
{
    std::cout << "Floating base: " << q_fb.transpose() << std::endl;
    std::cout << "Joint position: " << q_jnt.transpose() << std::endl;
}


