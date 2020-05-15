#ifndef CONTACT_H
#define CONTACT_H

#include <stdio.h>
#include <map> //
#include <vector>
#include <iostream> //
#include "enum.h"
#include "Eigen/Dense"
#include "constant_values.hpp"

class Contact {

	private:
		EndEffector name;
		Eigen::Vector3d position;	
		Eigen::Vector3d force;	
				
	public:	

		Contact(EndEffector _name, Eigen::Vector3d _position, Eigen::Vector3d _force);
		~Contact();

		EndEffector getEndEffectorName();
		Eigen::Vector3d getPosition();
		Eigen::Vector3d getForce();
};

#endif
