#ifndef STANCE_H
#define STANCE_H

#include <stdlib.h>
#include <vector>
#include "enum.h"
#include <Eigen/Dense>  
#include "Contact.hpp"
#include "constant_values.hpp"

class Stance {

	private:

		std::vector<Contact*> contactsList;
		
	public:	

        Stance();

		Contact* getContact(int i);
		void addContact(Contact* c);
        int getSize();
		void clear();

        std::vector<EndEffector> retrieveActiveEndEffectors();
		bool isActiveEndEffector(EndEffector pk);
		Eigen::Vector3d retrieveContactPosition(EndEffector pk);

};

#endif
