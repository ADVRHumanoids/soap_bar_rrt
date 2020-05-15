#ifndef STANCE_H
#define STANCE_H

#include <stdio.h>
#include <map> //
#include <vector>
#include <iostream> //
#include "enum.h"
#include "Eigen/Dense" //
#include "Contact.hpp" 
#include "constant_values.hpp"

class Stance {

	private:

		std::vector<Contact*> contactsList;
		
	public:	

		Stance();
		~Stance();

		Contact* getContact(int i);
		void addContact(Contact* c);
		int getSize();
		void clear();

		std::vector<EndEffector> retrieveActiveEndEffectors();

};

#endif
