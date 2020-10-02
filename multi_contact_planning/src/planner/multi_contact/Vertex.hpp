#ifndef VERTEX_H
#define VERTEX_H

#include <stdlib.h>
#include <vector>
#include "enum.h"
#include <Eigen/Dense> 
#include "Stance.hpp" 
#include "Configuration.hpp" 
#include "constant_values.hpp"

class Vertex {

	private:

		Stance sigma;
		Configuration q;
		
		int numExpansionAttempts;
		int parentIndex;
		
	public:	

		Vertex(Stance _sigma, Configuration _q, int _parentIndex);
		~Vertex();

		Stance getStance();
		Configuration getConfiguration();
		int getNumExpansionAttempts();
		void increaseNumExpansionAttempts(); 
		int getParentIndex();	

};

#endif
