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
        Configuration q_transition;
		
		int numExpansionAttempts;
		int parentIndex;

        bool transition;
		
	public:	

		Vertex(Stance _sigma, Configuration _q, int _parentIndex);
        Vertex(Stance _sigma, Configuration _q, Configuration _q_transition, int _parentIndex);
		~Vertex();

		Stance getStance();
		Configuration getConfiguration();
		int getNumExpansionAttempts();
		void increaseNumExpansionAttempts(); 
		int getParentIndex();	
        void setTransitionConfiguration(Configuration q);
        Configuration getTransitionConfiguration() const;

        bool getTransitionState();

};

#endif
