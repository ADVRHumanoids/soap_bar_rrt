#ifndef TREE_H
#define TREE_H

#include <stdlib.h>  
#include <vector>
#include <Eigen/Dense>
#include "Vertex.hpp"
#include "utils.hpp"  

class Tree {

	private:		
		std::vector<Vertex*> verticesList;

	public:
		Tree();
		~Tree();

		Vertex* getVertex(int i);
		void addVertex(Vertex* v);
		int getSize();
		void clear();

};

#endif
