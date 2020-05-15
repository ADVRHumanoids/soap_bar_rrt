#ifndef TREE_H
#define TREE_H

#include <stdio.h> //
#include <vector>
#include <map> //
#include <iostream> //
#include "Eigen/Dense"
#include "Vertex.hpp"
#include "enum.h" //
#include "constant_values.hpp" //
#include "utils.hpp" //
#include <stdlib.h> //
#include <algorithm> //

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
