#ifndef TREE_H
#define TREE_H

#include <stdlib.h>  
#include <vector>
#include <Eigen/Dense>
#include "Vertex.hpp"
#include "utils.hpp"  

#include <memory>

class Tree {

	private:	
        std::vector<std::shared_ptr<Vertex>> verticesList;

	public:
		Tree();
		~Tree() = default;

		std::shared_ptr<Vertex> getVertex(int i);
		void addVertex(std::shared_ptr<Vertex> v);
		int getSize();
		void clear();

};

#endif
