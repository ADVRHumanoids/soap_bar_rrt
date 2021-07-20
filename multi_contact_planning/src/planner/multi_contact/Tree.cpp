#include "Tree.hpp"

Tree::Tree(){
	verticesList.clear();
}

//Tree::~Tree(){}

std::shared_ptr<Vertex> Tree::getVertex(int i){
	return verticesList.at(i);
}

void Tree::addVertex(std::shared_ptr<Vertex> v){
	verticesList.push_back(v);
}

int Tree::getSize(){
	return verticesList.size();
}

void Tree::clear(){
	verticesList.clear();
}
 

