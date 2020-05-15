#include "RRTtree.hpp"

Tree::Tree(){
	verticesList.clear();
}

Tree::~Tree(){}

Vertex* Tree::getVertex(int i){
	return verticesList.at(i);
}

void Tree::addVertex(Vertex* v){
	verticesList.push_back(v);
}

int Tree::getSize(){
	return verticesList.size();
}

void Tree::clear(){
	verticesList.clear();
}
 

