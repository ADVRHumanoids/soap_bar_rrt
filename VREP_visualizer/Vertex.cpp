#include "Vertex.hpp"

Vertex::Vertex(Stance _sigma, Configuration _q, int _parentIndex){ 
	sigma = _sigma;
	q = _q;
	parentIndex = _parentIndex;
	numExpansionAttempts = 0;
}

Vertex::~Vertex(){}

Stance Vertex::getStance(){
	return sigma;
}

Configuration Vertex::getConfiguration(){
	return q;
}

int Vertex::getNumExpansionAttempts(){
	return numExpansionAttempts;
}

void Vertex::increaseNumExpansionAttempts(){
	numExpansionAttempts++;
}

int Vertex::getParentIndex(){
	return parentIndex;
}
