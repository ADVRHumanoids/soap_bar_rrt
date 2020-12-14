#include "Vertex.hpp"

Vertex::Vertex(Stance _sigma, Configuration _q, int _parentIndex){ 
	sigma = _sigma;
	q = _q;
	parentIndex = _parentIndex;
	numExpansionAttempts = 0;
    transition = false;
}

Vertex::Vertex(Stance _sigma, Configuration _q, Configuration _q_transition, int _parentIndex){
    sigma = _sigma;
    q = _q;
    parentIndex = _parentIndex;
    numExpansionAttempts = 0;
    q_transition = _q_transition;
    transition = true;
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

void Vertex::setTransitionConfiguration(Configuration q){
    q_transition = q;
}

Configuration Vertex::getTransitionConfiguration() const{
    return q_transition;
}

bool Vertex::getTransitionState(){
    return transition;
}
