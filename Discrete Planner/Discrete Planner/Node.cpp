/*
Date created: 11/05/2018
Author: Sumukha M. Harish
email: sumukhamh@gmail.com

This is the Node class that is used to represent every point on
the map. A graph of nodes represents the entire map. The node
class provides the following attributes:

point : (x,y)

value : 1 if obstacle, 0 if free

isVisited: if node has been previously visited while exploring

isInqueue: if node is in the priority queue for consideration

distanceFromStart: distance computed from start

heuristic: Euclidean heuristic distance from the goal

sum: distanceFromStart + heuristic

neighbours: pair of integers representing navigable options in 
the 4 orthogonal directions

parent: parent through which the node was visited

*/

#include "Node.h"

Node::Node(int val){
	value = val;
}

Node::~Node(){
}
