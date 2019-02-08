/*
Date created: 11/05/2018
Author: Sumukha M. Harish
email: sumukhamh@gmail.com

This the derived class from Discrete Motion Planner. It finds the optimal
path in a 2D map by using AStar algorithm. The planner makes use of the 
search() interface from the DiscretePlanner parent class to implement a
seperate method of its own for path finding. The OptimalPlanner class 
provides the following functions:

search():
Calls the makeGrid() function from DiscretePlanner to generate node graph 
and generates optimal path by A* algorithm.

AStar():
Implements the A* algorithm by making use of a priority queue. The heuristic
is the Euclidean distance. The time complexity of the algorithm is 
O(edges x vertices)log(vertices).

getPath():
Given a start and end node and after the completion of the A* algorithm, 
the getPath() method is used to obtain the vector of tuples which represents
the path from the the start to the end node.

*/

#include "OptimalPlanner.h"

OptimalPlanner::OptimalPlanner(){
}

OptimalPlanner::~OptimalPlanner(){
}

std::vector<std::tuple<int, int>> OptimalPlanner::search(std::vector<std::vector<int>> world, std::tuple<int, int> robot_pose, std::tuple<int, int> goal_pose)
{	
	printf("\nOptimal Planner:\n");

	makeGrid(world, robot_pose, goal_pose, visualize);

	m_OptimalPath = AStar();

	return m_OptimalPath;
}

std::vector<std::tuple<int, int>> OptimalPlanner::AStar(){
	
	printf("\nCalculating optimal path by A* algorithm...\n");

	if (!nodeMap[m_goal] || !nodeMap[m_robot]) {
		std::cout << "Invalid start or goal position!" << std::endl;
		return {};
	}

	nodeMap[m_robot]->distanceFromStart = 0;

	Node* start = nodeMap[m_robot];
	Node* end = nodeMap[m_goal];
	
	pq.push(start);
	 
	while (!pq.empty()) {
		Node* current = pq.top();
		pq.pop();
		
		current->visited = true;

		if (current == end) {
			pathFound = true;
			break;
		}

		for (auto n : current->neighbours) {
			Node* neighbour = nodeMap[n];

			if (!neighbour->visited) {
				if (neighbour->distanceFromStart > current->distanceFromStart + cost) {
					neighbour->distanceFromStart = current->distanceFromStart + cost;
					neighbour->sum = neighbour->distanceFromStart + neighbour->heuristic;
					neighbour->parent = current;
					if (!neighbour->isInqueue) {
						pq.push(neighbour);
						neighbour->isInqueue = true;
					}
				}
			}
		}
	}

	if (pathFound) {
		printf("Path found!\n");
		return getPath(start, end);
	}
	else {
		printf("Path not found!\n");
		return {};
	} 
}

std::vector<std::tuple<int, int>> OptimalPlanner::getPath(Node* start, Node* end) {
	Node* node = end;
	
	std::vector<std::tuple<int, int>> path;
	
	while (node != start) {
		m_world[node->x][node->y] = -6;
		path.push_back(std::make_tuple(node->x, node->y));
		node = node->parent;
	}

	m_world[start->x][start->y] = -6;
	
	if(visualize)
		visualizePath();

	path.push_back(std::make_tuple(start->x, start->y));

	std::reverse(path.begin(), path.end());

	return path;
}


