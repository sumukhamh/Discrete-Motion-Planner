/*
Date created: 11/05/2018
Author: Sumukha M. Harish
email: sumukhamh@gmail.com

This the derived class from Discrete Motion Planner. It tries to find a 
path in a 2D map by using generating a random walk. The planner makes use
of the search() interface from the DiscretePlanner parent class to implement 
a seperate method of its own for path finding. The RandomPlanner class
provides the following functions.

search():
Calls the makeGrid() function from DiscretePlanner to generate node graph
and generates path by randomPathGenerator().

randomPathGenerator():
The random planner tries to find a path to the goal by randomly moving in the 
environment (only orthogonal moves are legal). If the planner can not find an 
acceptable solution in less than max_step_number , the search should fail. 
The random planner, while being erratic, has a short memory, and it will never
attempt to visit a cell that was visited in the last sqrt(max_step_number) steps
except if this is the only available option.

max_step_number = world_row_size x world_column_size

getNextNode():
This method gets the next node based on the conditions followed by randomPathGenerator()

isInBuffer():
Checking if the node was in the last sqrt(max_step_number) steps

insertIntoBuffer():
Insert present node into memory. If size of memory is full, empty the last entry 
and push current node.

*/
#include "RandomPlanner.h"

RandomPlanner::RandomPlanner(){
}

RandomPlanner::~RandomPlanner(){
}

std::vector<std::tuple<int, int>> RandomPlanner::search(std::vector<std::vector<int>> world, std::tuple<int, int> robot_pose, std::tuple<int, int> goal_pose)
{
	printf("\nRandom Planner:\n");

	makeGrid(world, robot_pose, goal_pose, visualize);

	m_RandomPath = randomPathGenerator();

	return m_RandomPath;
}

std::vector<std::tuple<int, int>> RandomPlanner::randomPathGenerator()
{
	printf("\nCalculating random path by random path generator...\n"); 

	if (!nodeMap[m_goal] || !nodeMap[m_robot]) {
		std::cout << "Invalid start or goal position!" << std::endl;
		return {};
	}
	
	max_step_number = row * col;

	maxBufferSize = sqrt(max_step_number);

	std::vector<std::tuple<int, int>> path;

	Node* start = nodeMap[m_robot];
	Node* end = nodeMap[m_goal];
	Node* node = start;

	int randomChoice;
	int step = 1;

	while (step <= max_step_number) {
		if (node == end) {
			pathFound = true;
			break;
		}

		path.push_back(std::make_tuple(node->x, node->y));

		m_world[node->x][node->y] = -6;

		node = getNextNode(node);

		step++;
	}

	m_world[node->x][node->y] = -6;

	if(visualize)
		visualizePath();

	if (pathFound) {
		printf("Path found!\n");
		printf("Steps taken: %d\n", step);
		return path;
	}

	else {
		printf("Path not found!\n");
		return {};
	}
}

Node* RandomPlanner::getNextNode(Node* node) 
{
	int randomChoice = std::rand() % node->neighbours.size();

	Node* nextnode = nodeMap[node->neighbours[randomChoice]];

	while (isInBuffer(nextnode)) {
		if (node->neighbours.size() == 1)
			break;
		randomChoice = std::rand() % node->neighbours.size();
		nextnode = nodeMap[node->neighbours[randomChoice]];
		insertIntoBuffer(nextnode);
	}
	return nextnode;
}

bool RandomPlanner::isInBuffer(Node* node) {
	if (buffer.size() == 0 || std::find(buffer.begin(), buffer.end(), node) == buffer.end())
		return false;
	else
		return true;
}

void RandomPlanner::insertIntoBuffer(Node* node) {
	if (buffer.size() == maxBufferSize)
		buffer.pop_front();
	buffer.push_back(node);
}