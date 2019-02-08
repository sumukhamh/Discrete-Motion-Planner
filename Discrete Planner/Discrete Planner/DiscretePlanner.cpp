/*
Date created: 11/05/2018
Author: Sumukha M. Harish
email: sumukhamh@gmail.com

This is the base class of the Discrete Motion Planners on 2D environments.
The class is an abstract class with the search() interface provided in all
children classes. The DiscretePlanner class creates member varialbes
m_robot, m_goal, m_world. The children classes include 
1) RandomPlanner
2) OptimalPlanner

The DiscretePlanner class provides functions:

1) makeGrid():	
To create a node representation of the world

2) visualizeWorld():
To print out the world fed into the planner

3) generateGraph():
To generate a graph of nodes representing the map

4) getHueristic():
To obtain a euclidean distance heuristic from the goal to the node considered

5) getNeighbours()
From any point (x,y) in the map, get a vector of available navigable options

6) isValid():
Check if node is navigable

7) visualizePath():
Print the map with path encoded as '*'

The DiscretePlanner also creates a hashmap of the world by having the (x,y)
coordinates as the key and the pointer to the nodes as the values. boost::hash
is made use of to implement the hashing function.

*/
#include "DiscretePlanner.h"

DiscretePlanner::DiscretePlanner(){
}

DiscretePlanner::~DiscretePlanner(){
}

void DiscretePlanner::makeGrid(std::vector<std::vector<int>> world, std::tuple<int, int> robot_pose, std::tuple<int, int> goal_pose, bool visualize){
	m_world = world;
	m_robot = std::make_pair(std::get<0>(robot_pose), std::get<1>(robot_pose));
	m_goal = std::make_pair(std::get<0>(goal_pose), std::get<1>(goal_pose));

	row = m_world.size();
	col = m_world[0].size();

	printf("Robot start position: (%d, %d)\n", m_robot.first, m_robot.second);
	printf("Robot goal position: (%d, %d)\n", m_goal.first, m_goal.second);

	if(visualize)
		visualizeWorld();

	generateGraph();
}

void DiscretePlanner::visualizeWorld() {
	printf("\nThe world:\n");

	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			if (i == m_robot.first && j == m_robot.second)
				std::cout << 'R' << " ";
			else if (i == m_goal.first && j == m_goal.second)
				std::cout << 'G' << " "; 
			else if (m_world[i][j])
				std::cout << 1 << " ";
			else
				std::cout << 0 << " ";
		}
		std::cout << std::endl;
	}
}

void DiscretePlanner::generateGraph() {
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			if (!m_world[i][j]) {
				Node *node = new Node(m_world[i][j]);

				node->x = i;
				node->y = j;

				node->heuristic = getHeuristic(i, j);
				node->neighbours = getNeighbours(i, j);
				
				nodeMap[std::make_pair(i, j)] = node;

				node = nullptr;
				delete node;
			}
		}
	}
}

double DiscretePlanner::getHeuristic(int x, int y){
	return pow((x - m_goal.first), 2) + pow((y - m_goal.second), 2);
}

std::vector<std::pair<int, int>> DiscretePlanner::getNeighbours(int x, int y){
	std::vector<std::pair<int, int>> n;

	if (isValid(x - 1, y))
		n.push_back(std::make_pair(x - 1, y));
	if (isValid(x + 1, y))
		n.push_back(std::make_pair(x + 1, y));
	if (isValid(x, y - 1))
		n.push_back(std::make_pair(x, y - 1));
	if (isValid(x, y + 1))
		n.push_back(std::make_pair(x, y + 1));

	return n;
}

bool DiscretePlanner::isValid(int x, int y){
	if ((x >= 0 && x < row) && (y >= 0 && y < col) && (!m_world[x][y]))
		return true;
	else
		return false;
}

void DiscretePlanner::visualizePath(){
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			std::cout << (char)(m_world[i][j]+48) << " ";
		}
		std::cout << std::endl;
	}
}

