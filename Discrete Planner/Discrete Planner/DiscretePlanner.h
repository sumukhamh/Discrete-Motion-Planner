#pragma once
#include <iostream>
#include <tuple>
#include <vector>
#include <unordered_map>

#include "Node.h"
#include "boost/functional/hash.hpp"

class DiscretePlanner {
public:
	std::vector<std::vector<int>> m_world;
	std::pair<int, int> m_robot;
	std::pair<int, int> m_goal;

	int row;
	int col;
	int cost = 1;
	std::unordered_map<std::pair<int, int>, Node*, boost::hash<std::pair<int, int>>> nodeMap;

	DiscretePlanner();
	~DiscretePlanner();
	
	void makeGrid(std::vector<std::vector<int>> world, std::tuple<int, int> robot_pose, std::tuple<int, int> goal_pose, bool visualize);

	void visualizeWorld();

	void generateGraph();

	double getHeuristic(int x, int y);
	
	std::vector<std::pair<int, int>> getNeighbours(int x, int y);

	bool isValid(int x, int y);
	
	virtual std::vector<std::tuple<int, int>> search(std::vector<std::vector<int>> world, std::tuple<int, int> robot_pose, std::tuple<int, int> goal_pose) = 0;

	void visualizePath();
};

