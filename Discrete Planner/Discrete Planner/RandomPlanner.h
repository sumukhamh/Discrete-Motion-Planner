#pragma once
#include <iostream>
#include <tuple>
#include <vector>
#include <queue>
#include <algorithm>
#include <ctime>

#include "DiscretePlanner.h"

class RandomPlanner : public DiscretePlanner{
private:
	std::list<Node*> buffer;
	int maxBufferSize;

public:
	bool pathFound = false;
	bool visualize = true;
	int max_step_number;
	std::vector<std::tuple<int, int>> m_RandomPath;

	RandomPlanner();
	~RandomPlanner();

	std::vector<std::tuple<int, int>> search(std::vector<std::vector<int>> world, std::tuple<int, int> robot_pose, std::tuple<int, int> goal_pose);

	std::vector<std::tuple<int, int>> randomPathGenerator();

	Node* getNextNode(Node * node);

	bool isInBuffer(Node * node);

	void insertIntoBuffer(Node * node);
	
};

