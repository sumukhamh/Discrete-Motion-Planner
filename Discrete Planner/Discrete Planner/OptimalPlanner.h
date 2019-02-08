#pragma once
#include <iostream>
#include <tuple>
#include <vector>
#include <queue>
#include <algorithm>

#include "DiscretePlanner.h"

class OptimalPlanner : public DiscretePlanner {
public:
	bool pathFound = false;
	bool visualize = true;
	std::vector<std::tuple<int, int>> m_OptimalPath;

	OptimalPlanner();
	~OptimalPlanner();

	struct Compare {
		bool operator()(Node* n1, Node* n2) {
			return n1->sum > n2->sum;
		}
	};

	std::priority_queue<Node*, std::vector<Node*>, Compare> pq;
	
	std::vector<std::tuple<int, int>> search(std::vector<std::vector<int>> world, std::tuple<int, int> robot_pose, std::tuple<int, int> goal_pose);
	
	std::vector<std::tuple<int, int>> AStar();
	
	std::vector<std::tuple<int, int>> getPath(Node * start, Node * end);
};

