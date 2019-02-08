#pragma once
#include <vector>

class Node {
public:
	Node(int val);
	~Node();
	
	int x;
	int y;
	
	int value;
	
	bool visited = false;
	bool isInqueue = false;

	double distanceFromStart = INT_MAX;
	double heuristic = INT_MAX;
	double sum = INT_MAX;

	std::vector<std::pair<int, int>> neighbours = {};
	
	Node* parent = nullptr;
};

