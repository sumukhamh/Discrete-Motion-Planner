/*
Date created: 11/05/2018
Author: Sumukha M. Harish
email: sumukhamh@gmail.com

This is the source file for launching the Discrete Motion Planner package for 2D environments. 
The Discrete Motion Planner library consists of 2 planners: 
1) RandomPlanner 
2) OptimalPlanner.

The following file calls 2 tests:

1) NormalTest():
The Normal test creates 5 scenarios including trivial, corner cases and general cases. It tests
both the planners on these cases.
TestCase 1: Trivial with same node for robot and goal
TestCase 2: Goal in the place of an obstacle. Invalid 
TestCase 3: Goal in close vicinity
TestCase 4: Goal in the same column as robot but at the end of the map
TestCase 5: General case

2) StressTest():
The stress test uses the test case as given in the documentation robot_pose at (2,0) and goal_pose
at (5,5). It runs both the planners on the same map 100 times. The total number of times each planner
accomplishes to find the path is counted and a percentage efficiency of the planners is obtained.

The map in both test cases is the same. It is is a 2D-grid representation of the environment where 
the value 0 indicates a navigable space and the value 1 indicates an occupied/obstacle space.The 
start and goal poses are provided as tuples. The path is returned from the planners as a vector 
of tuples. 

To bring in random seed into the planners the objects are instantiated inside the loop for every 
iteration. For the purpose of readability, map and path visualization are enabled in the normal test
and disabled in the stress test.

*/

#include <iostream>
#include <tuple>
#include <vector>

#include "DiscretePlanner.h"
#include "OptimalPlanner.h"
#include "RandomPlanner.h"

void NormalTest() {
	printf("Tap to begin Normal Test\n");
	std::cin.get();

	std::vector<std::vector<int>> world = { { 0, 0, 1, 0, 0, 0 },
											{ 0, 0, 1, 0, 0, 0 },
											{ 0, 0, 0, 0, 1, 0 },
											{ 0, 0, 0, 0, 1, 0 },
											{ 0, 0, 1, 1, 1, 0 },
											{ 0, 0, 0, 0, 0, 0 } };

	std::vector<std::pair<std::tuple<int, int>, std::tuple<int, int>>> testCases;
	std::tuple<int, int> robot_pose;
	std::tuple<int, int> goal_pose;

	robot_pose = std::make_tuple(2, 0);
	goal_pose = std::make_tuple(2, 0);
	testCases.push_back(std::make_pair(robot_pose, goal_pose));

	robot_pose = std::make_tuple(2, 0);
	goal_pose = std::make_tuple(4, 4);
	testCases.push_back(std::make_pair(robot_pose, goal_pose));

	robot_pose = std::make_tuple(2, 0);
	goal_pose = std::make_tuple(3, 2);
	testCases.push_back(std::make_pair(robot_pose, goal_pose));

	robot_pose = std::make_tuple(5, 5);
	goal_pose = std::make_tuple(0, 5);
	testCases.push_back(std::make_pair(robot_pose, goal_pose));

	robot_pose = std::make_tuple(5, 5);
	goal_pose = std::make_tuple(2, 2);
	testCases.push_back(std::make_pair(robot_pose, goal_pose));

	std::srand(time(NULL));
	for (int i = 0; i < testCases.size(); i++) {
		printf("Testcase: %d", i + 1);

		robot_pose = testCases[i].first;
		goal_pose = testCases[i].second;

		RandomPlanner random_planner;
		random_planner.visualize = true;
		std::vector<std::tuple<int, int>> RandomPath = random_planner.search(world, robot_pose, goal_pose);

		OptimalPlanner optimal_planner;
		optimal_planner.visualize = true;
		std::vector<std::tuple<int, int>> OptimalPath = optimal_planner.search(world, robot_pose, goal_pose);

		printf("===============================================================================\n");
	}

	printf("Normal test done.\n");
	std::cin.get();

};

void StressTest() {
	printf("Tap to begin Stress Test\n");
	std::cin.get();
	std::vector<std::vector<int>> world = { { 0, 0, 1, 0, 0, 0 },
											{ 0, 0, 1, 0, 0, 0 },
											{ 0, 0, 0, 0, 1, 0 },
											{ 0, 0, 0, 0, 1, 0 },
											{ 0, 0, 1, 1, 1, 0 },
											{ 0, 0, 0, 0, 0, 0 } };

	std::vector<std::pair<std::tuple<int, int>, std::tuple<int, int>>> testCases;
	std::tuple<int, int> robot_pose = std::make_tuple(2, 0);
	std::tuple<int, int> goal_pose = std::make_tuple(5, 5);

	std::srand(time(NULL));
	int countRandom = 0;
	int countOptimal = 0;
	for (int i = 0; i < 100; i++) {
		printf("Testcase: %d", i + 1);

		RandomPlanner random_planner;
		random_planner.visualize = false;
		std::vector<std::tuple<int, int>> RandomPath = random_planner.search(world, robot_pose, goal_pose);
		if (random_planner.pathFound)
			countRandom++;

		OptimalPlanner optimal_planner;
		optimal_planner.visualize = false;
		std::vector<std::tuple<int, int>> OptimalPath = optimal_planner.search(world, robot_pose, goal_pose);
		if (optimal_planner.pathFound)
			countOptimal++;

		printf("===============================================================================\n");
	}
	printf("Percentage efficiency of Random Planner: %d\n", countRandom);
	printf("Percentage efficiency of Optimal Planner: %d\n", countOptimal);

	printf("Stress test done.\n");
	std::cin.get();
};

int main() {

	NormalTest();

	StressTest();

	return 0;
}