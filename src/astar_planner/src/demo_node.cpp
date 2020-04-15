#include <math.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include "AstarPlanner/Astar_searcher.h"

using namespace Eigen;

int main(int argc, char** argv) {
  ROS_INFO("enter main function");
  ros::init(argc, argv, "a_star_node");
  AstarPathFinder* astar_path_finder = new AstarPathFinder();
  // astar_path_finder->makePlan();
  ros::spin();
  delete astar_path_finder;
  return 0;
}
