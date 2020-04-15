#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <iostream>
#include "node.h"

using namespace Eigen;

struct Point2D {
  double x, y;
};

class AstarPathFinder {
 private:
  ros::NodeHandle nh;
  Vector2d start_pt;
  Vector2d goal_pt;
  Vector2d map_lower, map_upper;
  int max_x_id, max_y_id;
  nav_msgs::OccupancyGrid map_;
  // ros related
  ros::Subscriber map_sub, start_sub, goal_sub;
  ros::Publisher _grid_path_vis_pub, _visited_nodes_vis_pub, start_pub,
      goal_pub;

  /// The start pose set through RViz
  geometry_msgs::PoseWithCovarianceStamped start;
  /// The goal pose set through RViz
  geometry_msgs::PoseStamped goal;
  /// Flags for allowing the planner to plan
  bool validStart = false;
  /// Flags for allowing the planner to plan
  bool validGoal = false;
  bool validMap = false;
  std::vector<Vector2d> gridPath_;
  std::vector<Vector2d> visitedNotes_;
  visualization_msgs::MarkerArray path_in_rviz_;
  visualization_msgs::MarkerArray visited_notes_in_rviz_;
  ;

 protected:
  uint8_t* data;            // this 1D array store binary grid information
  Node2DPtr** GridNodeMap;  // this 2D array store Node2D
  Eigen::Vector2i goalIdx;
  int X_SIZE, Y_SIZE, grid_number;

  double resolution, inv_resolution;
  double xl, yl;
  double xu, yu;

  Node2DPtr terminatePtr;
  std::multimap<double, Node2DPtr> openSet;

  double getHeu(Node2DPtr node1, Node2DPtr node2);
  void AstarGetSucc(Node2DPtr currentPtr,
                    std::vector<Node2DPtr>& neighborPtrSets,
                    std::vector<double>& edgeCostSets);

  bool isOccupied(const int& idx_x, const int& idx_y) const;
  bool isOccupied(const Eigen::Vector2i& index) const;
  bool isFree(const int& idx_x, const int& idx_y) const;
  bool isFree(const Eigen::Vector2i& index) const;

  Eigen::Vector2d gridIndex2coord(const Eigen::Vector2i& index);
  Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d& pt);

 public:
  AstarPathFinder();
  ~AstarPathFinder(){};
  void AstarGraphSearch();
  void resetGrid(Node2DPtr ptr);
  void resetUsedGrids();

  void initGridMap();

  Eigen::Vector2d coordRounding(const Eigen::Vector2d& coord);
  std::vector<Eigen::Vector2d> getPath();
  std::vector<Eigen::Vector2d> getVisitedNodes();

  void setMapCallBack(const nav_msgs::OccupancyGrid::Ptr map);
  void setStartCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);
  void setGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& end);
  void makePlan();
  void visGridPath();
  void visVisitedNode();
  void deleteVariablesData();
};

#endif