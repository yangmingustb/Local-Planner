#include "AstarPlanner/Astar_searcher.h"

using namespace std;

AstarPathFinder::AstarPathFinder() {
  ROS_INFO("enter a star construction");

  // if no map, use this default map
  max_x_id = 50;
  max_y_id = 50;
  map_lower << -25.0, -25.0;
  map_upper << +25.0, +25.0;
  resolution = 1.0;
  start_pt(0) = 0.0;
  start_pt(1) = 0.0;
  goal_pt(0) = 0.0;
  goal_pt(1) = 0.0;
  initGridMap();

  map_sub = nh.subscribe("/map", 10, &AstarPathFinder::setMapCallBack, this);
  start_sub = nh.subscribe("/initialpose", 10,
                           &AstarPathFinder::setStartCallback, this);
  goal_sub = nh.subscribe("/move_base_simple/goal", 10,
                          &AstarPathFinder::setGoalCallback, this);
  start_pub = nh.advertise<geometry_msgs::PoseStamped>("startInRviz", 1, true);
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("goalInRviz", 1, true);
  _grid_path_vis_pub =
      nh.advertise<visualization_msgs::MarkerArray>("grid_path_vis", 1, true);
  _visited_nodes_vis_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "visited_nodes_vis", 1, true);
}

void AstarPathFinder::setMapCallBack(const nav_msgs::OccupancyGrid::Ptr map) {
  ROS_INFO("enter setmapCallback function");
  map_ = *map;
  ROS_INFO("setMapCallBack function is over");

  max_x_id = map_.info.width;
  max_y_id = map_.info.height;
  resolution = map_.info.resolution;

  ROS_INFO("xsize, %d, ysize,%d", max_x_id, max_y_id);
  map_lower(0) = -double(max_x_id) * resolution / 2.0;
  map_lower(1) = -double(max_y_id) * resolution / 2.0;
  map_upper(0) = double(max_x_id) * resolution / 2.0,
  map_upper(1) = double(max_y_id) * resolution / 2.0;

  ROS_INFO("map_lower, %f,%f", map_lower(0), map_lower(0));
  initGridMap();

  validMap = true;
  makePlan();
}

void AstarPathFinder::initGridMap() {
  ROS_INFO("grid map initialing");
  xl = map_lower(0);
  yl = map_lower(1);
  xu = map_upper(0);
  yu = map_upper(1);
  X_SIZE = max_x_id;
  Y_SIZE = max_y_id;
  grid_number = X_SIZE * Y_SIZE;
  inv_resolution = 1.0 / resolution;
  data = new uint8_t[grid_number];
  ROS_INFO("xl:%f ,yl,%f, xu,%f, yu,%f, XSIZE, %d, ySIZE, %d, reselution, %f",
           xl, yl, xu, yu, X_SIZE, Y_SIZE, resolution);
  memset(data, 0, grid_number * sizeof(uint8_t));
  if (map_.data.size() > 0) {
    for (int x = 0; x < X_SIZE; ++x) {
      for (int y = 0; y < Y_SIZE; ++y) {
        data[y * X_SIZE + x] = map_.data[y * X_SIZE + x] ? 1 : 0;
      }
    }
  }
  GridNodeMap = new Node2DPtr*[X_SIZE];
  for (int i = 0; i < X_SIZE; i++) {
    GridNodeMap[i] = new Node2DPtr[Y_SIZE];
    for (int k = 0; k < Y_SIZE; k++) {
      Vector2i tmpIdx(i, k);
      Vector2d pos = gridIndex2coord(tmpIdx);
      GridNodeMap[i][k] = new Node2D(tmpIdx, pos);
    }
  }
  ROS_INFO("grid map initialization is over");
}

void AstarPathFinder::resetGrid(Node2DPtr ptr) {
  ptr->id = 0;
  ptr->predecessor = NULL;
  ptr->gScore = inf;
  ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids() {
  for (int i = 0; i < X_SIZE; i++)
    for (int j = 0; j < Y_SIZE; j++) resetGrid(GridNodeMap[i][j]);
}

vector<Vector2d> AstarPathFinder::getVisitedNodes() {
  vector<Vector2d> visited_nodes;
  visited_nodes.clear();
  for (int i = 0; i < X_SIZE; i++)
    for (int j = 0; j < Y_SIZE; j++)
      // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and
      // close list
      if (GridNodeMap[i][j]->id == -1)  // visualize nodes in close list only
        visited_nodes.push_back(GridNodeMap[i][j]->coord);
  ROS_WARN("visited_nodes size : %d", visited_nodes.size());
  return visited_nodes;
}

Vector2d AstarPathFinder::gridIndex2coord(const Vector2i& index) {
  Vector2d pt;
  pt(0) = ((double)index(0) + 0.5) * resolution + xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + yl;
  return pt;
}

Vector2i AstarPathFinder::coord2gridIndex(const Vector2d& pt) {
  Vector2i idx;
  idx << min(max(int((pt(0) - xl) * inv_resolution), 0), X_SIZE - 1),
      min(max(int((pt(1) - yl) * inv_resolution), 0), Y_SIZE - 1);

  return idx;
}

Eigen::Vector2d AstarPathFinder::coordRounding(const Eigen::Vector2d& coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector2i& index) const {
  return isOccupied(index(0), index(1));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector2i& index) const {
  return isFree(index(0), index(1));
}

inline bool AstarPathFinder::isOccupied(const int& idx_x,
                                        const int& idx_y) const {
  return (idx_x >= 0 && idx_x < X_SIZE && idx_y >= 0 && idx_y < Y_SIZE &&
          (data[idx_x + idx_y * X_SIZE] == 1));
}

inline bool AstarPathFinder::isFree(const int& idx_x, const int& idx_y) const {
  return (idx_x >= 0 && idx_x < X_SIZE && idx_y >= 0 && idx_y < Y_SIZE &&
          (data[idx_x + idx_y * X_SIZE] < 1));
}

inline void AstarPathFinder::AstarGetSucc(Node2DPtr currentPtr,
                                          vector<Node2DPtr>& neighborPtrSets,
                                          vector<double>& edgeCostSets) {
  neighborPtrSets.clear();
  edgeCostSets.clear();
  Vector2i neighborIdx;
  for (int dx = -1; dx < 2; dx++) {
    for (int dy = -1; dy < 2; dy++) {
      if (dx == 0 && dy == 0) continue;
      neighborIdx(0) = (currentPtr->index)(0) + dx;
      neighborIdx(1) = (currentPtr->index)(1) + dy;
      if (neighborIdx(0) < 0 || neighborIdx(0) >= X_SIZE ||
          neighborIdx(1) < 0 || neighborIdx(1) >= Y_SIZE) {
        continue;
      }
      neighborPtrSets.push_back(GridNodeMap[neighborIdx(0)][neighborIdx(1)]);
      edgeCostSets.push_back(sqrt(dx * dx + dy * dy));
    }
  }
}

double AstarPathFinder::getHeu(Node2DPtr node1, Node2DPtr node2) {
  // choose possible heuristic function you want
  // Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
  double dx = abs(node1->index(0) - node2->index(0));
  double dy = abs(node1->index(1) - node2->index(1));
  double tie_breaker = 1 + 1 / 10000;

  double h = 0.0;
  int diag = min(dx, dy);
  dx -= diag;
  dy -= diag;

  if (dx == 0) {
    h = 1.0 * sqrt(2.0) * diag + 1.0 * abs(dy);
  }
  if (dy == 0) {
    h = 1.0 * sqrt(2.0) * diag + 1.0 * abs(dx);
  }

  return tie_breaker * h;
}

void AstarPathFinder::AstarGraphSearch() {
  ROS_INFO("searching");
  ROS_INFO("start:[%f,%f]", start_pt(0), start_pt(1));
  ROS_INFO("goal:[%f,%f]", goal_pt(0), goal_pt(1));
  ros::Time time_1 = ros::Time::now();

  // index of start_point and end_point
  Vector2i start_idx = coord2gridIndex(start_pt);
  Vector2i end_idx = coord2gridIndex(goal_pt);
  ROS_INFO("start id:[%d,%d]", start_idx(0), start_idx(1));
  ROS_INFO("goal id:[%d,%d]", end_idx(0), end_idx(1));
  goalIdx = end_idx;

  // position of start_point and end_point
  start_pt = gridIndex2coord(start_idx);
  goal_pt = gridIndex2coord(end_idx);

  // Initialize the pointers of struct GridNode which represent start node and
  // goal node
  Node2DPtr startPtr = new Node2D(start_idx, start_pt);
  Node2DPtr endPtr = new Node2D(end_idx, goal_pt);

  // currentPtr represents the node with lowest f(n) in the open_list
  Node2DPtr currentPtr = NULL;
  Node2DPtr neighborPtr = NULL;

  // put start node in open set
  startPtr->gScore = 0;
  startPtr->fScore = getHeu(startPtr, endPtr) + startPtr->gScore;
  startPtr->id = 1;  // id is a flag, in open list or not
  startPtr->coord = start_pt;
  openSet.insert(make_pair(startPtr->fScore, startPtr));

  double tentative_gScore;
  vector<Node2DPtr> neighborPtrSets;
  vector<double> edgeCostSets;

  // this is the main loop
  while (!openSet.empty()) {
    /*

    IMPORTANT NOTE!!!
    This part you should use the C++ STL: multimap, more details can be find in
    Homework description
    *
    *
    */
    currentPtr = openSet.begin()->second;
    openSet.erase(openSet.begin());
    currentPtr->id = -1;
    // if the current node is the goal
    if (currentPtr->index == goalIdx) {
      ros::Time time_2 = ros::Time::now();
      terminatePtr = currentPtr;
      ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost is %f m",
               (time_2 - time_1).toSec() * 1000.0,
               currentPtr->gScore * resolution);
      return;
    }
    // get the successors
    AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

    for (int i = 0; i < (int)neighborPtrSets.size(); i++) {
      /*
      IMPORTANT NOTE!!!
      neighborPtrSets[i]->id = -1 : unexpanded
      neighborPtrSets[i]->id = 1 : expanded, equal to this node is in close set
      *
      */
      neighborPtr = neighborPtrSets[i];
      if (isOccupied(neighborPtr->index) || neighborPtr->id == -1) continue;

      double edge_cost = edgeCostSets[i];
      tentative_gScore = currentPtr->gScore + edge_cost;

      if (neighborPtr->id != 1) {  // discover a new node

        neighborPtr->id = 1;
        neighborPtr->predecessor = currentPtr;
        neighborPtr->gScore = tentative_gScore;
        neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
        neighborPtr->nodeMapIt = openSet.insert(
            make_pair(neighborPtr->fScore,
                      neighborPtr));  // put neighbor in open set and record it.
        continue;
      } else if (tentative_gScore <=
                 neighborPtr->gScore) {  // in open set and need update

        neighborPtr->predecessor = currentPtr;
        neighborPtr->gScore = tentative_gScore;
        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
        openSet.erase(neighborPtr->nodeMapIt);
        neighborPtr->nodeMapIt = openSet.insert(
            make_pair(neighborPtr->fScore,
                      neighborPtr));  // put neighbor in open set and record it.
        continue;
      }
    }
  }

  // if search fails
  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
}

vector<Vector2d> AstarPathFinder::getPath() {
  vector<Vector2d> path;
  vector<Node2DPtr> gridPath;
  // trace back from the curretnt nodePtr to get all nodes along the path
  gridPath.push_back(terminatePtr);
  auto currentPtr = terminatePtr;
  while (currentPtr->predecessor != NULL) {
    currentPtr = currentPtr->predecessor;
    gridPath.push_back(currentPtr);
  }

  for (auto ptr : gridPath) path.push_back(ptr->coord);

  reverse(path.begin(), path.end());

  return path;
}

void AstarPathFinder::setStartCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& begin) {
  ROS_INFO("enter set start callback function.");
  ROS_INFO("------------------------------");
  ROS_INFO("received a start pose from the Rviz.");
  // double x = initial->pose.pose.position.x;
  // double y = initial->pose.pose.position.y;
  // double yaw = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz

  geometry_msgs::PoseStamped startRviz;
  startRviz.pose.position = begin->pose.pose.position;
  startRviz.pose.orientation = begin->pose.pose.orientation;
  startRviz.header.frame_id = "/map";
  startRviz.header.stamp = ros::Time::now();
  validStart = true;
  // publish start for RViz
  start_pub.publish(startRviz);
  start = *begin;
  ROS_INFO("[node] receive the planning start");

  makePlan();
}

void AstarPathFinder::setGoalCallback(
    const geometry_msgs::PoseStamped::ConstPtr& end) {
  ROS_INFO("enter set goal callback function.");
  ROS_INFO("------------------------------");
  ROS_INFO("received a goa pose from the Rviz.");
  // double x = initial->pose.pose.position.x;
  // double y = initial->pose.pose.position.y;
  // double yaw = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz

  geometry_msgs::PoseStamped goalRviz;
  goalRviz.pose.position = end->pose.position;
  goalRviz.pose.orientation = end->pose.orientation;
  goalRviz.header.frame_id = "/map";
  goalRviz.header.stamp = ros::Time::now();
  validGoal = true;
  goal = *end;
  goal_pub.publish(goalRviz);

  ROS_INFO("[node] receive the planning target");
  makePlan();
}

void AstarPathFinder::makePlan() {
  if (validGoal && validStart && validMap) {
    ROS_INFO("enter make plan function");

    double x = goal.pose.position.x;
    double y = goal.pose.position.y;
    goal_pt(0) = x;
    goal_pt(1) = y;

    // _________________________
    // retrieving start position
    x = start.pose.pose.position.x;
    y = start.pose.pose.position.y;
    start_pt(0) = x;
    start_pt(1) = y;
    ROS_INFO("start:[%f,%f]", start_pt(0), start_pt(1));
    ROS_INFO("goal:[%f,%f]", goal_pt(0), goal_pt(1));
    // Call A* to search for a path
    deleteVariablesData();
    AstarGraphSearch();

    // Retrieve the path
    gridPath_ = getPath();
    visitedNotes_ = getVisitedNodes();

    // Visualize the result
    visVisitedNode();
    visGridPath();
    // Reset map for next call
    resetUsedGrids();

  } else {
    ROS_INFO("missing valid goal or start");
  }
}

void AstarPathFinder::visGridPath() {
  visualization_msgs::Marker pathVehicle;
  int id = 0;
  for (auto i = 0; i < gridPath_.size(); i++) {
    pathVehicle.header.frame_id = "/map";
    pathVehicle.header.stamp = ros::Time(0);
    pathVehicle.id = id;
    pathVehicle.type = visualization_msgs::Marker::CUBE;

    pathVehicle.scale.x = resolution;
    pathVehicle.scale.y = resolution;
    pathVehicle.scale.z = resolution;
    pathVehicle.color.a = 0.5;
    pathVehicle.color.r = 255.0 / 1.0;
    pathVehicle.color.g = 0.0 / 255.0;
    pathVehicle.color.b = 0.0 / 255.0;

    pathVehicle.pose.position.x = gridPath_[i](0);
    pathVehicle.pose.position.y = gridPath_[i](1);
    pathVehicle.pose.position.z = 0;
    path_in_rviz_.markers.push_back(pathVehicle);
    id++;
  }

  _grid_path_vis_pub.publish(path_in_rviz_);
  ROS_INFO("pub path in rviz");
  ROS_INFO("path note number, %d", path_in_rviz_.markers.size());
}

void AstarPathFinder::visVisitedNode() {
  visualization_msgs::Marker pathVehicle;
  int id = 0;
  for (auto i = 0; i < visitedNotes_.size(); i++) {
    pathVehicle.header.frame_id = "/map";
    pathVehicle.header.stamp = ros::Time(0);
    pathVehicle.id = id;
    pathVehicle.type = visualization_msgs::Marker::CUBE;

    pathVehicle.scale.x = resolution;
    pathVehicle.scale.y = resolution;
    pathVehicle.scale.z = resolution;
    pathVehicle.color.a = 0.5;
    pathVehicle.color.r = 0.0 / 255.0;
    pathVehicle.color.g = 255.0 / 255.0;
    pathVehicle.color.b = 2550.0 / 255.0;

    pathVehicle.pose.position.x = visitedNotes_[i](0);
    pathVehicle.pose.position.y = visitedNotes_[i](1);
    pathVehicle.pose.position.z = 0;
    visited_notes_in_rviz_.markers.push_back(pathVehicle);
    id++;
  }
  _visited_nodes_vis_pub.publish(visited_notes_in_rviz_);
  ROS_INFO("pub visted note in rviz");
  ROS_INFO("visited note number, %d", visited_notes_in_rviz_.markers.size());
  // grids.markers.clear();
}

void AstarPathFinder::deleteVariablesData() {
  path_in_rviz_.markers.clear();
  visited_notes_in_rviz_.markers.clear();
  gridPath_.clear();
  visitedNotes_.clear();
  // openSet is the open_list implemented through multimap in STL library
  openSet.clear();
  terminatePtr = NULL;
}