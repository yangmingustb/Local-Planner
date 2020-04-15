//
// Created by ustb on 19-7-8.
//

#ifndef LATTICEPLANNER_LATTICEPLANNER_H
#define LATTICEPLANNER_LATTICEPLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <vector>

// #include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ctime>
#include <iostream>
#include <stack>

#include "../FrenetMath/cartesianToFrenet.h"
#include "../FrenetMath/collisionChecking.h"
#include "../FrenetMath/costFunctions.h"
#include "selfType.h"

namespace lattice_planner {

/**
 * dijkstra planning in the path lattice
 */
class Dijkstra {
 public:
  Dijkstra() = default;
  Dijkstra(std::vector<CubicCoefficients> &coefficients);

  /**
   * @brief Main algorithm of Dijkstra.
   */
  void makePlan();

  /**
  * calculate id of the vertices in the path lattice
  * @param p
  * @return
  */
  int calIndex(Node p);
  bool nodeIsInClosed(Node &p);
  Node minCostInOpen();
  bool NodeInOpen(Node &p, std::vector<Node>::iterator &it);
  void DeleteOpenNode(Node p);
  void determineGoal();
  void pathTrace(std::stack<Node> &path);

  /**
  * @brief Get permissible motion primatives for the bot
  * @return vector of permissible motions
  */
  std::vector<Node> GetNextMotion();

 private:
  ros::NodeHandle nh;
  std::vector<Node> open_list_;
  std::vector<Node> closed_list_;
  Node nodeStart_, goal_;
  std::vector<CubicCoefficients> coefficients_;
  // std::vector<geometry_msgs::PoseStamped> optimal_path, path_lattice;
  // sampling poses from the Frenet coordinate system
  std::vector<FrenetPose> samplingPoses;

  std::vector<geometry_msgs::PoseStamped> path_lattice;
  sensor_msgs::PointCloud2 cloud_msg;
  // pcl::PointCloud<pcl::PointXYZ> cloud;
  visualization_msgs::MarkerArray obstaclesInRviz_;
  visualization_msgs::MarkerArray vehiclePathInRviz_;
  nav_msgs::Path optimal_path_;
  nav_msgs::Path ref_line_;

  geometry_msgs::PoseWithCovarianceStamped cartesianStart_;
  FrenetPose FrenetStart_;
  ros::Publisher path_pub;
  ros::Publisher refLine_pub;
  // publish lattice
  // ros::Publisher lattice_pub = nh.advertise<nav_msgs::Path>("lattice",1,
  // true);

  ros::Publisher lattice_pub_;
  // std::vector<ros::Publisher> obs_pubs;
  ros::Publisher obss_pub;
  ros::Subscriber SubStart_;
  ros::Publisher pub_startInRviz_;
  ros::Publisher pub_vehiclePathInRviz_;
  double begin_time_;
  double running_time_;

  bool isValid_;
  cartesianToFrenet crtToFrt_;
  frenetToCartesian frtToCrt_;
  costFunction cost_;

  // std::vector<CartesianPoint> roadLeftBorder_;
  // std::vector<CartesianPoint> roadRightBorder_;

  sensor_msgs::PointCloud2 roadLeftBorder_;
  sensor_msgs::PointCloud2 roadRightBorder_;
  ros::Publisher roadLeftBorderPub_;
  ros::Publisher roadRightBorderPub_;

 public:
  void samplingNodes();
  void generatePath();
  void generateLattice();
  void generateRefLine();
  void generateRoadBorder();

  void ShowRefLineInRviz();
  void ShowObstaclesInRviz();
  void setStart_callback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial);

  bool makeCollisionChecking();

  /**
   * 启动程序时，从默认的起点规划一次
   */
  void defaultStart();
  void showVehiclePathInRviz();
  void showRoadBorderInRviz();

 private:
  double rhoLeft = 3;
  double rhoRight = -3;
  double r_circle = 1.0;
  double d_circle = 2.0;
  double obstacle_inflation = 1.5;
  int longitudinal_num = 5;
  int lateral_num = 20;  // 横向采样个数
  double longitudinal_step = 20.0;
  double lateral_step = 0.5;
  double lane_width_ = 3.75;
  int SampleNumberOnOneSide;  // sampling number on one side of the reference
                              // line
  double s0;
  double s_max;
  double s_end;
  double refLineRho_;

  // the frenet coordinates of obstacles
  FrenetPose obs1, obs2, obs3;
  std::vector<FrenetPose> obstacles_;

  double obstacleHeading = 0.0 * M_PI / 180.0;

  // 最后一列的编号
  std::vector<int> last_column_id;
  double vehicle_body_fast_check_circle =
      3.0;  // body collision fast check circle, the radius is 3.0 m
  double vehicle_body_envelope_circle =
      1.0;  // little circles, the radius is 1.0 m

  bool showLatticeFlag = true;
  bool collisionCheckingFlag = true;

  double roadWidth_;
};

}  // namespace lattice_planner

#endif  // LATTICEPLANNER_LATTICEPLANNER_H
