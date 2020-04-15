//
// Created by ustb on 19-7-8.
//

#include "latticePlanner.h"
#include <array>
#include <iomanip>
#include <queue>

#include <stdlib.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <typeinfo>

namespace lattice_planner {

Dijkstra::Dijkstra(std::vector<CubicCoefficients> &coefficients) {
  ROS_INFO("lattice planner is initialing..");
  coefficients_ = coefficients;

  ROS_INFO("lookup table size:%d", coefficients_.size());
  path_pub = nh.advertise<nav_msgs::Path>("optimal_path", 1, true);
  lattice_pub_ = nh.advertise<sensor_msgs::PointCloud2>("lattice", 1, true);
  refLine_pub = nh.advertise<nav_msgs::Path>("ref_line", 1, true);
  obss_pub =
      nh.advertise<visualization_msgs::MarkerArray>("obstacles", 100, true);
  pub_vehiclePathInRviz_ = nh.advertise<visualization_msgs::MarkerArray>(
      "vehiclePathInRviz", 1, true);
  roadLeftBorderPub_ =
      nh.advertise<sensor_msgs::PointCloud2>("roadLeftBorder", 1, true);
  roadRightBorderPub_ =
      nh.advertise<sensor_msgs::PointCloud2>("roadRightBorder", 1, true);

  nh.getParam("/lattice_planner/showLatticeFlag", showLatticeFlag);
  nh.getParam("/lattice_planner/r_circle", r_circle);
  nh.getParam("/lattice_planner/d_circle", d_circle);
  nh.getParam("/lattice_planner/obstacle_inflation", obstacle_inflation);
  nh.getParam("/localPlanner/longitudinal_num", longitudinal_num);
  nh.getParam("/localPlanner/lateral_num", lateral_num);
  nh.getParam("/lattice_planner/longitudinal_step", longitudinal_step);
  nh.getParam("/lattice_planner/lateral_step", lateral_step);
  nh.getParam("/lattice_planner/lane_width", lane_width_);
  ROS_INFO("lattice_planner/lateral_num: %d", lateral_num);

  SampleNumberOnOneSide =
      lateral_num / 2;  // sampling number on one side of the reference line
  // s0 = FrenetStart_.s;
  s_max = longitudinal_num * longitudinal_step;
  // s_end = s0 + s_max;
  refLineRho_ = 0.0;
  obstacleHeading = 0;
  // the frenet coordinates of obstacles

  obs1.s = 20.0;
  obs1.rho = refLineRho_ - 1;
  obs1.heading = obstacleHeading;

  obs2.s = 40.0;
  obs2.rho = refLineRho_ + 1;
  obs2.heading = obstacleHeading;
  obs3.s = 70.0;
  obs3.rho = refLineRho_ - 1;
  obs3.heading = obstacleHeading;
  obstacles_.push_back(obs1);
  obstacles_.push_back(obs2);
  obstacles_.push_back(obs3);

  roadWidth_ = 10.0;

  // 最后一列的编号
  last_column_id = {lateral_num * (longitudinal_num - 1) + 1,
                    lateral_num * longitudinal_num};

  frtToCrt_.setParameters(coefficients_);
  cost_.setParameters(coefficients_, refLineRho_, obstacles_, frtToCrt_);
  generateRefLine();
  crtToFrt_.setParameters(ref_line_, coefficients_);

  generateRoadBorder();
  showRoadBorderInRviz();
  defaultStart();
  SubStart_ =
      nh.subscribe("/initialpose", 100, &Dijkstra::setStart_callback, this);
  ROS_INFO("----lattice planner has been initialized---");
}

std::vector<Node> Dijkstra::GetNextMotion() {
  std::vector<Node> motion;
  for (int i = 0; i < lateral_num; i++) {
    Node tmp_motion(longitudinal_step,
                    (i - SampleNumberOnOneSide) * lateral_step + refLineRho_,
                    0.0, 0, 0);
    motion.push_back(tmp_motion);
  }

  //    cout<<"-------output motion-------------"<<endl;
  //    for(auto m:motion){
  //        cout<<m.x_<<"-"<<m.y_<<endl;
  //    }
  return motion;
}

int Dijkstra::calIndex(Node p) {
  double id = (p.rho_ - (refLineRho_ - SampleNumberOnOneSide * lateral_step)) /
                  lateral_step +
              ((p.s_ - nodeStart_.s_) / longitudinal_step - 1.0) * lateral_num +
              1.0;
  return int(id);
}

bool Dijkstra::nodeIsInClosed(Node &p) {
  for (auto k : closed_list_) {
    if (k.id_ == p.id_) return true;
  }
  return false;
}

Node Dijkstra::minCostInOpen() {
  auto p = *open_list_.begin();
  for (auto o : open_list_) {
    if (p.cost_ > o.cost_) p = o;
  }
  return p;
}

bool Dijkstra::NodeInOpen(Node &p, std::vector<Node>::iterator &it) {
  for (auto o = open_list_.begin(); o != open_list_.end(); o++) {
    if (o->id_ == p.id_) {
      it = o;
      return true;
    }
  }
  return false;
}

void Dijkstra::DeleteOpenNode(Node p) {
  for (auto o = open_list_.begin(); o != open_list_.end(); o++) {
    if (o->id_ == p.id_) {
      open_list_.erase(o);
      break;
    }
  }
}

void Dijkstra::makePlan() {
  std::vector<Node> motion = GetNextMotion();
  // std::cout<<"motion_size:"<<motion.size()<<std::endl;
  open_list_.push_back(nodeStart_);
  // cout<<open_list_.size()<<endl;

  // Main loop
  while (!open_list_.empty()) {
    Node current = minCostInOpen();
    // ROS_INFO("calculate the current node...");
    // std::cout<<"-------------current id-------------"<<std::endl;
    // std::cout<<current.id_<<std::endl;
    // std::cout<<"open_size:"<<open_list_.size()<<std::endl;
    closed_list_.push_back(current);
    for (auto i : motion) {
      Node new_point;
      // cout<<"current "<<current.x_<<endl;
      // cout<<"current address "<<&(current)<<endl;
      new_point.s_ = current.s_ + i.s_;
      new_point.rho_ = i.rho_;
      new_point.id_ = calIndex(new_point);
      // ROS_INFO("calculated the index...");
      new_point.pid_ = current.id_;
      new_point.cost_ = cost_.total_cost(current, new_point);

      // ROS_INFO("total cost has been calculated..");

      // cout<<"------------------newpoint.x--------"<<endl;
      // cout<<new_point.x_<<endl;
      if (new_point.s_ > s_end) break;

      auto it = open_list_.begin();
      bool flag = nodeIsInClosed(new_point);
      // ROS_INFO("nodeIsInClosed() method completed...");
      if (flag)
        continue;
      else if (NodeInOpen(new_point, it)) {
        if (it->cost_ > new_point.cost_) {
          it->cost_ = new_point.cost_;
          it->pid_ = current.id_;
        }
      } else
        open_list_.push_back(new_point);
    }
    DeleteOpenNode(current);
    // ROS_INFO("delete open list has been completed...");
  }

  ROS_INFO("MAKE PLAN completed... ");
}

void Dijkstra::determineGoal() {
  std::vector<Node> tmpVector;

  for (int i = *(last_column_id.cbegin());
       i != *(last_column_id.cbegin() + 1) + 1; i++) {
    for (auto j : closed_list_) {
      if (i == j.id_) {
        tmpVector.push_back(j);
        break;
      }
    }
  }

  goal_ = *tmpVector.begin();
  for (auto t : tmpVector) {
    if (goal_.cost_ > t.cost_) goal_ = t;
  }

  ROS_INFO("the goal is determined.");
}

void Dijkstra::pathTrace(std::stack<Node> &path) {
  path.push(goal_);

  int ptr = goal_.pid_;
  while (ptr != -1) {
    for (auto c : closed_list_) {
      if (c.id_ == ptr) {
        path.push(c);
        ptr = c.pid_;
        break;
      }
    }
  }
  ROS_INFO("path trace has been completed.");
}

void Dijkstra::generatePath() {
  ROS_INFO("GENERATE PATH...");
  makePlan();

  determineGoal();
  ROS_INFO("the Frenet coordinates of the goal. s:%f,rho:%f", goal_.s_,
           goal_.rho_);
  std::stack<lattice_planner::Node> pathNode;
  pathTrace(pathNode);

  // PoseCartesian cartesian_pose;
  geometry_msgs::PoseStamped cartesian_pose;

  // start cubic
  lattice_planner::FrenetPose start;
  start.s = pathNode.top().s_;
  start.rho = pathNode.top().rho_;
  start.heading = FrenetStart_.heading;

  pathNode.pop();
  lattice_planner::FrenetPose end;
  end.s = pathNode.top().s_;
  end.rho = pathNode.top().rho_;
  end.heading = 0.0 * M_PI / 180.0;

  CubicPolynomial cubic(start, end);
  FrenetPath frenet_path = cubic.FrenetCubicPolynomial();

  for (std::size_t i = 0; i < frenet_path.frtPath_.size(); i = i + 5) {
    cartesian_pose = frtToCrt_.transform(frenet_path.frtPath_[i]);

    optimal_path_.poses.push_back(cartesian_pose);
  }

  while (!pathNode.empty()) {
    lattice_planner::FrenetPose start;
    start.s = pathNode.top().s_;
    start.rho = pathNode.top().rho_;
    start.heading = 0.0 * M_PI / 180.0;

    pathNode.pop();
    if (pathNode.empty()) break;
    lattice_planner::FrenetPose end;
    end.s = pathNode.top().s_;
    end.rho = pathNode.top().rho_;
    end.heading = 0.0 * M_PI / 180.0;

    // std::cout<<"-----x,y-------------"<<std::endl;
    // std::cout<<start[0]<<","<<start[1]<<std::endl;
    CubicPolynomial cubic(start, end);
    FrenetPath frenet_path = cubic.FrenetCubicPolynomial();

    // std::vector<double> s_vec=*(set.begin());
    // std::vector<double> rho_vec=*(set.begin()+1);
    // std::vector<double> theta = *(set.begin()+2);

    for (std::size_t i = 0; i < frenet_path.frtPath_.size(); i = i + 5) {
      cartesian_pose = frtToCrt_.transform(frenet_path.frtPath_[i]);

      optimal_path_.poses.push_back(cartesian_pose);
    }
  }

  optimal_path_.header.stamp = ros::Time::now();
  // path.header.frame_id="/my_frame";
  optimal_path_.header.frame_id = "/map";

  if (collisionCheckingFlag) isValid_ = makeCollisionChecking();
  // else isValid_ = true;

  if (isValid_) {
    path_pub.publish(optimal_path_);
    running_time_ = (ros::Time::now().toSec() - begin_time_);
    ROS_INFO("%f secs for local_planner", running_time_);

    ROS_INFO("the generated path is valid.");
  } else {
    ROS_INFO("the generated path is invalid");
    optimal_path_.poses.clear();
    path_pub.publish(optimal_path_);
  }
  // return optimal_path;
}

void Dijkstra::samplingNodes() {
  FrenetPose tmp_pose;

  for (int i = 0; i < longitudinal_num; i++) {
    double x_i = (i + 1) * longitudinal_step + nodeStart_.s_;

    for (int j = 0; j < lateral_num; j++) {
      double y_i = (j - SampleNumberOnOneSide) * lateral_step + refLineRho_;
      // std::array<double, 3> tmp_node = {{x_i, y_i, 0.0 * M_PI / 180.0}};
      tmp_pose.s = x_i;
      tmp_pose.rho = y_i;
      tmp_pose.heading = 0.0 * M_PI / 180.0;
      samplingPoses.push_back(tmp_pose);
    }
    // Nodes.push_back(lateral_nodes);
  }
}

void Dijkstra::generateLattice() {
  samplingNodes();
  geometry_msgs::PoseStamped cartesian_pose;

  // 生成车辆起点到第一列采样点的图
  for (int i = 0; i < lateral_num; i++) {
    FrenetPose start;
    start.s = FrenetStart_.s;
    start.rho = FrenetStart_.rho;
    start.heading = FrenetStart_.heading;
    FrenetPose end;
    end.s = samplingPoses[i].s;
    end.rho = samplingPoses[i].rho;
    end.heading = samplingPoses[i].heading;

    CubicPolynomial cubic(start, end);
    FrenetPath frenet_path = cubic.FrenetCubicPolynomial();

    for (std::size_t i = 0; i < frenet_path.frtPath_.size(); i++) {
      cartesian_pose = frtToCrt_.transform(frenet_path.frtPath_[i]);
      path_lattice.push_back(cartesian_pose);
    }
  }

  // 采样点之间的图
  for (int i = 0; i < longitudinal_num - 1; i++) {
    for (int j = 0; j < lateral_num; j++) {
      for (int q = 0; q < lateral_num; q++) {
        FrenetPose start;
        start.s = samplingPoses[i * lateral_num + j].s;
        start.rho = samplingPoses[i * lateral_num + j].rho;
        start.heading = samplingPoses[i * lateral_num + j].heading;

        FrenetPose end;
        end.s = samplingPoses[(i + 1) * lateral_num + q].s;
        end.rho = samplingPoses[(i + 1) * lateral_num + q].rho;
        end.heading = samplingPoses[(i + 1) * lateral_num + q].heading;

        // std::cout<<"---------start end----------"<<std::endl;
        // std::cout<<"start x:"<<start[0]<<"end x: "<<end[0]<<std::endl;

        CubicPolynomial cubic(start, end);
        FrenetPath frenet_path = cubic.FrenetCubicPolynomial();

        for (std::size_t t = 0; t < frenet_path.frtPath_.size(); t++) {
          cartesian_pose = frtToCrt_.transform(frenet_path.frtPath_[t]);
          path_lattice.push_back(cartesian_pose);
        }
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // cloud->points.clear();
  for (int i = 0; i < path_lattice.size(); i++) {
    pcl::PointXYZ point;
    point.x = path_lattice[i].pose.position.x;
    point.y = path_lattice[i].pose.position.y;
    point.z = path_lattice[i].pose.position.z;
    cloud->points.push_back(point);
  }
  // lattice_pub.publish(lattice_path);

  pcl::toROSMsg(*cloud, cloud_msg);
  // output.header.frame_id = "odom";

  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = "/map";
  lattice_pub_.publish(cloud_msg);

  // delete cloud;
  // return path_lattice;
}

void Dijkstra::generateRefLine() {
  ref_line_.header.stamp = ros::Time::now();
  // path.header.frame_id="/my_frame";
  ref_line_.header.frame_id = "/map";

  for (auto i : coefficients_) {
    geometry_msgs::PoseStamped Refline_pose = frtToCrt_.getReflinePose(i.s);

    ref_line_.poses.push_back(Refline_pose);
  }
  // return ref_line_;
  ROS_INFO("reference line has been generated..");
}

void Dijkstra::setStart_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial) {
  begin_time_ = ros::Time::now().toSec();

  ROS_INFO("enter set start callback function.");
  ROS_INFO("------------------------------");
  ROS_INFO("received a start pose from the Rviz.");
  // double x = initial->pose.pose.position.x;
  // double y = initial->pose.pose.position.y;
  // double yaw = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz

  geometry_msgs::PoseStamped startRviz;
  startRviz.pose.position = initial->pose.pose.position;
  startRviz.pose.orientation = initial->pose.pose.orientation;
  startRviz.header.frame_id = "/map";
  startRviz.header.stamp = ros::Time::now();

  cartesianStart_ = *initial;

  pub_startInRviz_ =
      nh.advertise<geometry_msgs::PoseStamped>("startInRviz", 1, true);
  // publish start for RViz
  pub_startInRviz_.publish(startRviz);
  ROS_INFO("pub start in rviz.");

  FrenetStart_ = crtToFrt_.transform(cartesianStart_);

  nodeStart_.s_ = FrenetStart_.s;
  nodeStart_.rho_ = FrenetStart_.rho;
  nodeStart_.cost_ = 0.0;
  nodeStart_.id_ = 0;
  nodeStart_.pid_ = -1;
  s0 = FrenetStart_.s;

  s_end = s0 + s_max;

  optimal_path_.poses.clear();
  open_list_.clear();
  closed_list_.clear();
  path_lattice.clear();
  cloud_msg.fields.clear();
  samplingPoses.clear();
  vehiclePathInRviz_.markers.clear();

  ROS_INFO("optimal_path size:%d", optimal_path_.poses.size());

  generatePath();
  ROS_INFO("optimal_path size:%d", optimal_path_.poses.size());
  ShowRefLineInRviz();
  if (showLatticeFlag) generateLattice();
  ShowObstaclesInRviz();
  showVehiclePathInRviz();

  ROS_INFO("the set start callback function is completed.");
}

void Dijkstra::generateRoadBorder() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRight(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (auto i : ref_line_.poses) {
    pcl::PointXYZ point;

    double yaw = tf::getYaw(i.pose.orientation);
    point.x = i.pose.position.x + roadWidth_ / 2.0 * cos(yaw + M_PI / 2.0);
    point.y = i.pose.position.y + roadWidth_ / 2.0 * sin(yaw + M_PI / 2.0);
    point.z = 0.0;
    cloud->points.push_back(point);

    pcl::PointXYZ pointRight;
    pointRight.x = i.pose.position.x - roadWidth_ / 2.0 * cos(yaw + M_PI / 2.0);
    pointRight.y = i.pose.position.y - roadWidth_ / 2.0 * sin(yaw + M_PI / 2.0);
    pointRight.z = 0.0;
    cloudRight->points.push_back(pointRight);
  }

  pcl::toROSMsg(*cloud, roadLeftBorder_);

  pcl::toROSMsg(*cloudRight, roadRightBorder_);
  // output.header.frame_id = "odom";

  roadLeftBorder_.header.stamp = ros::Time::now();
  roadLeftBorder_.header.frame_id = "/map";
  roadRightBorder_.header.stamp = ros::Time::now();
  roadRightBorder_.header.frame_id = "/map";
}

void Dijkstra::ShowRefLineInRviz() {
  refLine_pub.publish(ref_line_);
  // return ref_line_;
}

void Dijkstra::ShowObstaclesInRviz() {
  // 设置初始形状为立方体
  uint32_t shape = visualization_msgs::Marker::CUBE;
  for (int i = 0; i != obstacles_.size(); i++) {
    // Create lines and points
    visualization_msgs::Marker obs;
    obs.header.frame_id = "/map";
    obs.header.stamp = ros::Time::now();
    obs.ns = "obstacles";
    obs.action = visualization_msgs::Marker::ADD;
    // obs.pose.orientation.w = 1.0;
    obs.id = i;
    obs.type = shape;

    geometry_msgs::PoseStamped poseCartesian =
        frtToCrt_.transform(obstacles_[i]);
    obs.pose = poseCartesian.pose;
    // obs.pose.position.x = poseCartesian.x;
    // obs.pose.position.y = poseCartesian.y;
    // obs.pose.position.z = 0;
    // obs.pose.orientation.x = 0.0;
    // obs.pose.orientation.y = 0.0;
    // obs.pose.orientation.z = 0.0;
    // obs.pose.orientation.w = 1.0;

    // 设置标记的比例，所有方向上尺度1表示1米
    obs.scale.x = 1.0;
    obs.scale.y = 1.0;
    obs.scale.z = 1.0;

    //设置标记颜色，确保alnha（不透明度）值不为0
    obs.color.r = 0.0f;
    obs.color.g = 1.0f;
    obs.color.b = 0.0f;
    obs.color.a = 1.0;

    obstaclesInRviz_.markers.push_back(obs);
  }

  obss_pub.publish(obstaclesInRviz_);
}

bool Dijkstra::makeCollisionChecking() {
  ROS_INFO("lattice plannnig's collision checking is called");
  CartesianPath ctsPath;
  CartesianPose p;

  // ROS_INFO("the length of the optimal path:%d", optimal_path_.poses.size());
  for (auto i = optimal_path_.poses.cbegin(); i != optimal_path_.poses.cend();
       i++) {
    p.x = i->pose.position.x;
    p.y = i->pose.position.y;

    tf::Quaternion q;
    tf::quaternionMsgToTF(i->pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    p.yaw = yaw;
    // p.yaw = tf::getYaw(i->pose.orientation);

    ctsPath.ctsPath_.push_back(p);
  }

  PointsObstacle obspoints;
  CartesianPoint ctsPoint;
  geometry_msgs::PoseStamped gPose;
  for (auto i : (obstacles_)) {
    gPose = frtToCrt_.transform(i);
    ctsPoint.x = gPose.pose.position.x;
    ctsPoint.y = gPose.pose.position.y;

    obspoints.points.push_back(ctsPoint);
  }
  CollisionChecker collision(ctsPath, obspoints, rhoLeft, rhoRight, crtToFrt_);
  isValid_ = collision.performChecking();

  return isValid_;
}

void Dijkstra::defaultStart() {
  begin_time_ = ros::Time::now().toSec();
  geometry_msgs::PoseStamped startRviz;
  startRviz.pose.position.x = 0.0;
  startRviz.pose.position.y = 0.0;
  geometry_msgs::Quaternion geo_q = tf::createQuaternionMsgFromYaw(80.0);
  startRviz.pose.orientation = geo_q;
  startRviz.header.frame_id = "/map";
  startRviz.header.stamp = ros::Time::now();

  cartesianStart_.pose.pose = startRviz.pose;

  pub_startInRviz_ =
      nh.advertise<geometry_msgs::PoseStamped>("startInRviz", 1, true);
  // publish start for RViz
  pub_startInRviz_.publish(startRviz);
  ROS_INFO("pub start in rviz.");

  FrenetStart_ = crtToFrt_.transform(cartesianStart_);

  nodeStart_.s_ = FrenetStart_.s;
  nodeStart_.rho_ = FrenetStart_.rho;
  nodeStart_.cost_ = 0.0;
  nodeStart_.id_ = 0;
  nodeStart_.pid_ = -1;
  s0 = FrenetStart_.s;

  s_end = s0 + s_max;

  optimal_path_.poses.clear();
  open_list_.clear();
  closed_list_.clear();
  path_lattice.clear();
  cloud_msg.fields.clear();
  samplingPoses.clear();
  vehiclePathInRviz_.markers.clear();

  ROS_INFO("optimal_path size:%d", optimal_path_.poses.size());

  generatePath();
  ROS_INFO("optimal_path size:%d", optimal_path_.poses.size());
  ShowRefLineInRviz();
  if (showLatticeFlag) generateLattice();
  ShowObstaclesInRviz();
  showVehiclePathInRviz();

  ROS_INFO("the default start need to be planned.");
}

void Dijkstra::showVehiclePathInRviz() {
  visualization_msgs::Marker pathVehicle;
  int id = 0;
  for (auto i = optimal_path_.poses.cbegin(); i < optimal_path_.poses.cend();
       i = i + 8) {
    pathVehicle.header.frame_id = "/map";
    pathVehicle.header.stamp = ros::Time(0);
    pathVehicle.id = id;
    pathVehicle.type = visualization_msgs::Marker::CUBE;

    pathVehicle.scale.x = 4.0;
    pathVehicle.scale.y = 1.6;
    pathVehicle.scale.z = 1.0;
    pathVehicle.color.a = 0.5;
    pathVehicle.color.r = 255.0 / 255.0;
    pathVehicle.color.g = 255.0 / 255.0;
    pathVehicle.color.b = 0.0 / 255.0;

    pathVehicle.pose = i->pose;
    vehiclePathInRviz_.markers.push_back(pathVehicle);
    id++;
  }

  pub_vehiclePathInRviz_.publish(vehiclePathInRviz_);
}

void Dijkstra::showRoadBorderInRviz() {
  roadLeftBorderPub_.publish(roadLeftBorder_);
  roadRightBorderPub_.publish(roadRightBorder_);
}

}  // namespace lattice_planner

#ifdef BUILD_INDIVIDUAL
/**
* @brief Script main function. Generates start and end nodes as well as grid,
* then creates the algorithm object and calls the main algorithm function.
* @return 0
*/
int main() {
  Node initState(0.0, 0.0, 0.0, 0, -1);

  // Make sure start and goal are not obstacles and their ids are correctly
  // assigned.

  Dijkstra new_dijkstra;
  std::vector<Node> closed = new_dijkstra.dijkstra(start);
  Node goal = planning.determineGoal();
  std::vector<Node> pathNode = planning.pathTrace(goal);
  for (auto i : pathNode) {
    std::cout << "node:" << i.x_ << "-" << i.y_ << std::endl;
  }

  return 0;
}
#endif
