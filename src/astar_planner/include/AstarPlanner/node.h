#ifndef _NODE_H_
#define _NODE_H_

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

#define inf 1 >> 20
struct Node2D;
typedef Node2D* Node2DPtr;

struct Node2D {
  int id;  // 1--> open set, -1 --> closed set
  Eigen::Vector2d coord;
  Eigen::Vector2i dir;  // direction of expanding
  Eigen::Vector2i index;

  double gScore, fScore;
  Node2DPtr predecessor;
  std::multimap<double, Node2DPtr>::iterator nodeMapIt;

  Node2D(Eigen::Vector2i _index, Eigen::Vector2d _coord) {
    id = 0;
    index = _index;
    coord = _coord;
    dir = Eigen::Vector2i::Zero();

    gScore = inf;
    fScore = inf;
    predecessor = NULL;
  }

  Node2D(){};
  ~Node2D(){};
};

#endif
