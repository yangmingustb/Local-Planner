// this node is to parameter the reference line using arc-length parameter s.

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include "referenceLine.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "reference_arclength_node");

  ros::NodeHandle nh;

  ros::Publisher referenceLine_pub =
      nh.advertise<nav_msgs::Path>("referenceLine", 1, true);
  ros::Publisher referenceLine_pub2 =
      nh.advertise<nav_msgs::Path>("referenceLine2", 1, true);

  reference_line::RefLine reference_line;

  nav_msgs::Path ref_path = reference_line.generateRefLine_inRviz();

  nav_msgs::Path ref_path2 = reference_line.readCoefficientsFromFile();

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    referenceLine_pub.publish(ref_path);
    referenceLine_pub2.publish(ref_path2);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}