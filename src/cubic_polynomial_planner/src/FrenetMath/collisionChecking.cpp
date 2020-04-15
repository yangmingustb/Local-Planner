

#include "collisionChecking.h"

namespace lattice_planner {

// VehicleDisks::VehicleDisks(CartesianPose &rearAxleMidPoint,
//                             double vehicleWidth,
//                             double vehicleLength,
//                             double H){

//     RearAxleMidPoint_ = rearAxleMidPoint;

//     VehicleWidth_=vehicleWidth;
//     VehicleLength_=vehicleLength;
//     H_ = H;
//     Distance_ = VehicleLength_/2.0 - H_;

//     disk1.CrtPoint_.x = RearAxleMidPoint_.x;
//     disk1.CrtPoint_.y = RearAxleMidPoint_.y;
//     disk1.radius= sqrt(H_*H_ + VehicleWidth_*VehicleWidth_/4.0);

//     disk2.CrtPoint_.x = RearAxleMidPoint_.x + Distance_ *
//     cos(RearAxleMidPoint_.yaw);
//     disk2.CrtPoint_.y = RearAxleMidPoint_.y + Distance_ *
//     sin(RearAxleMidPoint_.yaw);
//     disk2.radius = disk1.radius;

//     disk3.CrtPoint_.x = RearAxleMidPoint_.x + 2 * Distance_ *
//     cos(RearAxleMidPoint_.yaw);
//     disk3.CrtPoint_.y = RearAxleMidPoint_.y + 2 * Distance_ *
//     sin(RearAxleMidPoint_.yaw);
//     disk3.radius = disk1.radius;

//     BigDisk.CrtPoint_.x = disk2.CrtPoint_.x;
//     BigDisk.CrtPoint_.y = disk2.CrtPoint_.y;
//     BigDisk.radius = sqrt(VehicleLength_ * VehicleLength_ / 4.0 +
//     VehicleWidth_*VehicleWidth_/4.0);

// }

disksTrajectory::disksTrajectory(
    // std::vector<CubicCoefficients> &coefficients,
    // nav_msgs::Path &refline,
    cartesianToFrenet &crtToFrt, CartesianPath &crtPath) {
  VehicleWidth_ = 1.6;
  VehicleLength_ = 4.0;
  H_ = 1.0;
  Distance_ = VehicleLength_ / 2.0 - H_;

  // coefficients_ = coefficients;
  // refline_ = refline;
  crtToFrt_ = crtToFrt;
  crtPath_ = crtPath;
}

void disksTrajectory::setParameters(
    // std::vector<CubicCoefficients> &coefficients,
    // nav_msgs::Path &refline,
    cartesianToFrenet &crtToFrt, CartesianPath &crtPath) {
  VehicleWidth_ = 1.6;
  VehicleLength_ = 4.0;
  H_ = 1.0;
  Distance_ = VehicleLength_ / 2.0 - H_;

  // coefficients_ = coefficients;
  // refline_ = refline;
  crtToFrt_ = crtToFrt;
  crtPath_ = crtPath;
}

VehicleDisks disksTrajectory::generateDisks(CartesianPose &RearAxleMidPoint_) {
  // showClassParameters();
  VehicleDisks VehicleDisks_;
  VehicleDisks_.disk1.CrtPoint_.x = RearAxleMidPoint_.x;
  VehicleDisks_.disk1.CrtPoint_.y = RearAxleMidPoint_.y;
  VehicleDisks_.disk1.radius_ =
      sqrt(H_ * H_ + VehicleWidth_ * VehicleWidth_ / 4.0);

  // geometry_msgs::Quaternion q =
  // tf::createQuaternionMsgFromYaw(RearAxleMidPoint_.yaw);
  // geometry_msgs::PoseWithCovarianceStamped diskStamped1;
  // diskStamped1.pose.pose.position.x = disk1.CrtPoint_.x;
  // diskStamped1.pose.pose.position.y = disk1.CrtPoint_.y;
  // diskStamped1.pose.pose.orientation = q;
  // diskStamped1.header.stamp=ros::Time::now();
  // diskStamped1.header.frame_id="/map";
  // FrenetPose frtdisk1 = crtToFrt_.transform(diskStamped1);
  // disk1.FrtPoint_.s = frtdisk1.s;
  // disk1.FrtPoint_.rho = frtdisk1.rho;

  VehicleDisks_.disk2.CrtPoint_.x =
      RearAxleMidPoint_.x + Distance_ * cos(RearAxleMidPoint_.yaw);
  VehicleDisks_.disk2.CrtPoint_.y =
      RearAxleMidPoint_.y + Distance_ * sin(RearAxleMidPoint_.yaw);
  VehicleDisks_.disk2.radius_ = VehicleDisks_.disk1.radius_;

  // geometry_msgs::PoseWithCovarianceStamped diskStamped2;
  // diskStamped2.pose.pose.position.x = disk2.CrtPoint_.x;
  // diskStamped2.pose.pose.position.y = disk2.CrtPoint_.y;
  // diskStamped2.pose.pose.orientation = q;
  // diskStamped2.header.stamp=ros::Time::now();
  // diskStamped2.header.frame_id="/map";
  // FrenetPose frtdisk2 = crtToFrt_.transform(diskStamped2);
  // disk2.FrtPoint_.s = frtdisk2.s;
  // disk2.FrtPoint_.rho = frtdisk2.rho;

  VehicleDisks_.disk3.CrtPoint_.x =
      RearAxleMidPoint_.x + 2 * Distance_ * cos(RearAxleMidPoint_.yaw);
  VehicleDisks_.disk3.CrtPoint_.y =
      RearAxleMidPoint_.y + 2 * Distance_ * sin(RearAxleMidPoint_.yaw);
  VehicleDisks_.disk3.radius_ = VehicleDisks_.disk1.radius_;

  // geometry_msgs::PoseWithCovarianceStamped diskStamped3;
  // diskStamped3.pose.pose.position.x = disk3.CrtPoint_.x;
  // diskStamped3.pose.pose.position.y = disk3.CrtPoint_.y;
  // diskStamped3.pose.pose.orientation = q;
  // diskStamped3.header.stamp=ros::Time::now();
  // diskStamped3.header.frame_id="/map";
  // FrenetPose frtdisk3 = crtToFrt_.transform(diskStamped3);
  // disk3.FrtPoint_.s = frtdisk3.s;
  // disk3.FrtPoint_.rho = frtdisk3.rho;

  VehicleDisks_.BigDisk.CrtPoint_.x = VehicleDisks_.disk2.CrtPoint_.x;
  VehicleDisks_.BigDisk.CrtPoint_.y = VehicleDisks_.disk2.CrtPoint_.y;
  VehicleDisks_.BigDisk.radius_ = sqrt(VehicleLength_ * VehicleLength_ / 4.0 +
                                       VehicleWidth_ * VehicleWidth_ / 4.0);
  // BigDisk.FrtPoint_.s = disk2.FrtPoint_.s;
  // BigDisk.FrtPoint_.rho = disk2.FrtPoint_.rho;

  return VehicleDisks_;
}

std::vector<VehicleDisks> disksTrajectory::generateDisksTrajectory() {
  for (auto i : crtPath_.ctsPath_) {
    VehicleDisks tmpVehicleDisks = generateDisks(i);
    disksTrajectory_.push_back(tmpVehicleDisks);
  }

  return disksTrajectory_;
}

void disksTrajectory::showClassParameters() {
  ROS_INFO("H:%f", H_);
  ROS_INFO("vehicle width:%f", VehicleWidth_);
  ROS_INFO("vehicle length:%f", VehicleLength_);
  ROS_INFO("distance_:%f", Distance_);
}

CollisionChecker::CollisionChecker(
    CartesianPath &ctsPath, PointsObstacle &obstacles, double rhoLeft,
    double rhoRight,
    // nav_msgs::Path &refline,
    // std::vector<CubicCoefficients> &coefficients,
    cartesianToFrenet &crtToFrt) {
  ROS_INFO("collision checking is beginning...");
  ctsPath_ = ctsPath;
  obstacles_ = obstacles;
  rhoLeft_ = rhoLeft;
  rhoRight_ = rhoRight;

  // refline_ = refline;
  // coefficients_ = coefficients;
  crtToFrt_ = crtToFrt;

  disksTrajectoryObj_.setParameters(crtToFrt_, ctsPath_);
  disksTrajectoryObj_.showClassParameters();
}

bool CollisionChecker::performChecking() {
  auto time1 = ros::Time::now().toSec();
  std::vector<VehicleDisks> disksTrajectory_;

  auto timeGenerateDisksTrajec = ros::Time::now().toSec();
  disksTrajectory_ = disksTrajectoryObj_.generateDisksTrajectory();
  auto time2 = ros::Time::now().toSec() - timeGenerateDisksTrajec;
  ROS_INFO("generate disks trajectory running time is:%f", time2);
  ROS_INFO("the diks length is:%d", disksTrajectory_.size());

  /**
   * for debug
   */
  for (auto o : obstacles_.points) {
    ROS_INFO("obstacles position:%f,%f", o.x, o.y);
  }

  /**
   * for debug
   */
  for (auto d : disksTrajectory_) {
    // ROS_INFO("the big disk's radius is:%f",d.BigDisk.radius_);
    // ROS_INFO("the small disk's radius is:%f",d.disk1.radius_);s
  }
  for (auto m : disksTrajectory_) {
    VehicleDisk BigDisk = m.BigDisk;

    for (auto q : obstacles_.points) {
      double tmpDistance =
          pow(BigDisk.CrtPoint_.x - q.x, 2) + pow(BigDisk.CrtPoint_.y - q.y, 2);
      if (tmpDistance > pow(BigDisk.radius_, 2)) {
        // ROS_INFO("the distance is bigger than the BigDisk's radius.");
        continue;
      }

      else {
        // ROS_INFO("the distance is smaller than the BigDisk's radius.");
        VehicleDisk disk1 = m.disk1;
        VehicleDisk disk2 = m.disk2;
        VehicleDisk disk3 = m.disk3;
        tmpDistance =
            pow(disk1.CrtPoint_.x - q.x, 2) + pow(disk1.CrtPoint_.y - q.y, 2);
        if (tmpDistance < pow(disk1.radius_, 2)) {
          // ROS_INFO("the min distance is: %f", sqrt(tmpDistance));
          return false;
        }
        tmpDistance =
            pow(disk2.CrtPoint_.x - q.x, 2) + pow(disk2.CrtPoint_.y - q.y, 2);
        if (tmpDistance < pow(disk2.radius_, 2)) {
          // ROS_INFO("the min distance is: %f", sqrt(tmpDistance));
          return false;
        }
        tmpDistance =
            pow(disk3.CrtPoint_.x - q.x, 2) + pow(disk3.CrtPoint_.y - q.y, 2);
        if (tmpDistance < pow(disk1.radius_, 2)) {
          // ROS_INFO("the min distance is: %f", sqrt(tmpDistance));
          return false;
        }
      }
    }
  }
  ROS_INFO("collision checking is endding...");
  auto checkingTime = ros::Time::now().toSec() - time1;
  ROS_INFO("checking time is:%f", checkingTime);
  return true;
}
}