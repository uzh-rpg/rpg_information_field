//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <act_map_ros/common_ros.h>

#include <rpg_common/pose.h>
#include <geometry_msgs/Pose.h>

namespace act_map_ros
{
template <typename T>
void eigen3XdToPCLPointCloud(
    const Eigen::Matrix<T, 3, Eigen::Dynamic>& eigen_mat, PCLPointCloud* pcl_pc)
{
  // as unordered pointcloud
  pcl_pc->height = 1;
  pcl_pc->width = eigen_mat.cols();
  for (int i = 0; i < eigen_mat.cols(); i++)
  {
    pcl_pc->push_back(
        pcl::PointXYZ(eigen_mat(0, i), eigen_mat(1, i), eigen_mat(2, i)));
  }
}

void Vec3dVecToPCLPointCloud(
    const act_map::Vec3dVec& vec, PCLPointCloud* pcl_pc);

template <typename T>
void pCLPointCloudToEigen3Xd(
    const PCLPointCloud& pcl_pc, Eigen::Matrix<T, 3, Eigen::Dynamic>* eigen_mat)
{
  int N = static_cast<int>(pcl_pc.size());
  eigen_mat->resize(Eigen::NoChange, N);
  for (int i = 0; i < N; i++)
  {
    (*eigen_mat)(0, i) = pcl_pc.points[i].x;
    (*eigen_mat)(1, i) = pcl_pc.points[i].y;
    (*eigen_mat)(2, i) = pcl_pc.points[i].z;
  }
}

inline void kindrToROSPose(const rpg::Pose& kindr_pose,
                           geometry_msgs::Pose* ros_pose)
{
  ros_pose->position.x = kindr_pose.getPosition().x();
  ros_pose->position.y = kindr_pose.getPosition().y();
  ros_pose->position.z = kindr_pose.getPosition().z();
  ros_pose->orientation.w = kindr_pose.getRotation().w();
  ros_pose->orientation.x = kindr_pose.getRotation().x();
  ros_pose->orientation.y = kindr_pose.getRotation().y();
  ros_pose->orientation.z = kindr_pose.getRotation().z();
}

inline void rosPoseToKindr(const geometry_msgs::Pose& ros_pose,
                           rpg::Pose* pose)
{
  const geometry_msgs::Point& pos = ros_pose.position;
  const geometry_msgs::Quaternion& quat = ros_pose.orientation;
  pose->getPosition() = Eigen::Vector3d(pos.x, pos.y, pos.z);
  pose->getRotation() = rpg::Rotation(quat.w, quat.x, quat.y, quat.z);
}
}
