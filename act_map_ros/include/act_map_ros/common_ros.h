//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/MarkerArray.h>

#include <act_map/common.h>

namespace act_map_ros
{
extern const std::string kWorldFrame;

using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;

inline void setPCLPointCloud(PCLPointCloud* pc,
                             const std::string& frame_id,
                             const ros::Time& time)
{
  pc->header.frame_id = frame_id;
  pcl_conversions::toPCL(time, pc->header.stamp);
}

inline void createCamFrameId(const size_t i, std::string* cam_frame_id)
{
  (*cam_frame_id) = "cam" + std::to_string(i);
}

void drawKeypoints(const Eigen::Matrix2Xd& us,
                   const cv::Scalar color,
                   const int radius,
                   cv::Mat* img);

void directionsToMarkerArray(const act_map::Vec3dVec& s_points,
                             const act_map::Vec3dVec& e_points,
                             const double size,
                             visualization_msgs::MarkerArray* ma);

void directionsToArrowsArray(const act_map::Vec3dVec& s_points,
                             const act_map::Vec3dVec& directions,
                             const std::vector<size_t>& ids,
                             const std::vector<std_msgs::ColorRGBA>& colors,
                             const double size,
                             visualization_msgs::MarkerArray* ma);

void publishCameraMarker(ros::Publisher pub,
                         const std::string& frame_id,
                         const std::string& ns,
                         const ros::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Eigen::Vector3d& color);

void scaleLog10Ntimes(const std::vector<double>& values,
                      const size_t log_times,
                      std::vector<double>* color_intensities);

void normalizeToRGB(const std::vector<double>& values,
                    std::vector<std_msgs::ColorRGBA>* color_intensities);
}
