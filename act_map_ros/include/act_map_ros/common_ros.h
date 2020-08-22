#pragma once

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <rpg_common/pose.h>

#include <act_map/common.h>

namespace act_map_ros
{
extern const std::string kWorldFrame;

using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;

inline std_msgs::ColorRGBA getColor(const std::string& color_name)
{
  std_msgs::ColorRGBA c;
  c.a = 1.0;
  if (color_name == "RED")
  {
    c.r = 1.0; c.g = 0.0; c.b = 0.0;
  }
  else if (color_name == "GREEN")
  {
    c.r = 0.0; c.g = 1.0; c.b = 0.0;
  }
  else if (color_name == "BLUE")
  {
    c.r = 0.0; c.g = 0.0; c.b = 1.0;
  }
  else
  {
    LOG(FATAL) << "Unknown color name.";
  }
  return c;
}

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

void directionsToArrowsArray(const act_map::Vec3dVec& s_points,
                             const act_map::Vec3dVec& directions,
                             const std::vector<size_t>& ids,
                             const std::vector<std_msgs::ColorRGBA>& colors,
                             const std::string& ns,
                             const double size,
                             visualization_msgs::MarkerArray* ma);

void publishCameraMarker(const ros::Publisher& pub,
                         const std::string& frame_id,
                         const std::string& ns,
                         const ros::Time& timestamp,
                         const int id,
                         const int action,
                         const double marker_scale,
                         const Eigen::Vector3d& color,
                         const rpg::Pose& T_offset);

void scaleLog10Ntimes(const std::vector<double>& values,
                      const size_t log_times,
                      std::vector<double>* color_intensities);

void normalizeToRGB(const std::vector<double>& values,
                    std::vector<std_msgs::ColorRGBA>* color_intensities);

void rotTransToAxisMarkerArray(const act_map::RotMatVec& Rwb_vec,
                               const act_map::Vec3dVec& pos_vec,
                               const std::string& ns,
                               const double size,
                               visualization_msgs::MarkerArray* ma);

void rotTransToAxisMarkerArray(const rpg::PoseVec& Twb_vec,
                               const std::string& ns,
                               const double size,
                               visualization_msgs::MarkerArray* ma);
}
