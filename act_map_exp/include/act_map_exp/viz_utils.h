#pragma once

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <rpg_common/pose.h>

namespace act_map_exp
{
void visualizeTrajectory(const rpg::PoseVec& Twb_vec,
                         const ros::Publisher& traj_pos_pub,
                         const ros::Publisher& traj_orient_pub,
                         const rpg::Pose& Tbc, const Eigen::Vector3d& rgb,
                         const std::string& frame);

void sampleColor(const std_msgs::ColorRGBA& color_s,
                 const std_msgs::ColorRGBA& color_e,
                 const size_t N,
                 std::vector<std_msgs::ColorRGBA>* samples);

void clearMarkerArray(const ros::Publisher& ma_pub);
void clearMarker(const ros::Publisher& m_pub);

void visualizePath(const rpg::PoseVec& poses,
                   const std_msgs::ColorRGBA& start_c,
                   const std_msgs::ColorRGBA& end_c,
                   const ros::Publisher& marker_pub,
                   const int id,
                   const std::string& frame,
                   const std::string& ns);
}
