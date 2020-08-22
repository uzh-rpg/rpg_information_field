#include "act_map_exp/viz_utils.h"

#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <act_map/common.h>
#include <act_map_ros/common_ros.h>
#include <vi_utils/common_utils.h>

#include "act_map_exp/exp_utils.h"

namespace act_map_exp
{
using Point = pcl::PointXYZRGB;
using PointCloud = pcl::PointCloud<Point>;

void visualizeTrajectory(const rpg::PoseVec& Twbs,
                         const ros::Publisher& traj_pos_pub,
                         const ros::Publisher& traj_orient_pub,
                         const rpg::Pose& Tbc, const Eigen::Vector3d& rgb,
                         const std::string& frame)
{
  ros::Time now = ros::Time::now();
  if (traj_pos_pub.getNumSubscribers() > 0)
  {
    PointCloud traj_pos_pc;
    pcl_conversions::toPCL(now, traj_pos_pc.header.stamp);
    traj_pos_pc.header.frame_id = frame;
    traj_pos_pc.reserve(Twbs.size());
    for (const rpg::Pose& Twb : Twbs)
    {
      Point pt;
      pt.x = static_cast<float>(Twb.getPosition().x());
      pt.y = static_cast<float>(Twb.getPosition().y());
      pt.z = static_cast<float>(Twb.getPosition().z());
      pt.r = static_cast<uint8_t>(rgb(0) * 255);
      pt.g = static_cast<uint8_t>(rgb(1) * 255);
      pt.b = static_cast<uint8_t>(rgb(2) * 255);
      traj_pos_pc.push_back(pt);
    }
    traj_pos_pub.publish(traj_pos_pc);
  }

  if (traj_orient_pub.getNumSubscribers() > 0)
  {
    constexpr size_t kMaxOrientViz = 10u;
    std::vector<size_t> viz_idx;
    if (Twbs.size() < kMaxOrientViz + 1)
    {
      for (size_t i = 0; i < Twbs.size(); i++)
      {
        viz_idx.emplace_back(i);
      }
    }
    else
    {
      sampleIndices(kMaxOrientViz, Twbs.size(), &viz_idx);
    }

    rpg::PoseVec Twcs;
    for (const size_t idx : viz_idx)
    {
      Twcs.emplace_back(Twbs.at(idx) * Tbc);
    }

    visualization_msgs::MarkerArray ma_body;
    constexpr double vis_downscale = 4;
    const double vis_scale_base =
        (Twbs.front().getPosition() - Twbs.back().getPosition()).norm() /
        (vis_downscale * kMaxOrientViz);
    act_map_ros::rotTransToAxisMarkerArray(Twbs, "body", vis_scale_base,
                                           &ma_body);
    traj_orient_pub.publish(ma_body);
    visualization_msgs::MarkerArray ma_cam;
    act_map_ros::rotTransToAxisMarkerArray(Twcs, "cam", vis_scale_base * 0.5,
                                           &ma_cam);
    traj_orient_pub.publish(ma_cam);

    ros::Time now = ros::Time::now();
    for (size_t idx = 0; idx < Twcs.size(); idx++)
    {
      act_map_ros::publishCameraMarker(
          traj_orient_pub, frame, "cam_view", now, static_cast<int>(idx),
          visualization_msgs::Marker::ADD, vis_scale_base*2.50, rgb, Twcs[idx]);
    }
  }
}

void sampleColor(const std_msgs::ColorRGBA& color_s,
                 const std_msgs::ColorRGBA& color_e, const size_t N,
                 std::vector<std_msgs::ColorRGBA>* samples)
{
  CHECK(samples);
  std::vector<double> r;
  vi_utils::linspace(color_s.r, color_e.r, N, &r);
  std::vector<double> g;
  vi_utils::linspace(color_s.g, color_e.g, N, &g);
  std::vector<double> b;
  vi_utils::linspace(color_s.b, color_e.b, N, &b);
  std::vector<double> a;
  vi_utils::linspace(color_s.a, color_e.a, N, &a);

  samples->clear();
  for (size_t i = 0; i < N; i++)
  {
    std_msgs::ColorRGBA c;
    c.r = r[i];
    c.g = g[i];
    c.b = b[i];
    c.a = a[i];
    samples->emplace_back(c);
  }
}

void clearMarkerArray(const ros::Publisher& ma_pub)
{
  visualization_msgs::MarkerArray ma_clear;
  visualization_msgs::Marker m_clear;
  m_clear.action = visualization_msgs::Marker::DELETEALL;
  ma_clear.markers.push_back(m_clear);
  ma_pub.publish(ma_clear);
}

void clearMarker(const ros::Publisher& m_pub)
{
  visualization_msgs::Marker m_clear;
  m_clear.action = visualization_msgs::Marker::DELETEALL;
  m_pub.publish(m_clear);
}

void visualizePath(const rpg::PoseVec& poses,
                   const std_msgs::ColorRGBA& start_c,
                   const std_msgs::ColorRGBA& end_c,
                   const ros::Publisher& marker_pub,
                   const int id,
                   const std::string& frame,
                   const std::string& ns)
{
  if (poses.size() == 0)
  {
    LOG(WARNING) << "no path exists.";
    return;
  }

  if (marker_pub.getNumSubscribers() > 0)
  {
    visualization_msgs::Marker line_list;
    line_list.ns = ns;
    line_list.id = id;
    line_list.header.frame_id = frame;
    line_list.header.stamp = ros::Time::now();
    line_list.pose.orientation.w = 1.0;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.scale.x = getPathLength(poses) * 0.01;
    std::vector<std_msgs::ColorRGBA> colors;
    sampleColor(start_c, end_c, poses.size(), &colors);
    for (size_t i = 0; i < poses.size(); i++)
    {
      geometry_msgs::Point pt;
      pt.x = poses[i].getPosition().x();
      pt.y = poses[i].getPosition().y();
      pt.z = poses[i].getPosition().z();
      line_list.points.push_back(pt);
      line_list.colors.push_back(colors[i]);
    }

    marker_pub.publish(line_list);
  }
}

}  // namespace act_map_exp
