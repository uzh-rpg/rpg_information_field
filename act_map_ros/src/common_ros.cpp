//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map_ros/common_ros.h"

#include <glog/logging.h>

namespace act_map_ros
{
const std::string kWorldFrame("world");

void drawKeypoints(const Eigen::Matrix2Xd& us,
                   const cv::Scalar color,
                   const int radius,
                   cv::Mat* img)
{
  CHECK_NOTNULL(img);
  CHECK(!img->empty());

  for (int i = 0; i < us.cols(); i++)
  {
    cv::circle(*img, cv::Point2d(us(0, i), us(1, i)), radius, color);
  }
}

void directionsToMarkerArray(const act_map::Vec3dVec& s_points,
                             const act_map::Vec3dVec& e_points,
                             const double size,
                             visualization_msgs::MarkerArray* ma)
{
  CHECK_NOTNULL(ma);
  CHECK_EQ(s_points.size(), e_points.size());

  visualization_msgs::Marker m_dir;
  m_dir.header.frame_id = kWorldFrame;
  m_dir.ns = "dir";
  m_dir.id = 0;
  m_dir.type = visualization_msgs::Marker::LINE_LIST;
  std_msgs::ColorRGBA c;
  c.a = 1.0;
  c.g = 0.5;
  m_dir.color = c;
  m_dir.action = visualization_msgs::Marker::ADD;

  // end points
  visualization_msgs::Marker m_end;
  m_end.header.frame_id = kWorldFrame;
  m_end.ns = "end";
  m_end.id = 1;
  m_end.type = visualization_msgs::Marker::CUBE_LIST;
  std_msgs::ColorRGBA c_end;
  c_end.a = 1.0;
  c_end.r = 0.8;
  m_end.color = c_end;
  m_end.action = visualization_msgs::Marker::ADD;
  for (size_t i = 0; i < s_points.size(); i++)
  {
    const double line_width = 0.2 * size;
    const double line_len = 1.5 * size;

    geometry_msgs::Point start;
    start.x = s_points[i].x();
    start.y = s_points[i].y();
    start.z = s_points[i].z();
    m_dir.points.emplace_back(start);
    geometry_msgs::Point end;
    end.x = start.x + e_points[i].x() * line_len;
    end.y = start.y + e_points[i].y() * line_len;
    end.z = start.z + e_points[i].z() * line_len;
    m_dir.points.emplace_back(end);
    m_dir.scale.x = line_width;  // line width

    m_end.points.push_back(end);
    m_end.scale.x = m_end.scale.y = m_end.scale.z = line_width * 1.3;
  }
  ma->markers.push_back(m_dir);
  ma->markers.push_back(m_end);
}

void directionsToArrowsArray(const act_map::Vec3dVec& s_points,
                             const act_map::Vec3dVec& directions,
                             const std::vector<size_t>& ids,
                             const std::vector<std_msgs::ColorRGBA>& colors,
                             const double size,
                             visualization_msgs::MarkerArray* ma)
{
  CHECK_EQ(s_points.size(), directions.size());
  CHECK_EQ(s_points.size(), ids.size());
  CHECK_EQ(s_points.size(), colors.size());
  CHECK_NOTNULL(ma);

  for (size_t i = 0; i < s_points.size(); i++)
  {
    visualization_msgs::Marker m;
    m.header.frame_id = kWorldFrame;
    m.ns = "dir";
    m.id = ids[i];
    m.type = visualization_msgs::Marker::ARROW;
    m.color = colors[i];
    m.action = visualization_msgs::Marker::ADD;

    const double shaft_dia = 0.2 * size;
    const double arrow_len = 10 * shaft_dia;

    geometry_msgs::Point start;
    start.x = s_points[i].x();
    start.y = s_points[i].y();
    start.z = s_points[i].z();
    m.points.emplace_back(start);
    geometry_msgs::Point end;
    end.x = start.x + directions[i].x() * arrow_len;
    end.y = start.y + directions[i].y() * arrow_len;
    end.z = start.z + directions[i].z() * arrow_len;
    m.points.emplace_back(end);
    m.scale.x = shaft_dia;        // shaft dia
    m.scale.y = 2.5 * shaft_dia;  // head dia
    m.scale.z = 0.6 * arrow_len;  // head len
    ma->markers.emplace_back(m);
  }
}

void publishCameraMarker(ros::Publisher pub,
                         const std::string& frame_id,
                         const std::string& ns,
                         const ros::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Eigen::Vector3d& color)
{
  /*
   * draw a pyramid as the camera marker
   */
  const double sqrt2_2 = sqrt(2) / 2;

  visualization_msgs::Marker marker;

  // the marker will be displayed in frame_id
  marker.header.frame_id = frame_id;
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.action = action;
  marker.id = id;

  // make rectangles as frame
  double r_w = 1.0;
  double z_plane = (r_w / 2.0) * marker_scale;
  marker.pose.position.x = 0;
  marker.pose.position.y = (r_w / 4.0) * marker_scale;
  marker.pose.position.z = z_plane;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = r_w * marker_scale;
  marker.scale.y = 0.04 * marker_scale;
  marker.scale.z = 0.04 * marker_scale;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1;

  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.id--;
  pub.publish(marker);
  marker.pose.position.y = -(r_w / 4.0) * marker_scale;
  marker.id--;
  pub.publish(marker);

  marker.scale.x = (r_w / 2.0) * marker_scale;
  marker.pose.position.x = (r_w / 2.0) * marker_scale;
  marker.pose.position.y = 0;
  marker.pose.orientation.w = sqrt2_2;
  marker.pose.orientation.z = sqrt2_2;
  marker.id--;
  pub.publish(marker);
  marker.pose.position.x = -(r_w / 2.0) * marker_scale;
  marker.id--;
  pub.publish(marker);

  // make pyramid edges
  marker.scale.x = (3.0 * r_w / 4.0) * marker_scale;
  marker.pose.position.z = 0.5 * z_plane;

  marker.pose.position.x = (r_w / 4.0) * marker_scale;
  marker.pose.position.y = (r_w / 8.0) * marker_scale;
  //  0.08198092, -0.34727674,  0.21462883,  0.9091823
  marker.pose.orientation.x = 0.08198092;
  marker.pose.orientation.y = -0.34727674;
  marker.pose.orientation.z = 0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  pub.publish(marker);

  marker.pose.position.x = -(r_w / 4.0) * marker_scale;
  marker.pose.position.y = (r_w / 8.0) * marker_scale;
  // -0.27395078, -0.22863284,  0.9091823 ,  0.21462883
  marker.pose.orientation.x = 0.08198092;
  marker.pose.orientation.y = 0.34727674;
  marker.pose.orientation.z = -0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  pub.publish(marker);

  marker.pose.position.x = -(r_w / 4.0) * marker_scale;
  marker.pose.position.y = -(r_w / 8.0) * marker_scale;
  //  -0.08198092,  0.34727674,  0.21462883,  0.9091823
  marker.pose.orientation.x = -0.08198092;
  marker.pose.orientation.y = 0.34727674;
  marker.pose.orientation.z = 0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  pub.publish(marker);

  marker.pose.position.x = (r_w / 4.0) * marker_scale;
  marker.pose.position.y = -(r_w / 8.0) * marker_scale;
  // -0.08198092, -0.34727674, -0.21462883,  0.9091823
  marker.pose.orientation.x = -0.08198092;
  marker.pose.orientation.y = -0.34727674;
  marker.pose.orientation.z = -0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id--;
  pub.publish(marker);
}

void scaleLog10Ntimes(const std::vector<double>& values,
                      const size_t log_times,
                      std::vector<double>* color_intensities)
{
  CHECK_NOTNULL(color_intensities);
  (*color_intensities) = values;
  for (size_t i = 0; i < log_times; i++)
  {
    for (size_t idx = 0; idx < values.size(); idx++)
    {
      //      CHECK((*color_intensities)[idx] > 0);
      (*color_intensities)[idx] = std::log10((*color_intensities)[idx]);
    }
  }
}

void normalizeToRGB(const std::vector<double>& values,
                    std::vector<std_msgs::ColorRGBA>* color_intensities)
{
  CHECK_NOTNULL(color_intensities);
  color_intensities->resize(values.size());
  double vmax = *std::max_element(values.begin(), values.end());
  double vmin = *std::min_element(values.begin(), values.end());

  // adapt from :
  // https://stackoverflow.com/questions/7706339/grayscale-to-red-green-blue-matlab-jet-color-scale

  const double dv = vmax - vmin;
  for (size_t idx = 0; idx < values.size(); idx++)
  {
    double r = 1.0;
    double g = 1.0;
    double b = 1.0;
    double v = values[idx];
    if (v < vmin)
    {
      v = vmin;
    }
    if (v > vmax)
    {
      v = vmax;
    }

    if (v < (vmin + 0.25 * dv))
    {
      r = 0;
      g = 4 * (v - vmin) / dv;
    }
    else if (v < (vmin + 0.5 * dv))
    {
      r = 0;
      b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
    }
    else if (v < (vmin + 0.75 * dv))
    {
      r = 4 * (v - vmin - 0.5 * dv) / dv;
      b = 0;
    }
    else
    {
      g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
      b = 0;
    }
    (*color_intensities)[idx].a = 1.0;
    (*color_intensities)[idx].r = r;
    (*color_intensities)[idx].g = g;
    (*color_intensities)[idx].b = b;
  }
}
}
