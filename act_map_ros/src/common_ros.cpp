#include "act_map_ros/common_ros.h"

#include <glog/logging.h>
#include <numeric>

namespace act_map_ros
{
const std::string kWorldFrame("world");

void drawKeypoints(const Eigen::Matrix2Xd& us, const cv::Scalar color,
                   const int radius, cv::Mat* img)
{
  CHECK_NOTNULL(img);
  CHECK(!img->empty());

  for (int i = 0; i < us.cols(); i++)
  {
    cv::circle(*img, cv::Point2d(us(0, i), us(1, i)), radius, color);
  }
}

void directionsToArrowsArray(const act_map::Vec3dVec& s_points,
                             const act_map::Vec3dVec& directions,
                             const std::vector<size_t>& ids,
                             const std::vector<std_msgs::ColorRGBA>& colors,
                             const std::string& ns, const double size,
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
    m.ns = ns;
    m.id = ids[i];
    m.type = visualization_msgs::Marker::ARROW;
    m.color = colors[i];
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;

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

void publishCameraMarker(const ros::Publisher& pub, const std::string& frame_id,
                         const std::string& ns, const ros::Time& timestamp,
                         const int id, const int action,
                         const double marker_scale,
                         const Eigen::Vector3d& color,
                         const rpg::Pose& T_offset)
{
  /*
   * draw a pyramid as the camera marker
   */
  const double sqrt2_2 = sqrt(2) / 2;

  constexpr int n_marker = 8;
  const int marker_id_base = id * n_marker;

  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker marker;
  // the marker will be displayed in frame_id
  marker.header.frame_id = frame_id;
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.action = action;

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
  marker.id = marker_id_base;
  ma.markers.push_back(marker);

  marker.pose.position.y = -(r_w / 4.0) * marker_scale;
  marker.id = marker_id_base + 1;
  ma.markers.push_back(marker);

  marker.scale.x = (r_w / 2.0) * marker_scale;
  marker.pose.position.x = (r_w / 2.0) * marker_scale;
  marker.pose.position.y = 0;
  marker.pose.orientation.w = sqrt2_2;
  marker.pose.orientation.z = sqrt2_2;
  marker.id = marker_id_base + 2;
  ma.markers.push_back(marker);
  marker.pose.position.x = -(r_w / 2.0) * marker_scale;
  marker.id = marker_id_base + 3;
  ma.markers.push_back(marker);

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
  marker.id = marker_id_base + 4;
  ma.markers.push_back(marker);

  marker.pose.position.x = -(r_w / 4.0) * marker_scale;
  marker.pose.position.y = (r_w / 8.0) * marker_scale;
  // -0.27395078, -0.22863284,  0.9091823 ,  0.21462883
  marker.pose.orientation.x = 0.08198092;
  marker.pose.orientation.y = 0.34727674;
  marker.pose.orientation.z = -0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id = marker_id_base + 5;
  ma.markers.push_back(marker);

  marker.pose.position.x = -(r_w / 4.0) * marker_scale;
  marker.pose.position.y = -(r_w / 8.0) * marker_scale;
  //  -0.08198092,  0.34727674,  0.21462883,  0.9091823
  marker.pose.orientation.x = -0.08198092;
  marker.pose.orientation.y = 0.34727674;
  marker.pose.orientation.z = 0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id = marker_id_base + 6;
  ma.markers.push_back(marker);

  marker.pose.position.x = (r_w / 4.0) * marker_scale;
  marker.pose.position.y = -(r_w / 8.0) * marker_scale;
  // -0.08198092, -0.34727674, -0.21462883,  0.9091823
  marker.pose.orientation.x = -0.08198092;
  marker.pose.orientation.y = -0.34727674;
  marker.pose.orientation.z = -0.21462883;
  marker.pose.orientation.w = 0.9091823;
  marker.id = marker_id_base + 7;
  ma.markers.push_back(marker);

  for (visualization_msgs::Marker& m : ma.markers)
  {
    rpg::Pose T_om;
    T_om.getRotation() =
        rpg::Rotation(m.pose.orientation.w, m.pose.orientation.x,
                      m.pose.orientation.y, m.pose.orientation.z);
    T_om.getPosition() =
        rpg::Position(m.pose.position.x, m.pose.position.y, m.pose.position.z);
    rpg::Pose Twm = T_offset * T_om;
    m.pose.position.x = Twm.getPosition().x();
    m.pose.position.y = Twm.getPosition().y();
    m.pose.position.z = Twm.getPosition().z();
    m.pose.orientation.w = Twm.getRotation().w();
    m.pose.orientation.x = Twm.getRotation().x();
    m.pose.orientation.y = Twm.getRotation().y();
    m.pose.orientation.z = Twm.getRotation().z();
  }

  pub.publish(ma);
}

void scaleLog10Ntimes(const std::vector<double>& values, const size_t log_times,
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

void rotTransToAxisMarkerArray(const rpg::PoseVec& Twb_vec,
                               const std::string& ns, const double size,
                               visualization_msgs::MarkerArray* ma)
{
  act_map::RotMatVec Rwb_vec(Twb_vec.size());
  act_map::Vec3dVec twb_vec(Twb_vec.size());
  for (size_t i = 0; i < Twb_vec.size(); i++)
  {
    Rwb_vec[i] = Twb_vec[i].getRotation().getRotationMatrix();
    twb_vec[i] = Twb_vec[i].getPosition();
  }
  rotTransToAxisMarkerArray(Rwb_vec, twb_vec, ns, size, ma);
}

void rotTransToAxisMarkerArray(const act_map::RotMatVec& Rwb_vec,
                               const act_map::Vec3dVec& pos_vec,
                               const std::string& ns, const double size,
                               visualization_msgs::MarkerArray* ma)
{
  CHECK_EQ(Rwb_vec.size(), pos_vec.size());
  CHECK_NOTNULL(ma);
  if (Rwb_vec.size() == 0)
  {
    return;
  }
  const size_t N = Rwb_vec.size();

  act_map::Vec3dVec axis_vec;
  axis_vec.emplace_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  axis_vec.emplace_back(Eigen::Vector3d(0.0, 1.0, 0.0));
  axis_vec.emplace_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  std::vector<std_msgs::ColorRGBA> color_vec{ getColor("RED"),
                                              getColor("GREEN"),
                                              getColor("BLUE") };

  act_map::Vec3dVec pos_s_vec;
  pos_s_vec.reserve(3 * N);
  act_map::Vec3dVec dir_vec;
  dir_vec.reserve(3 * N);
  std::vector<std_msgs::ColorRGBA> colors;
  colors.reserve(3 * N);
  size_t cnt = 0;
  for (size_t ai = 0; ai < 3; ai++)
  {
    for (size_t vi = 0; vi < N; vi++)
    {
      pos_s_vec.push_back(pos_vec[vi]);
      rpg::Rotation rot(Rwb_vec[vi]);
      dir_vec.push_back(rot.rotate(axis_vec[ai]));
      colors.push_back(color_vec[ai]);
      cnt++;
    }
  }
  CHECK_EQ(cnt, 3 * N);

  std::vector<size_t> ids(3 * N);
  std::iota(ids.begin(), ids.end(), 0);
  directionsToArrowsArray(pos_s_vec, dir_vec, ids, colors, ns, size, ma);
}

}  // namespace act_map_ros
