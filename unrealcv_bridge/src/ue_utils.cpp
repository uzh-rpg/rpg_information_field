#include "unrealcv_bridge/ue_utils.hpp"

namespace unrealcv_bridge
{
void rayDepthToZDepth(const cv::Mat& ray_depth, const float f, cv::Mat* z_depth)
{
  CHECK_EQ(ray_depth.type(), CV_32F);
  z_depth->create(ray_depth.rows, ray_depth.cols, CV_32F);

  const int width = ray_depth.cols;
  const int height = ray_depth.rows;

  const float yc = 0.5f * static_cast<float>(height) - 0.5f;
  const float xc = 0.5f * static_cast<float>(width) - 0.5f;
  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      const float point_depth = ray_depth.at<float>(y, x);
      const float dx = static_cast<float>(x) - xc;
      const float dy = static_cast<float>(y) - yc;
      const float distance_from_center = std::sqrt(dx * dx + dy * dy);
      const float z_d =
          point_depth /
          std::sqrt(1.0f + std::pow(distance_from_center / f, 2.0f));
      z_depth->at<float>(y, x) = z_d;
    }
  }
}

void UEPose::fromEigen(const Eigen::MatrixXd& row_vec, const bool pos_first)
{
  CHECK_EQ(row_vec.cols(), 6);
  CHECK_EQ(row_vec.rows(), 1);

  if (pos_first)
  {
    pitch = row_vec(0, 3);
    yaw = row_vec(0, 4);
    roll = row_vec(0, 5);
    x = row_vec(0, 0);
    y = row_vec(0, 1);
    z = row_vec(0, 2);
  }
  else
  {
    pitch = row_vec(0, 0);
    yaw = row_vec(0, 1);
    roll = row_vec(0, 2);
    x = row_vec(0, 3);
    y = row_vec(0, 4);
    z = row_vec(0, 5);
  }
}

void UEPose::fromUnrealCVStr(const std::string &str)
{
  std::vector<double> xyzpyr;
  std::istringstream iss(str);
  std::string val;
  while (std::getline(iss, val, ' '))
  {
    xyzpyr.emplace_back(static_cast<double>(std::stod(val)));
  }

  CHECK_EQ(xyzpyr.size(), 6);
  x = xyzpyr[0];
  y = xyzpyr[1];
  z = xyzpyr[2];
  pitch = xyzpyr[3];
  yaw = xyzpyr[4];
  roll = xyzpyr[5];
}
}  // namespace unrealcv_bridge
