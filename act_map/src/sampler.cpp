//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map/sampler.h"

#include <random>
#include <ctime>

#include <vi_utils/common_utils.h>

#include "act_map/conversion.h"

namespace act_map
{
namespace utils
{
void sampleRotation(const double angle_res_deg,
                    rpg_common::RotationVec* rot_samples)
{
  CHECK_NOTNULL(rot_samples);
  rot_samples->clear();

  std::vector<double> lat_values;
  std::vector<double> long_values;
  double step_rad = M_PI * (angle_res_deg / 180.0);
  vi_utils::linspace(-M_PI, M_PI, step_rad, &long_values);
  vi_utils::linspace(-M_PI_2, M_PI_2, step_rad, &lat_values);
  const Eigen::Vector3d e3(0, 0, 1);
  for (size_t lat_idx = 0; lat_idx < lat_values.size(); lat_idx++)
  {
    for (size_t long_idx = 0; long_idx < long_values.size(); long_idx++)
    {
      double x =
          std::cos(lat_values[lat_idx]) * std::cos(long_values[long_idx]);
      double y =
          std::cos(lat_values[lat_idx]) * std::sin(long_values[long_idx]);
      double z = std::sin(lat_values[lat_idx]);

      Eigen::Vector3d e3c_w(x, y, z);
      double angle = std::acos(e3c_w.dot(e3));
      Eigen::Vector3d axis = e3.cross(e3c_w);
      axis.normalize();
      VLOG(10) << "Angle: " << angle << ", axis:\n" << axis;
      Eigen::Vector3d scaled_axis = angle * axis;
      rpg::Rotation rot_wc(scaled_axis);

      // check
      Eigen::Vector3d rotated_e3c_w = rot_wc.rotate(e3);
      VLOG(10) << "e3c_w:\n" << e3c_w << "\ne3:\n" << e3;
      CHECK(rotated_e3c_w.isApprox(e3c_w));

      rot_samples->push_back(rot_wc);
    }
  }
}

void generateRandomPointsWithin(const int N,
                                const double x_low,
                                const double x_high,
                                const double y_low,
                                const double y_high,
                                const double z_low,
                                const double z_high,
                                rpg::PositionVec* points)
{
  CHECK_NOTNULL(points);
  points->clear();

  CHECK_GT(x_high, x_low);
  CHECK_GT(y_high, y_low);
  CHECK_GT(z_high, z_low);

  std::srand(std::time(0));
  std::random_device rd;
  std::mt19937 rng(rd());

  std::uniform_real_distribution<> x_dis(x_low, x_high);
  std::uniform_real_distribution<> y_dis(y_low, y_high);
  std::uniform_real_distribution<> z_dis(z_low, z_high);

  for (int i = 0; i < N; i++)
  {
    points->emplace_back(Eigen::Vector3d(x_dis(rng), y_dis(rng), z_dis(rng)));
  }
}

void generateRandomPointsWithin(const int N,
                                const double r_low,
                                const double r_high,
                                rpg::PositionVec* points)
{
  CHECK_NOTNULL(points);
  points->clear();
  CHECK_GT(r_high, r_low);
  CHECK_GT(r_high, 0);
  CHECK_GT(r_low, 0);

  std::srand(std::time(0));
  std::random_device rd;
  std::mt19937 rng(rd());

  std::uniform_real_distribution<> r_dis(r_low, r_high);

  for (int i = 0; i < N; i++)
  {
    Eigen::Vector3d f;
    f.setRandom();
    f.normalize();
    points->emplace_back(r_dis(rng) * f);
  }
}

void generateRandomPointsWithin(const int N,
                                const double r_low,
                                const double r_high,
                                Eigen::Matrix3Xd* points)
{
  CHECK_NOTNULL(points);
  rpg::PositionVec pos_vec;
  generateRandomPointsWithin(N, r_low, r_high, &pos_vec);
  VecKVecToEigenKX(pos_vec, points);
}

void generateRandomPointsWithin(const int N,
                                const double x_low,
                                const double x_high,
                                const double y_low,
                                const double y_high,
                                const double z_low,
                                const double z_high,
                                Eigen::Matrix3Xd* points)
{
  CHECK_NOTNULL(points);
  rpg::PositionVec pos_vec;
  generateRandomPointsWithin(
      N, x_low, x_high, y_low, y_high, z_low, z_high, &pos_vec);
  VecKVecToEigenKX(pos_vec, points);
}

void generateUniformPointsWithin(const double step,
                                 const double x_low,
                                 const double x_high,
                                 const double y_low,
                                 const double y_high,
                                 const double z_low,
                                 const double z_high,
                                 std::vector<double>* xvalues,
                                 std::vector<double>* yvalues,
                                 std::vector<double>* zvalues)
{
  CHECK_GT(x_high, x_low);
  CHECK_GT(y_high, y_low);
  CHECK_GT(z_high, z_low);
  CHECK_NOTNULL(xvalues);
  xvalues->clear();
  CHECK_NOTNULL(yvalues);
  yvalues->clear();
  CHECK_NOTNULL(zvalues);
  zvalues->clear();

  vi_utils::linspace(x_low, x_high, step, xvalues);
  vi_utils::linspace(y_low, y_high, step, yvalues);
  vi_utils::linspace(z_low, z_high, step, zvalues);
}

void generateUniformPointsWithin(const double step,
                                 const double x_low,
                                 const double x_high,
                                 const double y_low,
                                 const double y_high,
                                 const double z_low,
                                 const double z_high,
                                 rpg::PositionVec* points)
{
  CHECK_NOTNULL(points);

  std::vector<double> xvalues, yvalues, zvalues;
  generateUniformPointsWithin(step,
                              x_low,
                              x_high,
                              y_low,
                              y_high,
                              z_low,
                              z_high,
                              &xvalues,
                              &yvalues,
                              &zvalues);
  const size_t kNx = xvalues.size();
  const size_t kNy = yvalues.size();
  const size_t kNz = zvalues.size();
  points->resize(kNx * kNy * kNz);
  size_t cnt = 0;
  for (size_t ix = 0; ix < kNx; ix++)
  {
    for (size_t iy = 0; iy < kNy; iy++)
    {
      for (size_t iz = 0; iz < kNz; iz++)
      {
        (*points)[cnt].x() = xvalues[ix];
        (*points)[cnt].y() = yvalues[iy];
        (*points)[cnt].z() = zvalues[iz];
        cnt++;
      }
    }
  }
  CHECK_EQ(points->size(), cnt);
}

void generateUniformPointsWithin(const double step,
                                 const double x_range,
                                 const double y_range,
                                 const double z_range,
                                 rpg::PositionVec* points)
{
  generateUniformPointsWithin(step,
                              -x_range / 2,
                              x_range / 2,
                              -y_range / 2,
                              y_range / 2,
                              -z_range / 2,
                              z_range / 2,
                              points);
}
}
}
