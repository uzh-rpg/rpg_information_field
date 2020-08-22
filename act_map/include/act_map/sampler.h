#pragma once

#include <rpg_common/pose.h>

namespace act_map
{
// TODO: template the sample functions on the dimension (and type?)
namespace utils
{
void sampleRotation(const double angle_res_deg, rpg::RotationVec* rot_samples);

void generateRandomPointsWithin(const int N, const double x_low,
                                const double x_high, const double y_low,
                                const double y_high, const double z_low,
                                const double z_high, rpg::PositionVec* points);

void generateRandomPointsWithin(const int N, const double x_low,
                                const double x_high, rpg::PositionVec* points);

void generateRandomPointsWithin(const int N, const double x_low,
                                const double x_high, Eigen::Matrix3Xd* points);

void generateRandomPointsWithin(const int N, const double x_low,
                                const double x_high, const double y_low,
                                const double y_high, const double z_low,
                                const double z_high, Eigen::Matrix3Xd* points);

void generateRandomPointsWithin(const int N, const double xrange,
                                const double yrange, const double zrange,
                                Eigen::Matrix3Xd* points);

void generateRandomPointsWithin(const int N, const double xrange,
                                const double yrange, const double zrange,
                                rpg::PositionVec* points);

void generateUniformPointsWithin(const double step, const double x_low,
                                 const double x_high, const double y_low,
                                 const double y_high, const double z_low,
                                 const double z_high,
                                 std::vector<double>* xvalues,
                                 std::vector<double>* yvalues,
                                 std::vector<double>* zvalues);

void generateUniformPointsWithin(const double step, const double x_low,
                                 const double x_high, const double y_low,
                                 const double y_high, const double z_low,
                                 const double z_high, rpg::PositionVec* points);

void generateUniformPointsWithin(const double step, const double x_range,
                                 const double y_range, const double z_range,
                                 rpg::PositionVec* points);

void generateUniformPointsWithin(const double step,
                                 const std::vector<double>& ranges,
                                 rpg::PositionVec* points);

inline void getRotationPerturbTwoSides(const Eigen::Matrix3d& Rwc,
                                       const double dtheta, const int idx,
                                       Eigen::Matrix3d* Rwcm,
                                       Eigen::Matrix3d* Rwcp)
{
  Eigen::Vector3d zm = Eigen::Vector3d::Zero();
  zm(idx) -= dtheta;
  Eigen::Vector3d zp = Eigen::Vector3d::Zero();
  zp(idx) += dtheta;
  *Rwcm = rpg::Rotation::exp(zm).getRotationMatrix() * Rwc;
  *Rwcp = rpg::Rotation::exp(zp).getRotationMatrix() * Rwc;
}

void sampleIndicesInRange(const size_t n_samples, const size_t min_idx,
                          const size_t max_idx, std::vector<size_t>* indices);
}  // namespace utils
}  // namespace act_map
