//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <rpg_common/pose.h>

namespace act_map
{
// TODO: template the sample functions on the dimension (and type?)
namespace utils
{
void sampleRotation(const double angle_res_deg, rpg::RotationVec* rot_samples);

void generateRandomPointsWithin(const int N,
                                const double x_low,
                                const double x_high,
                                const double y_low,
                                const double y_high,
                                const double z_low,
                                const double z_high,
                                rpg::PositionVec* points);

void generateRandomPointsWithin(const int N,
                                const double x_low,
                                const double x_high,
                                rpg::PositionVec* points);

void generateRandomPointsWithin(const int N,
                                const double x_low,
                                const double x_high,
                                Eigen::Matrix3Xd* points);

void generateRandomPointsWithin(const int N,
                                const double x_low,
                                const double x_high,
                                const double y_low,
                                const double y_high,
                                const double z_low,
                                const double z_high,
                                Eigen::Matrix3Xd* points);

void generateUniformPointsWithin(const double step,
                                 const double x_low,
                                 const double x_high,
                                 const double y_low,
                                 const double y_high,
                                 const double z_low,
                                 const double z_high,
                                 std::vector<double>* xvalues,
                                 std::vector<double>* yvalues,
                                 std::vector<double>* zvalues);

void generateUniformPointsWithin(const double step,
                                 const double x_low,
                                 const double x_high,
                                 const double y_low,
                                 const double y_high,
                                 const double z_low,
                                 const double z_high,
                                 rpg::PositionVec* points);

void generateUniformPointsWithin(const double step,
                                 const double x_range,
                                 const double y_range,
                                 const double z_range,
                                 rpg::PositionVec* points);
}
}
