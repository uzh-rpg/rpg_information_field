#pragma once

#include <rpg_common/eigen_type.h>
#include <rpg_common/pose.h>

namespace vi_utils
{
using ImuStamps = Eigen::Matrix<int64_t, Eigen::Dynamic, 1>;
using ImuAccGyrContainer = rpg::Matrix6X;
// Order: Accelerometer, Gyroscope
using ImuAccGyr = rpg::Vector6;
}
namespace vi
{
using ImuStamps = vi_utils::ImuStamps;
using ImuAccGyrContainer = vi_utils::ImuAccGyrContainer;
using ImuAccGyr = vi_utils::ImuAccGyr;

}
