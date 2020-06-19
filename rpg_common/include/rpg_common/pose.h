//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#pragma once

#include <kindr/minimal/quat-transformation.h>

#include "rpg_common/aligned.h"

namespace rpg_common {

typedef kindr::minimal::QuatTransformation Pose;
typedef kindr::minimal::RotationQuaternion Rotation;
typedef Eigen::Vector3d Position;

using PoseVec = Aligned<std::vector, Pose>;
using PositionVec = Aligned<std::vector, Position>;
using RotationVec = Aligned<std::vector, Rotation>;

}  // namespace rpg_common
namespace rpg = rpg_common;
