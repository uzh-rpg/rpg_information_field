#pragma once

#include <cstdint>
#include <string>

#include "act_map/voxblox/core/color.h"
#include "act_map/voxblox/core/common.h"

namespace act_map {

namespace voxblox {

struct TsdfVoxel {
  float distance = 0.0f;
  float weight = 0.0f;
  Color color;
};

struct EsdfVoxel {
  float distance = 0.0f;

  bool observed = false;
  // Whether the voxel was copied from the TSDF (false) or created from a pose
  // or some other source (true). This member is not serialized!!!
  bool hallucinated = false;
  bool in_queue = false;
  bool fixed = false;

  // Relative direction toward parent. If itself, then either uninitialized
  // or in the fixed frontier.
  Eigen::Vector3i parent = Eigen::Vector3i::Zero();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct OccupancyVoxel {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  float probability_log = 0.0f;
  // average view directions, should be of L2 norm 1
  Eigen::Vector3f aver_view_from_pt = Eigen::Vector3f::Zero();
  bool observed = false;
};

struct IntensityVoxel {
  float intensity = 0.0f;
  float weight = 0.0f;
};

// Used for serialization only.
namespace voxel_types {
const std::string kNotSerializable = "not_serializable";
const std::string kTsdf = "tsdf";
const std::string kEsdf = "esdf";
const std::string kOccupancy = "occupancy";
const std::string kIntensity = "intensity";
}  // namespace voxel_types

template <typename Type>
std::string getVoxelType() {
  return voxel_types::kNotSerializable;
}

template <>
inline std::string getVoxelType<TsdfVoxel>() {
  return voxel_types::kTsdf;
}

template <>
inline std::string getVoxelType<EsdfVoxel>() {
  return voxel_types::kEsdf;
}

template <>
inline std::string getVoxelType<OccupancyVoxel>() {
  return voxel_types::kOccupancy;
}

template <>
inline std::string getVoxelType<IntensityVoxel>() {
  return voxel_types::kIntensity;
}

}  // namespace voxblox

} // namespace act_map

