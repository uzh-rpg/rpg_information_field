#pragma once

#include "act_map/voxblox/core/color.h"
#include "act_map/voxblox/core/common.h"
#include "act_map/voxblox/core/voxel.h"
namespace act_map {
namespace voxblox {
template <typename VoxelType>
void mergeVoxelAIntoVoxelB(const VoxelType& voxel_A, VoxelType* voxel_B);

template <>
void mergeVoxelAIntoVoxelB(const TsdfVoxel& voxel_A, TsdfVoxel* voxel_B);

template <>
void mergeVoxelAIntoVoxelB(const EsdfVoxel& voxel_A, EsdfVoxel* voxel_B);

template <>
void mergeVoxelAIntoVoxelB(const OccupancyVoxel& voxel_A,
                           OccupancyVoxel* voxel_B);

}  // namespace voxblox
}  // namespace act_map

