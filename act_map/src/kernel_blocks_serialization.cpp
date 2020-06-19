//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "voxblox/core/block.h"
#include "act_map/kernel_voxels.h"

namespace voxblox
{
// deserialization
// 32 bit data
template <>
void Block<act_map::TraceVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data)
{
}

template <>
void Block<act_map::InfoVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data)
{
}
// 64 bit
template <>
void Block<act_map::TraceVoxel>::deserializeFromIntegers(
    const std::vector<uint64_t>& data)
{
  constexpr size_t kNumDataPacketsPerVoxel = 13u;
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, data.size());
  for (size_t vox_i = 0, data_start_i = 0;
       vox_i < num_voxels_; vox_i++, data_start_i+=kNumDataPacketsPerVoxel)
  {
    act_map::TraceVoxel& vox = voxels_[vox_i];
    vox.deserializeFromIntegers(data, data_start_i);
  }
}

template <>
void Block<act_map::InfoVoxel>::deserializeFromIntegers(
    const std::vector<uint64_t>& data)
{
  constexpr size_t kNumDataPacketsPerVoxel = 468u;
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, data.size());
  for (size_t vox_i = 0, data_start_i = 0;
       vox_i < num_voxels_; vox_i++, data_start_i+=kNumDataPacketsPerVoxel)
  {
    act_map::InfoVoxel& vox = voxels_[vox_i];
    vox.deserializeFromIntegers(data, data_start_i);
  }
}

// serialization
// 32 bit
template <>
void Block<act_map::TraceVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const
{
}

template <>
void Block<act_map::InfoVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const
{
}
// 64 bit
template <>
void Block<act_map::TraceVoxel>::serializeToIntegers(
    std::vector<uint64_t>* data) const
{
  CHECK_NOTNULL(data);
  constexpr size_t kNumDataPacketsPerVoxel = 13u;
  data->clear();
  data->reserve(num_voxels_ * kNumDataPacketsPerVoxel);
  for (size_t voxel_idx = 0u; voxel_idx < num_voxels_; ++voxel_idx)
  {
    const act_map::TraceVoxel& voxel = voxels_[voxel_idx];
    std::vector<uint64_t> data_i;
    voxel.serializeToIntegers(&data_i);
    data->insert(data->end(), data_i.begin(), data_i.end());
  }
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, data->size());
}

template <>
void Block<act_map::InfoVoxel>::serializeToIntegers(
    std::vector<uint64_t>* data) const
{
  CHECK_NOTNULL(data);
  constexpr size_t kNumDataPacketsPerVoxel = 468u;
  data->clear();
  data->reserve(num_voxels_ * kNumDataPacketsPerVoxel);
  for (size_t voxel_idx = 0u; voxel_idx < num_voxels_; ++voxel_idx)
  {
    const act_map::InfoVoxel& voxel = voxels_[voxel_idx];
    std::vector<uint64_t> data_i;
    voxel.serializeToIntegers(&data_i);
    data->insert(data->end(), data_i.begin(), data_i.end());
  }
  CHECK_EQ(num_voxels_ * kNumDataPacketsPerVoxel, data->size());
}
}
