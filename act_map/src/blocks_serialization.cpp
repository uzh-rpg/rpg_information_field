#include "act_map/voxblox/core/block.h"
#include "act_map/quadratic_factor_voxel.h"
#include "act_map/positional_factor_voxel.h"
#include "act_map/depth_voxel.h"

namespace
{
template <typename T, typename intT>
void blockSerialization(const act_map::voxblox::Block<T>& blk, std::vector<intT>* data)
{
  CHECK_NOTNULL(data);
  const size_t serial_size = blk.getVoxelByLinearIndex(0).serializationSize();
  data->clear();
  data->reserve(blk.num_voxels() * serial_size);
  for (size_t voxel_idx = 0u; voxel_idx < blk.num_voxels(); ++voxel_idx)
  {
    const T& voxel = blk.getVoxelByLinearIndex(voxel_idx);
    std::vector<intT> data_i;
    voxel.serializeToIntegers(&data_i);
    data->insert(data->end(), data_i.begin(), data_i.end());
  }
  CHECK_EQ(blk.num_voxels() * serial_size, data->size());
}

template <typename T, typename intT>
void blockDeserialization(const std::vector<intT>& data, act_map::voxblox::Block<T>* blk)
{
  size_t data_start_i = 0;
  for (size_t vox_i = 0; vox_i < blk->num_voxels(); vox_i++)
  {
    T& vox = blk->getVoxelByLinearIndex(vox_i);
    vox.deserializeFromIntegers(data, data_start_i);
    data_start_i += vox.serializationSize();
  }
  CHECK_EQ(data_start_i, data.size());
}
}  // namespace

namespace act_map
{

namespace voxblox
{
// deserialization
// 32 bit data
template <>
void Block<act_map::QuadTraceVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data)
{
  blockDeserialization(data, this);
}

template <>
void Block<act_map::QuadInfoVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data)
{
  blockDeserialization(data, this);
}

template <>
void Block<act_map::GPTraceVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data)
{
  blockDeserialization(data, this);
}

template <>
void Block<act_map::GPInfoVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data)
{
  blockDeserialization(data, this);
}

template <>
void Block<act_map::QuadPolyTraceVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data)
{
  blockDeserialization(data, this);
}

template <>
void Block<act_map::QuadPolyInfoVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data)
{
  blockDeserialization(data, this);
}

template <>
void Block<act_map::DepthVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data)
{
  blockDeserialization(data, this);
}

// 64 bit
template <>
void Block<act_map::QuadTraceVoxel>::deserializeFromIntegers(
    const std::vector<uint64_t>& data)
{
  blockDeserialization(data, this);
}

template <>
void Block<act_map::QuadInfoVoxel>::deserializeFromIntegers(
    const std::vector<uint64_t>& data)
{
  blockDeserialization(data, this);
}

template <>
void Block<act_map::GPTraceVoxel>::deserializeFromIntegers(
    const std::vector<uint64_t>& data)
{
  blockDeserialization(data, this);
}

template <>
void Block<act_map::GPInfoVoxel>::deserializeFromIntegers(
    const std::vector<uint64_t>& data)
{
  blockDeserialization(data, this);
}

template <>
void Block<act_map::QuadPolyTraceVoxel>::deserializeFromIntegers(
    const std::vector<uint64_t>& data)
{
  blockDeserialization(data, this);
}

template <>
void Block<act_map::QuadPolyInfoVoxel>::deserializeFromIntegers(
    const std::vector<uint64_t>& data)
{
  blockDeserialization(data, this);
}

template <>
void Block<act_map::DepthVoxel>::deserializeFromIntegers(
    const std::vector<uint64_t>& data)
{
  blockDeserialization(data, this);
}

// serialization
// 32 bit
template <>
void Block<act_map::QuadTraceVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const
{
  blockSerialization(*this, data);
}

template <>
void Block<act_map::QuadInfoVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const
{
  blockSerialization(*this, data);
}

template <>
void Block<act_map::GPTraceVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const
{
  blockSerialization(*this, data);
}

template <>
void Block<act_map::GPInfoVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const
{
  blockSerialization(*this, data);
}

template <>
void Block<act_map::QuadPolyTraceVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const
{
  blockSerialization(*this, data);
}

template <>
void Block<act_map::QuadPolyInfoVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const
{
  blockSerialization(*this, data);
}

template <>
void Block<act_map::DepthVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const
{
  blockSerialization(*this, data);
}

// 64 bit
template <>
void Block<act_map::QuadTraceVoxel>::serializeToIntegers(
    std::vector<uint64_t>* data) const
{
  blockSerialization(*this, data);
}

template <>
void Block<act_map::QuadInfoVoxel>::serializeToIntegers(
    std::vector<uint64_t>* data) const
{
  blockSerialization(*this, data);
}

template <>
void Block<act_map::QuadPolyTraceVoxel>::serializeToIntegers(
    std::vector<uint64_t>* data) const
{
  blockSerialization(*this, data);
}

template <>
void Block<act_map::QuadPolyInfoVoxel>::serializeToIntegers(
    std::vector<uint64_t>* data) const
{
  blockSerialization(*this, data);
}

template <>
void Block<act_map::GPTraceVoxel>::serializeToIntegers(
    std::vector<uint64_t>* data) const
{
  blockSerialization(*this, data);
}

template <>
void Block<act_map::GPInfoVoxel>::serializeToIntegers(
    std::vector<uint64_t>* data) const
{
  blockSerialization(*this, data);
}

template <>
void Block<act_map::DepthVoxel>::serializeToIntegers(
    std::vector<uint64_t>* data) const
{
  blockSerialization(*this, data);
}
}  // namespace voxblox
}  // namespace act_map
