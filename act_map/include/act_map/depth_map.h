#pragma once

#include "act_map/common.h"

namespace act_map
{
struct DepthMapOptions
{
  LayerOptions depth_layer_opts_;
  double depth_voxel_step_deg_ = 5.0;
};

class DepthMap
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DepthMap() = delete;
  DepthMap(const DepthMap&) = delete;
  DepthMap& operator=(const DepthMap&) = delete;

  DepthMap(const DepthMapOptions& options);

  virtual ~DepthMap();

  inline DepthLayer::Ptr depthLayerPtr()
  {
    return depth_layer_;
  }
  inline const DepthLayer& depthLayerCRef() const
  {
    return *depth_layer_;
  }

  void allocateByPoints(const Vec3dVec& points);

  size_t numInitializedVoxels() const;

  inline size_t numAllocatedBlocks() const
  {
    return depthLayerCRef().getNumberOfAllocatedBlocks();
  }

  inline size_t numAllocatedVoxels() const
  {
    return depthLayerCRef().getNumberOfAllocatedVoxels();
  }

  void allocateUniformWithin(const std::vector<double>& ranges);

  void reset();

  inline voxblox::BlockIndexList& rawBlockIdx()
  {
    return raw_block_indices_;
  }

  inline const voxblox::BlockIndexList& rawBlockIdx() const
  {
    return raw_block_indices_;
  }

  inline DepthBlock::Ptr getDepthBlockPtr(const voxblox::BlockIndex bidx)
  {
    return depth_layer_->getBlockPtrByIndex(bidx);
  }

  inline DepthBlock::ConstPtr
  getDepthBlockPtr(const voxblox::BlockIndex bidx) const
  {
    return depth_layer_->getBlockPtrByIndex(bidx);
  }

  inline voxblox::BlockIndex consumeRawBlockIdx()
  {
    CHECK(!raw_block_indices_.empty());
    voxblox::BlockIndex res_idx = raw_block_indices_.back();
    raw_block_indices_.pop_back();
    return res_idx;
  }

  inline const DepthVoxel*
  getDepthVoxelPtrByCoordinates(const Eigen::Vector3d& pt) const
  {
    return depth_layer_->getVoxelPtrByCoordinates(pt);
  }

  void queryPointsVisibilityAt(const Eigen::Vector3d& pos,
                               const rpg::PositionVec& points,
                               std::vector<VisStatus>* vis) const;

  DepthMapOptions opts_;

  friend std::ostream& operator<<(std::ostream& os, const DepthMap& dm);

  void saveDepthLayer(const std::string& fn) const;

  void loadDepthLayer(const std::string& fn);

  // for testing
  void setBlockConstant(const voxblox::BlockIndex bidx,
                        const DepthVoxel::FloatType& val);
  void setLayerConstant(const DepthVoxel::FloatType& val);
  void setBlockRandom(const voxblox::BlockIndex& bidx);

private:
  DepthLayer::Ptr depth_layer_;
  voxblox::BlockIndexList raw_block_indices_;
};

using DepthMapPtr = std::shared_ptr<DepthMap>;

}  // namespace act_map
