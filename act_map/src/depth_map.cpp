#include "act_map/depth_map.h"

#include "act_map/voxblox/io/layer_io.h"
#include "act_map/voxblox_utils.h"

namespace act_map
{
DepthMap::~DepthMap()
{
}

DepthMap::DepthMap(const DepthMapOptions& options)
  : opts_(options), raw_block_indices_()
{
  depth_layer_ = std::make_shared<DepthLayer>(
      opts_.depth_layer_opts_.vox_size, opts_.depth_layer_opts_.vox_per_side);
}

void DepthMap::reset()
{
  depth_layer_->removeAllBlocks();
  raw_block_indices_.clear();
}

std::ostream& operator<<(std::ostream& os, const DepthMap& dm)
{
  os << "DepthMap:\n"
     << "- vox. size: " << dm.opts_.depth_layer_opts_.vox_size << std::endl
     << "- # vox. per side: " << dm.opts_.depth_layer_opts_.vox_per_side
     << std::endl
     << "- degree step: " << dm.opts_.depth_voxel_step_deg_;

  return os;
}

void DepthMap::allocateByPoints(const Vec3dVec& points)
{
  voxblox::BlockIndexList new_blks;
  utils::allocateBlocksByCoordinatesBatch(points, depth_layer_.get(), &new_blks,
                                          nullptr);
  for (const voxblox::BlockIndex& idx : new_blks)
  {
    DepthBlock::Ptr blk_ptr = depth_layer_->getBlockPtrByIndex(idx);
    for (size_t vox_idx = 0; vox_idx < blk_ptr->num_voxels(); vox_idx++)
    {
      DepthVoxel& vox = blk_ptr->getVoxelByLinearIndex(vox_idx);
      vox.init(blk_ptr->computeCoordinatesFromLinearIndex(vox_idx),
               opts_.depth_voxel_step_deg_);
    }
  }

  raw_block_indices_.insert(raw_block_indices_.begin(), new_blks.begin(),
                            new_blks.end());
}

void DepthMap::allocateUniformWithin(const std::vector<double>& ranges)
{
  Vec3dVec points;
  utils::generateUniformPointsWithin(opts_.depth_layer_opts_.vox_size, ranges,
                                     &points);
  this->allocateByPoints(points);
}

size_t DepthMap::numInitializedVoxels() const
{
  voxblox::BlockIndexList all_blks;
  depth_layer_->getAllAllocatedBlocks(&all_blks);
  size_t cnt = 0;
  for (const voxblox::BlockIndex& idx : all_blks)
  {
    DepthBlock::ConstPtr blk_ptr = depth_layer_->getBlockPtrByIndex(idx);
    for (size_t vox_idx = 0; vox_idx < blk_ptr->num_voxels(); vox_idx++)
    {
      const DepthVoxel& vox = blk_ptr->getVoxelByLinearIndex(vox_idx);
      if (vox.initialized())
      {
        cnt++;
      }
    }
  }

  return cnt;
}

void DepthMap::saveDepthLayer(const std::string& fn) const
{
  voxblox::io::SaveLayer<DepthVoxel>(*depth_layer_, fn);
}

void DepthMap::loadDepthLayer(const std::string& fn)
{
  depth_layer_.reset();
  voxblox::io::LoadLayer<DepthVoxel>(fn, &(depth_layer_));
}

void DepthMap::setBlockConstant(const voxblox::BlockIndex bidx,
                                const DepthVoxel::FloatType& val)
{
  DepthBlock::Ptr blk_ptr = this->getDepthBlockPtr(bidx);
  for (size_t vox_i = 0; vox_i < blk_ptr->num_voxels(); vox_i++)
  {
    blk_ptr->getVoxelByLinearIndex(vox_i).setDepthMap(val);
  }
}

void DepthMap::setLayerConstant(const DepthVoxel::FloatType &val)
{
  voxblox::BlockIndexList blks;
  depth_layer_->getAllAllocatedBlocks(&blks);
  for (const voxblox::BlockIndex& blk_idx : blks)
  {
    this->setBlockConstant(blk_idx, val);
  }
  raw_block_indices_.clear();
}

void DepthMap::setBlockRandom(const voxblox::BlockIndex& bidx)
{
  DepthBlock::Ptr blk_ptr = this->getDepthBlockPtr(bidx);
  for (size_t vox_i = 0; vox_i < blk_ptr->num_voxels(); vox_i++)
  {
    blk_ptr->getVoxelByLinearIndex(vox_i).rayDepthMatRef().setRandom();
  }
}

void DepthMap::queryPointsVisibilityAt(const Eigen::Vector3d& pos,
                                       const rpg::PositionVec& points,
                                       std::vector<VisStatus>* vis) const
{
  CHECK(vis);
  vis->clear();
  vis->reserve(points.size());

  DepthVoxel* vox = depth_layer_->getVoxelPtrByCoordinates(pos);
  if (!vox)
  {
    vis->resize(points.size(), VisStatus::kNotCovered);
    return;
  }

  for (const Eigen::Vector3d& pt : points)
  {
    vis->push_back(vox->queryVisibilityPoint(pt));
  }
}

}  // namespace act_map
