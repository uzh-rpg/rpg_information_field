#pragma once

#include <act_map/common.h>

namespace act_map
{
namespace utils
{
struct CollisionCheckerOptions
{
  double min_dist_thresh_ = 0.1;
  double average_dist_thresh = 0.2;
};

enum class CollisionRes
{
  kUnknown,
  kCollide,
  kFree
};

CollisionRes doesPointCollideWithOccLayer(
    const OccupancyLayer& occ_layer, const Eigen::Vector3d pt_w,
    const float occ_thresh, const CollisionCheckerOptions& col_ops);

template <typename T>
int maskCollidedVoxels(const OccupancyLayer& occ_layer, voxblox::Block<T>* blk,
                       const float occ_thresh,
                       const CollisionCheckerOptions& col_ops)
{
  int n_masked = 0;
  for (size_t vox_i = 0; vox_i < blk->num_voxels(); vox_i++)
  {
    Eigen::Vector3d vc = blk->computeCoordinatesFromLinearIndex(vox_i);
    CollisionRes res =
        doesPointCollideWithOccLayer(occ_layer, vc, occ_thresh, col_ops);
    if (res == CollisionRes::kCollide)
    {
      blk->setVoxMasked(true, vox_i);
      n_masked++;
    }
  }
  return n_masked;
}
}
}
