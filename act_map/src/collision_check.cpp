#include "act_map/collision_check.h"

#include "act_map/voxblox_utils.h"

namespace act_map
{
namespace utils
{
CollisionRes doesPointCollideWithOccLayer(
    const OccupancyLayer& occ_layer, const Eigen::Vector3d pt_w,
    const float occ_thresh, const CollisionCheckerOptions& col_ops)
{
  OccupancyBlock::ConstPtr blk_ptr = occ_layer.getBlockPtrByCoordinates(pt_w);
  if (!blk_ptr)
  {
    return CollisionRes::kUnknown;
  }
  act_map::Vec3dVec occupied_vox_centers;
  getCentersOfOccupiedVoxels(*blk_ptr, occ_thresh, &occupied_vox_centers);
  if (occupied_vox_centers.empty())
  {
    return CollisionRes::kFree;
  }

  const size_t n_occ = occupied_vox_centers.size();
  std::vector<double> distances(n_occ);
  for (size_t i = 0; i < occupied_vox_centers.size(); i++)
  {
    distances[i] = (pt_w - occupied_vox_centers[i]).norm();
  }

  double dist_sum = 0;
  for (const double v : distances)
  {
    dist_sum += v;
  }

  // diverse checks
  if ((dist_sum / n_occ) < col_ops.average_dist_thresh)
  {
    return CollisionRes::kCollide;
  }

  auto min_it = std::min_element(distances.begin(), distances.end());
  if (*min_it < col_ops.min_dist_thresh_)
  {
    return CollisionRes::kCollide;
  }

  return CollisionRes::kFree;
}
}
}
