#include "act_map/voxblox_utils.h"

#include "act_map/voxblox/integrator/occupancy_integrator.h"
#include "act_map/sampler.h"
#include "act_map/optim_orient.h"

namespace act_map
{
namespace utils
{
void setPointsInOccupancyLayer(const Eigen::Matrix3Xd& points_w,
                               act_map::OccupancyLayer* occ_layer)
{
  voxblox::OccupancyIntegrator::Config occ_inte_cfg;
  voxblox::OccupancyIntegrator occ_integrator(occ_inte_cfg, occ_layer);
  for (int i = 0; i < 10; i++)
  {
    rpg::Pose T_w_c;
    T_w_c.setIdentity();
    Vec3dVec points_c_vec;
    eigenKXToVecKVec(points_w, &points_c_vec);
    occ_integrator.integratePointCloud(T_w_c, points_c_vec);
  }
}

void getViewDirsOfOccupiedVoxels(const OccupancyLayer& layer,
                                 const float occ_thresh, Vec3dVec* view_dirs)
{
  getFromAllVoxels<OccupancyVoxel, Eigen::Vector3d>(
      layer, getViewDirOccVoxel,
      std::bind(isOccupancyVoxelOccupied, std::placeholders::_1, occ_thresh),
      view_dirs);
}

void getCentersOfOccupiedVoxels(const OccupancyLayer& occ_layer,
                                const float occ_thresh, Vec3dVec* blk_cs,
                                V3dVecVec* blk_points_w,
                                V3dVecVec* blk_view_dirs)
{
  CHECK_NOTNULL(blk_cs);
  CHECK_NOTNULL(blk_points_w);
  blk_cs->clear();
  blk_points_w->clear();
  if (blk_view_dirs)
  {
    blk_view_dirs->clear();
  }

  voxblox::BlockIndexList occ_blks;
  occ_layer.getAllAllocatedBlocks(&occ_blks);
  for (const voxblox::BlockIndex& bidx : occ_blks)
  {
    const OccupancyBlock& blk_i = occ_layer.getBlockByIndex(bidx);
    Vec3dVec points_i;
    Vec3dVec views_i;
    for (size_t vidx = 0; vidx < blk_i.num_voxels(); vidx++)
    {
      const OccupancyVoxel& vox_i = blk_i.getVoxelByLinearIndex(vidx);
      if (isOccupancyVoxelOccupied(vox_i, occ_thresh))
      {
        Eigen::Vector3d vox_c = blk_i.computeCoordinatesFromLinearIndex(vidx);
        points_i.emplace_back(vox_c);
        if (blk_view_dirs)
        {
          views_i.emplace_back(
              blk_i.getVoxelByLinearIndex(vidx).aver_view_from_pt.cast<double>());
        }
      }
    }
    if (!points_i.empty())
    {
      Eigen::Vector3d blk_c;
      getBlockCenterFromBlk(blk_i, &blk_c);
      blk_cs->emplace_back(blk_c);
      blk_points_w->emplace_back(points_i);
      if (blk_view_dirs)
      {
        blk_view_dirs->emplace_back(views_i);
      }
    }
  }
}

void getCentersOfOccupiedVoxels(const OccupancyLayer& layer,
                                const float occ_thresh, Vec3dVec* points_w)
{
  getFromAllVoxels<OccupancyVoxel, Eigen::Vector3d>(
      layer, getVoxelCenter<OccupancyVoxel>,
      std::bind(isOccupancyVoxelOccupied, std::placeholders::_1, occ_thresh),
      points_w);
}

void getCentersOfOccupiedVoxels(const voxblox::Block<OccupancyVoxel>& blk,
                                const float occ_thresh, Vec3dVec* points_w)
{
  getFromAllVoxels<OccupancyVoxel, Eigen::Vector3d>(
      blk, getVoxelCenter<OccupancyVoxel>,
      std::bind(isOccupancyVoxelOccupied, std::placeholders::_1, occ_thresh),
      points_w);
}

size_t countNumOccupiedVoxels(const OccupancyLayer& layer,
                              const float occ_thresh)
{
  return countAllVoxelsIf<OccupancyVoxel>(
      layer,
      std::bind(isOccupancyVoxelOccupied, std::placeholders::_1, occ_thresh));
}

}  // namespace utils
}  // namespace act_map
