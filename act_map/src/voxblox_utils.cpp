//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map/voxblox_utils.h"

#include <voxblox/integrator/occupancy_integrator.h>

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

void getCentersOfOccupiedVoxels(const OccupancyLayer& occ_layer,
                                const float occ_thresh,
                                Vec3dVec* blk_cs,
                                V3dVecVec* blk_points_w)
{
  CHECK_NOTNULL(blk_cs);
  CHECK_NOTNULL(blk_points_w);
  blk_cs->clear();
  blk_points_w->clear();

  voxblox::BlockIndexList occ_blks;
  occ_layer.getAllAllocatedBlocks(&occ_blks);
  for (const voxblox::BlockIndex& bidx : occ_blks)
  {
    const OccupancyBlock& blk_i = occ_layer.getBlockByIndex(bidx);
    Vec3dVec points_i;
    for (size_t vidx = 0; vidx < blk_i.num_voxels(); vidx++)
    {
      const OccupancyVoxel& vox_i = blk_i.getVoxelByLinearIndex(vidx);
      if (isOccupancyVoxelOccupied(vox_i, occ_thresh))
      {
        Eigen::Vector3d vox_c = blk_i.computeCoordinatesFromLinearIndex(vidx);
        points_i.emplace_back(vox_c);
      }
    }
    if (!points_i.empty())
    {
      Eigen::Vector3d blk_c;
      getBlockCenterFromBlk(blk_i, &blk_c);
      blk_cs->emplace_back(blk_c);
      blk_points_w->emplace_back(points_i);
    }
  }
}

void getCentersOfOccupiedVoxels(const OccupancyLayer& layer,
                                const float occ_thresh,
                                Vec3dVec* points_w)
{
  getFromAllVoxels<OccupancyVoxel, Eigen::Vector3d>(
      layer,
      getVoxelCenter<OccupancyVoxel>,
      std::bind(isOccupancyVoxelOccupied, std::placeholders::_1, occ_thresh),
      points_w);
}

void getCentersOfOccupiedVoxels(const voxblox::Block<OccupancyVoxel>& blk,
                                const float occ_thresh,
                                Vec3dVec* points_w)
{
  getFromAllVoxels<OccupancyVoxel, Eigen::Vector3d>(
      blk,
      getVoxelCenter<OccupancyVoxel>,
      std::bind(isOccupancyVoxelOccupied, std::placeholders::_1, occ_thresh),
      points_w);
}

void getBestViewsClosed(const act_map::TraceLayer& tl,
                        const voxblox::BlockIndexList& blk_idxs,
                        const double k1,
                        const double k2,
                        const int samples_per_side,
                        act_map::Vec3dVec* vox_cs,
                        act_map::Vec3dVec* best_views,
                        voxblox::LongIndexVector* global_idxs)
{
  CHECK_NOTNULL(vox_cs);
  CHECK_NOTNULL(best_views);
  CHECK_GT(samples_per_side, 0);

  for (const voxblox::BlockIndex blk_idx : blk_idxs)
  {
    const TraceBlock& blk = tl.getBlockByIndex(blk_idx);
    voxblox::VoxelIndexList sub_vox_idxs;
    subsampleVoxelIndices(blk, samples_per_side, &sub_vox_idxs);
    for (const voxblox::VoxelIndex& vox_idx : sub_vox_idxs)
    {
      size_t lin_idx = blk.computeLinearIndexFromVoxelIndex(vox_idx);
      if (!blk.isVoxDataValid(lin_idx))
      {
        continue;
      }
      const TraceVoxel& vox = blk.getVoxelByVoxelIndex(vox_idx);
      Eigen::Vector3d pos = blk.computeCoordinatesFromVoxelIndex(vox_idx);
      Eigen::Vector3d bview;
      minTraceDirectionUnconstrained(k1, k2, vox.K1, vox.K2, &bview);

      vox_cs->emplace_back(pos);
      best_views->emplace_back(bview * (-1));
      global_idxs->emplace_back(
          voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
              blk_idx, vox_idx, static_cast<int>(blk.voxels_per_side())));
    }
  }
}

void getBestViewsSample(const act_map::TraceLayer& tl,
                        const voxblox::BlockIndexList& blk_idxs,
                        const double k1,
                        const double k2,
                        const double k3,
                        const int samples_per_side,
                        act_map::Vec3dVec* vox_cs,
                        act_map::Vec3dVec* best_views,
                        std::vector<double>* values,
                        voxblox::LongIndexVector* global_idxs)
{
  CHECK_NOTNULL(vox_cs);
  vox_cs->clear();
  CHECK_NOTNULL(best_views);
  best_views->clear();
  CHECK_NOTNULL(values);
  values->clear();
  CHECK_NOTNULL(global_idxs);
  global_idxs->clear();
  static rpg::RotationVec rot_samples;
  if (rot_samples.empty())
  {
    utils::sampleRotation(10.0, &rot_samples);
  }

  for (const voxblox::BlockIndex blk_idx : blk_idxs)
  {
    const TraceBlock& blk = tl.getBlockByIndex(blk_idx);
    if (!blk.activated())
    {
      continue;
    }
    voxblox::VoxelIndexList sub_vox_idxs;
    subsampleVoxelIndices(blk, samples_per_side, &sub_vox_idxs);
    for (const voxblox::VoxelIndex& vox_idx : sub_vox_idxs)
    {
      size_t lin_idx = blk.computeLinearIndexFromVoxelIndex(vox_idx);
      if (!blk.isVoxDataValid(lin_idx))
      {
        continue;
      }
      const TraceVoxel& vox = blk.getVoxelByVoxelIndex(vox_idx);
      Eigen::Vector3d pos = blk.computeCoordinatesFromVoxelIndex(vox_idx);
      Eigen::Vector3d bview;
      double val;
      optim_orient::getOptimViewFromTraceKernels(
          rot_samples, k1, k2, k3, vox.K1, vox.K2, vox.K3, &bview, &val);

      vox_cs->emplace_back(pos);
      best_views->emplace_back(bview);
      global_idxs->emplace_back(
          voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
              blk_idx, vox_idx, static_cast<int>(blk.voxels_per_side())));
      values->emplace_back(val);
    }
  }
}

CollisionRes
doesPointCollideWithOccLayer(const OccupancyLayer& occ_layer,
                             const Eigen::Vector3d pt_w,
                             const float occ_thresh,
                             const CollisionCheckerOptions& col_ops)
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

}  // utils
}  // act_map
