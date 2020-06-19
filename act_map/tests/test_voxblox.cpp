//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>
#include <vi_utils/map.h>

#include <voxblox/integrator/occupancy_integrator.h>
#include "act_map/voxblox_utils.h"
#include "act_map/sampler.h"

using namespace act_map;

using namespace act_map;

class VoxbloxTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::string dir;
    std::string fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    abs_map_ = dir + "/test_data/test_map.txt";

    map_.reset(new vi::Map());
    map_->load(abs_map_, std::string());
  }

  std::string abs_map_;
  vi::MapPtr map_;
  double x_range_ = 4.0;
  double y_range_ = 4.0;
  double z_range_ = 2.0;
};

TEST_F(VoxbloxTest, testAllocateBlocks)
{
  LayerOptions occ_options;
  OccupancyLayer::Ptr occ_layer_ptr = std::make_shared<OccupancyLayer>(
      occ_options.vox_size, occ_options.vox_per_side);
  rpg::PositionVec uni_points;
  utils::generateUniformPointsWithin(
      occ_options.vox_size, x_range_, y_range_, z_range_, &uni_points);
  for (const Eigen::Vector3d& pt : uni_points)
  {
    occ_layer_ptr->allocateBlockPtrByCoordinates(pt);
  }

  for (const Eigen::Vector3d& pt : uni_points)
  {
    OccupancyBlock::Ptr blk_ptr = occ_layer_ptr->getBlockPtrByCoordinates(pt);
    EXPECT_TRUE(blk_ptr != nullptr);
    voxblox::OccupancyVoxel* vox = occ_layer_ptr->getVoxelPtrByCoordinates(pt);
    EXPECT_TRUE(vox);
  }
}

TEST_F(VoxbloxTest, testOccupancyIntegration)
{
  LayerOptions occ_options;
  OccupancyLayer::Ptr occ_layer_ptr = std::make_shared<OccupancyLayer>(
      occ_options.vox_size, occ_options.vox_per_side);
  utils::setPointsInOccupancyLayer(map_->points_, occ_layer_ptr.get());

  voxblox::OccupancyIntegrator::Config occ_inte_cfg;
  for (int i = 0; i < map_->points_.cols(); i++)
  {
    Eigen::Vector3d pt = map_->points_.col(i);
    voxblox::OccupancyVoxel* vox = occ_layer_ptr->getVoxelPtrByCoordinates(pt);
    EXPECT_TRUE(vox != nullptr);
    EXPECT_TRUE(vox->observed);
    EXPECT_GT(
        vox->probability_log,
        voxblox::logOddsFromProbability(occ_inte_cfg.threshold_occupancy));
  }

  voxblox::BlockIndexList occ_blk_idxs;
  occ_layer_ptr->getAllAllocatedBlocks(&occ_blk_idxs);
  for (voxblox::BlockIndex blk_i : occ_blk_idxs)
  {
    OccupancyBlock::Ptr blk_ptr = occ_layer_ptr->getBlockPtrByIndex(blk_i);
    EXPECT_TRUE(blk_ptr != nullptr);
    for (size_t vox_i = 0; vox_i < blk_ptr->num_voxels(); vox_i++)
    {
      const voxblox::OccupancyVoxel& vox =
          blk_ptr->getVoxelByLinearIndex(vox_i);
    }
  }
}

TEST_F(VoxbloxTest, testOccupancyStatusUpdate)
{
  LayerOptions occ_options;
  occ_options.vox_size = 0.01;
  OccupancyLayer::Ptr occ_layer_ptr = std::make_shared<OccupancyLayer>(
      occ_options.vox_size, occ_options.vox_per_side);

  voxblox::OccupancyIntegrator::Config occ_inte_cfg;
  voxblox::OccupancyIntegrator occ_integrator(occ_inte_cfg,
                                              occ_layer_ptr.get());
  rpg::Pose T_w_c;
  T_w_c.setIdentity();
  Vec3dVec points_c_vec;
  eigenKXToVecKVec(map_->points_, &points_c_vec);

  voxblox::HierarchicalIndexMap del_pts_w;
  voxblox::HierarchicalIndexMap add_pts_w;
  for (int i = 0; i < 10; i++)
  {
    occ_integrator.integratePointCloud(
        T_w_c, points_c_vec, &del_pts_w, &add_pts_w);
    if (add_pts_w.size() > 0)
    {
      std::cout << "Voxels are marked occupied at " << i << "th iteration.\n";
      break;
    }
  }
  EXPECT_EQ(0, del_pts_w.size());
  EXPECT_GT(add_pts_w.size(), 0);
  size_t n_pts = 0;
  for (const auto& pair : add_pts_w)
  {
    n_pts += pair.second.size();
  }
  EXPECT_EQ(map_->points_.cols(), n_pts);
}

TEST_F(VoxbloxTest, testCollisionWithOccupancy)
{
  LayerOptions occ_options;
  occ_options.vox_size = 0.05;
  OccupancyLayer::Ptr occ_layer_ptr = std::make_shared<OccupancyLayer>(
      occ_options.vox_size, occ_options.vox_per_side);
  utils::setPointsInOccupancyLayer(map_->points_, occ_layer_ptr.get());

  std::srand(std::time(nullptr));
  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_int_distribution<> idx_dist(0, map_->points_.cols() - 1);

  Eigen::Vector3d rpt = map_->points_.col(idx_dist(rng));
  OccupancyBlock::Ptr blk_ptr = occ_layer_ptr->getBlockPtrByCoordinates(rpt);
  EXPECT_TRUE(blk_ptr.get());

  utils::CollisionCheckerOptions col_ops;
  col_ops.min_dist_thresh_ = occ_options.vox_size;
  col_ops.average_dist_thresh = 2.0 * occ_options.vox_size;
  const float occ_thresh = 0.7f;

  // collision check for a single point
  utils::CollisionRes res = utils::doesPointCollideWithOccLayer(
      *occ_layer_ptr, rpt, occ_thresh, col_ops);
  EXPECT_EQ(utils::CollisionRes::kCollide, res);

  Eigen::Vector3d far_pt;
  far_pt.setRandom();
  far_pt = far_pt.array() * 10 + 20;
  res = utils::doesPointCollideWithOccLayer(
      *occ_layer_ptr, far_pt, occ_thresh, col_ops);
  EXPECT_EQ(utils::CollisionRes::kUnknown, res);

  Eigen::Vector3d in_pt;
  in_pt.setRandom();  // [-1, 1]
  in_pt = in_pt.array() * 0.5 + 1;
  in_pt(2) = 0;
  res = utils::doesPointCollideWithOccLayer(
      *occ_layer_ptr, in_pt, occ_thresh, col_ops);
  EXPECT_EQ(utils::CollisionRes::kFree, res);

  // mask a block
  OccupancyLayer::Ptr layer2(new OccupancyLayer(*occ_layer_ptr));
  Eigen::Vector3d rpt2 = map_->points_.col(idx_dist(rng));
  OccupancyBlock::Ptr blk_rpt = layer2->getBlockPtrByCoordinates(rpt2);
  maskCollidedVoxels(*occ_layer_ptr, blk_rpt.get(), occ_thresh, col_ops);
  EXPECT_GT(utils::numOfMaskedVoxels(*blk_rpt), 0);

  Eigen::Vector3d org(0.0, 0.0, 0.0);
  OccupancyBlock::Ptr blk_in = layer2->getBlockPtrByCoordinates(org);
  maskCollidedVoxels(*occ_layer_ptr, blk_in.get(), occ_thresh, col_ops);
  EXPECT_EQ(utils::numOfMaskedVoxels(*blk_in), 0);
}

RPG_COMMON_TEST_MAIN
{
}
