//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map/kernel_layer_integrator.h"

#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>
#include <vi_utils/map.h>

#include <voxblox/integrator/occupancy_integrator.h>
#include <voxblox/utils/layer_utils.h>
#include "act_map/voxblox_utils.h"
#include "act_map/sampler.h"

using namespace act_map;

class KernelLayerIntegratorTest : public ::testing::Test
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

    LayerOptions ker_options;
    uni_layer_ptr_ = std::make_shared<TraceLayer>(ker_options.vox_size,
                                                  ker_options.vox_per_side);
    rpg::PositionVec uni_points;
    utils::generateUniformPointsWithin(
        ker_options.vox_size, x_range_, y_range_, z_range_, &uni_points);
    for (const Eigen::Vector3d& pt : uni_points)
    {
      uni_layer_ptr_->allocateBlockPtrByCoordinates(pt);
    }

    LayerOptions occ_options;
    occ_options.vox_size = 0.01;
    occ_layer_ptr_ = std::make_shared<OccupancyLayer>(occ_options.vox_size,
                                                      occ_options.vox_per_side);
  }

  std::string abs_map_;
  vi::MapPtr map_;
  double x_range_ = 4.0;
  double y_range_ = 4.0;
  double z_range_ = 2.0;

  voxblox::Layer<TraceVoxel>::Ptr uni_layer_ptr_;
  OccupancyLayer::Ptr occ_layer_ptr_;
};

TEST_F(KernelLayerIntegratorTest, testInitialization)
{
  KernelLayerIntegratorOptions options;

  LayerOptions occ_options;
  OccupancyLayer::Ptr occ_layer_ptr = std::make_shared<OccupancyLayer>(
      occ_options.vox_size, occ_options.vox_per_side);

  LayerOptions ker_options;
  InfoLayer::Ptr info_layer_ptr = std::make_shared<InfoLayer>(
      ker_options.vox_size, ker_options.vox_per_side);

  TraceLayer::Ptr trace_layer_ptr = std::make_shared<TraceLayer>(
      ker_options.vox_size, ker_options.vox_per_side);

  KernelLayerIntegrator<InfoVoxel> info_ker_integrator(
      options, occ_layer_ptr, info_layer_ptr);
  KernelLayerIntegrator<TraceVoxel> trace_ker_integrator(
      options, occ_layer_ptr, trace_layer_ptr);
}

TEST_F(KernelLayerIntegratorTest, testTraceLayerBatchUpdate)
{
  // initialization
  utils::setPointsInOccupancyLayer(map_->points_, occ_layer_ptr_.get());

  LayerOptions ker_options;
  TraceLayer::Ptr trace_layer_ptr = std::make_shared<TraceLayer>(
      ker_options.vox_size, ker_options.vox_per_side);

  KernelLayerIntegratorOptions options;
  KernelLayerIntegrator<TraceVoxel> trace_ker_integrator(
      options, occ_layer_ptr_, trace_layer_ptr);

  // Allocating kernel layer
  EXPECT_EQ(0, trace_layer_ptr->getNumberOfAllocatedBlocks());
  rpg::PositionVec uni_points;
  utils::generateUniformPointsWithin(
      ker_options.vox_size, x_range_, y_range_, z_range_, &uni_points);
  for (const Eigen::Vector3d& pt : uni_points)
  {
    trace_layer_ptr->allocateBlockPtrByCoordinates(pt);
  }
  EXPECT_GT(trace_layer_ptr->getNumberOfAllocatedBlocks(), 0);

  // update batch
  voxblox::BlockIndexList cand_blks;
  trace_layer_ptr->getAllAllocatedBlocks(&cand_blks);
  voxblox::BlockIndexList updated;
  EXPECT_FALSE(utils::checkAllBlocksUpdated(*trace_layer_ptr));
  trace_ker_integrator.recomputeFromOccupancyLayer(cand_blks, &updated);
  EXPECT_TRUE(utils::checkAllBlocksUpdated(*trace_layer_ptr));
  EXPECT_EQ(trace_layer_ptr->getNumberOfAllocatedBlocks(), updated.size());
  EXPECT_EQ(utils::getNumOfUpdatedBlocks(*trace_layer_ptr), updated.size());
  voxblox::BlockIndexList layer_updated;
  trace_layer_ptr->getAllUpdatedBlocks(&layer_updated);
  EXPECT_EQ(layer_updated.size(), updated.size());
}

TEST_F(KernelLayerIntegratorTest, testTraceLayerAddDelete)
{
  utils::setPointsInOccupancyLayer(map_->points_, occ_layer_ptr_.get());

  TraceLayer::Ptr zero_layer_ptr(new TraceLayer(*uni_layer_ptr_));
  TraceLayer::Ptr inc_layer_ptr(new TraceLayer(*uni_layer_ptr_));

  // integrator
  KernelLayerIntegratorOptions options;
  KernelLayerIntegrator<TraceVoxel> integrator1(
      options, occ_layer_ptr_, uni_layer_ptr_);
  voxblox::BlockIndexList cand_blks;
  uni_layer_ptr_->getAllAllocatedBlocks(&cand_blks);
  integrator1.recomputeFromOccupancyLayer(cand_blks);

  // add and substract
  voxblox::BlockIndexList updated_blks;
  act_map::Vec3dVec points_occ;
  utils::getCentersOfOccupiedVoxels(
      *occ_layer_ptr_, options.occ_thresh_, &points_occ);
  KernelLayerIntegrator<TraceVoxel> integrator2(
      options, occ_layer_ptr_, inc_layer_ptr);
  voxblox::BlockIndexList all_blks;
  uni_layer_ptr_->getAllAllocatedBlocks(&all_blks);
  integrator2.addPointsToKernelLayer(points_occ, all_blks, &updated_blks);
  EXPECT_EQ(inc_layer_ptr->getNumberOfAllocatedBlocks(), updated_blks.size());
  EXPECT_TRUE(voxblox::utils::isSameLayer(*uni_layer_ptr_, *inc_layer_ptr));
  integrator2.deletePointsFromKernelLayer(points_occ, all_blks, &updated_blks);
  EXPECT_EQ(inc_layer_ptr->getNumberOfAllocatedBlocks(), updated_blks.size());
  EXPECT_TRUE(voxblox::utils::isSameLayer(*zero_layer_ptr, *inc_layer_ptr));
}

TEST_F(KernelLayerIntegratorTest, testVisibleDistance)
{
  //
  Eigen::Matrix3Xd far_points;
  far_points.resize(Eigen::NoChange, 10);
  far_points.setRandom();
  far_points.array() += 100.0;

  utils::setPointsInOccupancyLayer(far_points, occ_layer_ptr_.get());

  // integrator
  KernelLayerIntegratorOptions options;
  KernelLayerIntegrator<TraceVoxel> trace_ker_integrator(
      options, occ_layer_ptr_, uni_layer_ptr_);
  voxblox::BlockIndexList updated;
  voxblox::BlockIndexList cand_blks;
  uni_layer_ptr_->getAllAllocatedBlocks(&cand_blks);
  trace_ker_integrator.recomputeFromOccupancyLayer(cand_blks, &updated);
  EXPECT_TRUE(updated.empty());
}

TEST_F(KernelLayerIntegratorTest, testHeirachicalDistanceCheck)
{
  // these points should be withing the update range
  utils::setPointsInOccupancyLayer(map_->points_, occ_layer_ptr_.get());
  Vec3dVec points_w;
  eigenKXToVecKVec(map_->points_, &points_w);

  LayerOptions ker_options;
  ker_options.vox_size = 1.0;
  ker_options.vox_per_side = 1;
  TraceLayer::Ptr trace_layer_ptr(
      new TraceLayer(ker_options.vox_size, ker_options.vox_per_side));
  trace_layer_ptr->allocateBlockPtrByIndex(voxblox::BlockIndex(0, 0, 0));

  KernelLayerIntegratorOptions options;
  KernelLayerIntegrator<TraceVoxel> trace_ker_integrator(
      options, occ_layer_ptr_, trace_layer_ptr);

  voxblox::VoxelIndex dummy_vox_idx(1, 1, 1);

  Eigen::Vector3d org(0, 0, 0);
  occ_layer_ptr_->allocateBlockPtrByCoordinates(org);
  voxblox::HierarchicalIndexMap points_list_org{
    {occ_layer_ptr_->computeBlockIndexFromCoordinates(org), { dummy_vox_idx }}
  };

  Eigen::Vector3d far(100, 100, 100);
  occ_layer_ptr_->allocateBlockPtrByCoordinates(far);
  voxblox::HierarchicalIndexMap points_list_far{
    {occ_layer_ptr_->computeBlockIndexFromCoordinates(far), { dummy_vox_idx }}
  };

  Eigen::Vector3d bound_in(
      trace_layer_ptr->block_size() / 2 +
          trace_ker_integrator.getBlockCenterCheckThresh() -
          occ_layer_ptr_->block_size(),
      trace_layer_ptr->block_size() / 2,
      trace_layer_ptr->block_size() / 2);
  occ_layer_ptr_->allocateBlockPtrByCoordinates(bound_in);
  voxblox::HierarchicalIndexMap points_list_bound_in{
    {occ_layer_ptr_->computeBlockIndexFromCoordinates(bound_in),
    { dummy_vox_idx }}
  };

  Eigen::Vector3d bound_out(
      trace_layer_ptr->block_size() / 2 +
          trace_ker_integrator.getBlockCenterCheckThresh() +
          occ_layer_ptr_->block_size(),
      trace_layer_ptr->block_size() / 2,
      trace_layer_ptr->block_size() / 2);
  occ_layer_ptr_->allocateBlockPtrByCoordinates(bound_out);
  voxblox::HierarchicalIndexMap points_list_bound_out{
    {occ_layer_ptr_->computeBlockIndexFromCoordinates(bound_out),
    { dummy_vox_idx }}
  };

  voxblox::BlockIndexList updated;
  voxblox::BlockIndexList cand_blks;
  trace_layer_ptr->getAllAllocatedBlocks(&cand_blks);

  std::vector<voxblox::HierarchicalIndexMap> test_points_list{
    points_list_org,
    points_list_far,
    points_list_bound_in,
    points_list_bound_out
  };
  Vec3dVec test_blk_cs{ org, far, bound_in, bound_out };
  std::vector<KernelUpdateAction> test_actions{ KernelUpdateAction::kReCompute,
                                                KernelUpdateAction::kAdd,
                                                KernelUpdateAction::kDelete };
  std::vector<size_t> expect_res{ cand_blks.size(), 0, cand_blks.size(), 0 };

  for (const auto& action : test_actions)
  {
    for (size_t i = 0; i < test_points_list.size(); i++)
    {
      trace_ker_integrator.updateKernelLayerFromPoints(
          { test_blk_cs[i] },
          { points_w },
          trace_ker_integrator.getBlockCenterCheckThresh(),
          cand_blks,
          action,
          &updated);
      EXPECT_EQ(expect_res[i], updated.size());

      trace_ker_integrator.updateKernelLayerFromPoints(
          test_points_list[i],
          trace_ker_integrator.getBlockCenterCheckThresh(),
          cand_blks,
          action,
          &updated);
      EXPECT_EQ(expect_res[i], updated.size());
    }
  }
}

RPG_COMMON_TEST_MAIN
{
}
