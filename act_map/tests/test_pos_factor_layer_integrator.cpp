#include "act_map/positional_factor_layer_integrator.h"

#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>
#include <vi_utils/map.h>

#include "act_map/voxblox/integrator/occupancy_integrator.h"
#include "act_map/voxblox/utils/layer_utils.h"
#include "act_map/voxblox_utils.h"
#include "act_map/sampler.h"
#include "act_map/common.h"
#include "act_map/depth_map.h"

using namespace act_map;

template <typename T>
class PosFactorLayerIntegratorTest : public ::testing::Test
{
public:
  virtual void SetUp()
  {
    std::string dir;
    std::string fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    abs_map_ = dir + "/test_data/test_map_100.txt";
    const std::string abs_vis_cfg_folder = dir + "/test_data/"
                                                 "fov50_fs50_lm1000_k10";

    setGPVisiblityFromFolder(abs_vis_cfg_folder);
    setQuadPolyVisiblity(QuadVisScoreOptions());

    map_.reset(new vi::Map());
    map_->load(abs_map_, std::string());
    LayerOptions layer_options;
    layer_options.vox_size = 0.3;

    // positional factor layer
    uni_layer_ptr_ = std::make_shared<voxblox::Layer<T>>(
        layer_options.vox_size, layer_options.vox_per_side);
    rpg::PositionVec uni_points;
    utils::generateUniformPointsWithin(layer_options.vox_size, x_range_,
                                       y_range_, z_range_, &uni_points);
    for (const Eigen::Vector3d& pt : uni_points)
    {
      uni_layer_ptr_->allocateBlockPtrByCoordinates(pt);
    }

    DepthMapOptions dm_options;
    dm_options.depth_layer_opts_ = layer_options;
    dm_options.depth_voxel_step_deg_ = 2.0;
    depth_map_ptr_ = std::make_shared<DepthMap>(dm_options);
    depth_map_ptr_->allocateByPoints(uni_points);

    LayerOptions occ_options;
    occ_options.vox_size = 0.02;
    occ_layer_ptr_ = std::make_shared<OccupancyLayer>(occ_options.vox_size,
                                                      occ_options.vox_per_side);

    VisibilityCheckerOptions vis_check_options;
    vis_check_options.max_dist = 10.0;
    vis_check_options.min_dist = 0.1;
    vis_check_ptr_no_dm_ =
        std::make_shared<VisibilityChecker>(vis_check_options);
    vis_check_ptr_dm_ =
        std::make_shared<VisibilityChecker>(vis_check_options, depth_map_ptr_);
  }

  std::string abs_map_;
  vi::MapPtr map_;
  double x_range_ = 1.0;
  double y_range_ = 1.0;
  double z_range_ = 1.0;

  typename voxblox::Layer<T>::Ptr uni_layer_ptr_;
  OccupancyLayer::Ptr occ_layer_ptr_;
  DepthMapPtr depth_map_ptr_;
  VisibilityCheckerPtr vis_check_ptr_dm_;
  VisibilityCheckerPtr vis_check_ptr_no_dm_;
};

using PosFactorVoxelTypes =
    ::testing::Types<QuadInfoVoxel, QuadTraceVoxel, GPInfoVoxel, GPTraceVoxel,
                     QuadPolyInfoVoxel, QuadPolyTraceVoxel>;
// using KernelVoxelTypes =
//    ::testing::Types<TraceVoxel, GPInfoVoxel, GPTraceVoxel>;
TYPED_TEST_CASE(PosFactorLayerIntegratorTest, PosFactorVoxelTypes);

TYPED_TEST(PosFactorLayerIntegratorTest, testInitialization)
{
  PositionalFactorLayerIntegratorOptions options;
  LayerOptions occ_options;
  OccupancyLayer::Ptr occ_layer_ptr = std::make_shared<OccupancyLayer>(
      occ_options.vox_size, occ_options.vox_per_side);

  LayerOptions pos_fac_layer_options;
  QuadInfoLayer::Ptr info_layer_ptr = std::make_shared<QuadInfoLayer>(
      pos_fac_layer_options.vox_size, pos_fac_layer_options.vox_per_side);

  QuadTraceLayer::Ptr trace_layer_ptr = std::make_shared<QuadTraceLayer>(
      pos_fac_layer_options.vox_size, pos_fac_layer_options.vox_per_side);

  PositionalFactorLayerIntegrator<QuadInfoVoxel> info_factor_integrator(
      options, occ_layer_ptr, info_layer_ptr, this->vis_check_ptr_no_dm_);
  PositionalFactorLayerIntegrator<QuadTraceVoxel> trace_factor_integrator(
      options, occ_layer_ptr, trace_layer_ptr, this->vis_check_ptr_no_dm_);

  GPInfoLayer::Ptr gp_info_layer = std::make_shared<GPInfoLayer>(
      pos_fac_layer_options.vox_size, pos_fac_layer_options.vox_per_side);
  PositionalFactorLayerIntegrator<GPInfoVoxel> gp_info_factor_integrator(
      options, occ_layer_ptr, gp_info_layer, this->vis_check_ptr_no_dm_);

  GPTraceLayer::Ptr gp_trace_layer = std::make_shared<GPTraceLayer>(
      pos_fac_layer_options.vox_size, pos_fac_layer_options.vox_per_side);
  PositionalFactorLayerIntegrator<GPTraceVoxel> gp_trace_factor_integrator(
      options, occ_layer_ptr, gp_trace_layer, this->vis_check_ptr_no_dm_);
}

TYPED_TEST(PosFactorLayerIntegratorTest, testLayerBatchUpdate)
{
  // initialization
  utils::setPointsInOccupancyLayer(this->map_->points_,
                                   this->occ_layer_ptr_.get());

  LayerOptions pos_fac_layer_options;
  typename voxblox::Layer<TypeParam>::Ptr layer_ptr =
      std::make_shared<voxblox::Layer<TypeParam>>(
          pos_fac_layer_options.vox_size, pos_fac_layer_options.vox_per_side);

  PositionalFactorLayerIntegratorOptions options;
  PositionalFactorLayerIntegrator<TypeParam> factor_integrator(
      options, this->occ_layer_ptr_, layer_ptr, this->vis_check_ptr_no_dm_);

  // Allocating positional factor layer
  EXPECT_EQ(0, layer_ptr->getNumberOfAllocatedBlocks());
  rpg::PositionVec uni_points;
  utils::generateUniformPointsWithin(pos_fac_layer_options.vox_size,
                                     this->x_range_, this->y_range_,
                                     this->z_range_, &uni_points);
  for (const Eigen::Vector3d& pt : uni_points)
  {
    layer_ptr->allocateBlockPtrByCoordinates(pt);
  }
  EXPECT_GT(layer_ptr->getNumberOfAllocatedBlocks(), 0);

  // update batch
  voxblox::BlockIndexList cand_blks;
  layer_ptr->getAllAllocatedBlocks(&cand_blks);
  voxblox::BlockIndexList updated;
  EXPECT_FALSE(utils::checkAllBlocksUpdated(*layer_ptr));
  factor_integrator.recomputeFromOccupancyLayer(cand_blks, &updated);
  EXPECT_TRUE(utils::checkAllBlocksUpdated(*layer_ptr));
  EXPECT_EQ(layer_ptr->getNumberOfAllocatedBlocks(), updated.size());
  EXPECT_EQ(utils::getNumOfUpdatedBlocks(*layer_ptr), updated.size());
  voxblox::BlockIndexList layer_updated;
  layer_ptr->getAllUpdatedBlocks(&layer_updated);
  EXPECT_EQ(layer_updated.size(), updated.size());
}

TYPED_TEST(PosFactorLayerIntegratorTest, testLayerAddDelete)
{
  utils::setPointsInOccupancyLayer(this->map_->points_,
                                   this->occ_layer_ptr_.get());

  typename voxblox::Layer<TypeParam>::Ptr zero_layer_ptr(
      new voxblox::Layer<TypeParam>(*(this->uni_layer_ptr_)));
  typename voxblox::Layer<TypeParam>::Ptr inc_layer_ptr(
      new voxblox::Layer<TypeParam>(*(this->uni_layer_ptr_)));

  // integrator
  PositionalFactorLayerIntegratorOptions options;
  PositionalFactorLayerIntegrator<TypeParam> integrator1(
      options, this->occ_layer_ptr_, this->uni_layer_ptr_,
      this->vis_check_ptr_no_dm_);
  voxblox::BlockIndexList cand_blks;
  this->uni_layer_ptr_->getAllAllocatedBlocks(&cand_blks);
  integrator1.recomputeFromOccupancyLayer(cand_blks);

  // add and substract
  voxblox::BlockIndexList updated_blks;
  act_map::Vec3dVec points_occ;
  act_map::Vec3dVec occ_view_dirs;
  utils::getCentersOfOccupiedVoxels(*(this->occ_layer_ptr_),
                                    options.occ_thresh_, &points_occ);
  utils::getViewDirsOfOccupiedVoxels(*(this->occ_layer_ptr_),
                                     options.occ_thresh_, &occ_view_dirs);
  PositionalFactorLayerIntegrator<TypeParam> integrator2(
      options, this->occ_layer_ptr_, inc_layer_ptr, this->vis_check_ptr_no_dm_);
  voxblox::BlockIndexList all_blks;
  this->uni_layer_ptr_->getAllAllocatedBlocks(&all_blks);
  integrator2.addPointsToFactorLayer(points_occ, occ_view_dirs, all_blks,
                                     &updated_blks);
  EXPECT_EQ(inc_layer_ptr->getNumberOfAllocatedBlocks(), updated_blks.size());
  EXPECT_TRUE(
      voxblox::utils::isSameLayer(*(this->uni_layer_ptr_), *inc_layer_ptr));
  integrator2.deletePointsFromFactorLayer(points_occ, occ_view_dirs, all_blks,
                                          &updated_blks);
  EXPECT_EQ(inc_layer_ptr->getNumberOfAllocatedBlocks(), updated_blks.size());
  EXPECT_TRUE(voxblox::utils::isSameLayer(*zero_layer_ptr, *inc_layer_ptr));
}

TYPED_TEST(PosFactorLayerIntegratorTest, testVisibleDistance)
{
  //
  Eigen::Matrix3Xd far_points;
  far_points.resize(Eigen::NoChange, 10);
  far_points.setRandom();
  far_points.array() += 100.0;

  utils::setPointsInOccupancyLayer(far_points, this->occ_layer_ptr_.get());

  // integrator
  PositionalFactorLayerIntegratorOptions options;
  PositionalFactorLayerIntegrator<TypeParam> factor_integrator(
      options, this->occ_layer_ptr_, this->uni_layer_ptr_,
      this->vis_check_ptr_no_dm_);
  voxblox::BlockIndexList updated;
  voxblox::BlockIndexList cand_blks;
  this->uni_layer_ptr_->getAllAllocatedBlocks(&cand_blks);
  factor_integrator.recomputeFromOccupancyLayer(cand_blks, &updated);
  EXPECT_TRUE(updated.empty());
}

TYPED_TEST(PosFactorLayerIntegratorTest, testDepthMap)
{
  // set points
  Eigen::Matrix3Xd points;
  points.resize(Eigen::NoChange, 10);
  points.setRandom();
  points.array() += 2.0;
  utils::setPointsInOccupancyLayer(points, this->occ_layer_ptr_.get());

  PositionalFactorLayerIntegratorOptions options;
  voxblox::BlockIndexList updated;
  voxblox::BlockIndexList cand_blks;

  // integrator with depth map
  PositionalFactorLayerIntegrator<TypeParam> pos_fac_integrator_dm(
      options, this->occ_layer_ptr_, this->uni_layer_ptr_,
      this->vis_check_ptr_dm_);
  this->uni_layer_ptr_->getAllAllocatedBlocks(&cand_blks);

  this->depth_map_ptr_->setLayerConstant(0.01);
  pos_fac_integrator_dm.recomputeFromOccupancyLayer(cand_blks, &updated);
  EXPECT_EQ(updated.size(), 0);

  this->depth_map_ptr_->setLayerConstant(100.0);
  pos_fac_integrator_dm.recomputeFromOccupancyLayer(cand_blks, &updated);
  EXPECT_EQ(updated.size(), cand_blks.size());
}

TYPED_TEST(PosFactorLayerIntegratorTest, testHeirachicalDistanceCheck)
{
  // these points should be withing the update range
  utils::setPointsInOccupancyLayer(this->map_->points_,
                                   this->occ_layer_ptr_.get());
  Vec3dVec points_w;
  eigenKXToVecKVec(this->map_->points_, &points_w);

  LayerOptions pos_fac_layer_options;
  pos_fac_layer_options.vox_size = 1.0;
  pos_fac_layer_options.vox_per_side = 1;
  typename voxblox::Layer<TypeParam>::Ptr layer_ptr(
      new voxblox::Layer<TypeParam>(pos_fac_layer_options.vox_size,
                                    pos_fac_layer_options.vox_per_side));
  layer_ptr->allocateBlockPtrByIndex(voxblox::BlockIndex(0, 0, 0));

  PositionalFactorLayerIntegratorOptions options;
  PositionalFactorLayerIntegrator<TypeParam> factor_integrator(
      options, this->occ_layer_ptr_, layer_ptr, this->vis_check_ptr_no_dm_);

  voxblox::VoxelIndex dummy_vox_idx(1, 1, 1);

  Eigen::Vector3d org(0, 0, 0);
  this->occ_layer_ptr_->allocateBlockPtrByCoordinates(org);
  voxblox::HierarchicalIndexMap points_list_org{
    { this->occ_layer_ptr_->computeBlockIndexFromCoordinates(org),
      { dummy_vox_idx } }
  };

  Eigen::Vector3d far(100, 100, 100);
  this->occ_layer_ptr_->allocateBlockPtrByCoordinates(far);
  voxblox::HierarchicalIndexMap points_list_far{
    { this->occ_layer_ptr_->computeBlockIndexFromCoordinates(far),
      { dummy_vox_idx } }
  };

  Eigen::Vector3d bound_in(layer_ptr->block_size() / 2 +
                               factor_integrator.getBlockCenterCheckThresh() -
                               this->occ_layer_ptr_->block_size(),
                           layer_ptr->block_size() / 2,
                           layer_ptr->block_size() / 2);
  this->occ_layer_ptr_->allocateBlockPtrByCoordinates(bound_in);
  voxblox::HierarchicalIndexMap points_list_bound_in{
    { this->occ_layer_ptr_->computeBlockIndexFromCoordinates(bound_in),
      { dummy_vox_idx } }
  };

  Eigen::Vector3d bound_out(layer_ptr->block_size() / 2 +
                                factor_integrator.getBlockCenterCheckThresh() +
                                this->occ_layer_ptr_->block_size(),
                            layer_ptr->block_size() / 2,
                            layer_ptr->block_size() / 2);
  this->occ_layer_ptr_->allocateBlockPtrByCoordinates(bound_out);
  voxblox::HierarchicalIndexMap points_list_bound_out{
    { this->occ_layer_ptr_->computeBlockIndexFromCoordinates(bound_out),
      { dummy_vox_idx } }
  };

  voxblox::BlockIndexList updated;
  voxblox::BlockIndexList cand_blks;
  layer_ptr->getAllAllocatedBlocks(&cand_blks);

  std::vector<voxblox::HierarchicalIndexMap> test_points_list{
    points_list_org, points_list_far, points_list_bound_in,
    points_list_bound_out
  };
  Vec3dVec test_blk_cs{ org, far, bound_in, bound_out };
  std::vector<FactorUpdateAction> test_actions{ FactorUpdateAction::kReCompute,
                                                FactorUpdateAction::kAdd,
                                                FactorUpdateAction::kDelete };
  std::vector<size_t> expect_res{ cand_blks.size(), 0, cand_blks.size(), 0 };

  for (const auto& action : test_actions)
  {
    for (size_t i = 0; i < test_points_list.size(); i++)
    {
      act_map::Vec3dVec view_dirs;
      factor_integrator.updateFactorLayerFromPoints(
          { test_blk_cs[i] }, { points_w }, { view_dirs },
          factor_integrator.getBlockCenterCheckThresh(), cand_blks, action,
          &updated);
      EXPECT_EQ(expect_res[i], updated.size());

      factor_integrator.updateFactorLayerFromPoints(
          test_points_list[i], factor_integrator.getBlockCenterCheckThresh(),
          cand_blks, action, &updated);
      EXPECT_EQ(expect_res[i], updated.size());
    }
  }
}

RPG_COMMON_TEST_MAIN
{
}
