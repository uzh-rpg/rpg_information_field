//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "act_map/layer_evaluator.h"

#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>
#include <vi_utils/map.h>

#include <voxblox/io/layer_io.h>
#include "act_map/kernel_layer_integrator.h"
#include "act_map/sampler.h"

using namespace act_map;

class LayerEvaluatorTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    std::string dir;
    std::string fn;
    rpg::fs::splitPathAndFilename(__FILE__, &dir, &fn);
    std::string abs_map = dir + "/test_data/test_map.txt";

    vi::Map map;
    map.load(abs_map, std::string());

    LayerOptions ker_options;
    trace_layer_ptr_ = std::make_shared<TraceLayer>(ker_options.vox_size,
                                                    ker_options.vox_per_side);
    info_layer_ptr_ = std::make_shared<InfoLayer>(ker_options.vox_size,
                                                  ker_options.vox_per_side);
    rpg::PositionVec uni_points;
    utils::generateUniformPointsWithin(
        ker_options.vox_size, 2.1, 2.1, 2.1, &uni_points);
    for (const Eigen::Vector3d& pt : uni_points)
    {
      trace_layer_ptr_->allocateBlockPtrByCoordinates(pt);
      info_layer_ptr_->allocateBlockPtrByCoordinates(pt);
    }

    LayerOptions occ_options;
    occ_layer_ptr_ = std::make_shared<OccupancyLayer>(occ_options.vox_size,
                                                      occ_options.vox_per_side);
    utils::setPointsInOccupancyLayer(map.points_, occ_layer_ptr_.get());

    KernelLayerIntegratorOptions options;
    KernelLayerIntegrator<TraceVoxel> integrator1(
        options, occ_layer_ptr_, trace_layer_ptr_);
    voxblox::BlockIndexList cand_blks;
    trace_layer_ptr_->getAllAllocatedBlocks(&cand_blks);
    integrator1.recomputeFromOccupancyLayer(cand_blks);

    KernelLayerIntegrator<InfoVoxel> integrator2(
        options, occ_layer_ptr_, info_layer_ptr_);
    info_layer_ptr_->getAllAllocatedBlocks(&cand_blks);
    integrator2.recomputeFromOccupancyLayer(cand_blks);
  }
  OccupancyLayer::Ptr occ_layer_ptr_;
  TraceLayer::Ptr trace_layer_ptr_;
  InfoLayer::Ptr info_layer_ptr_;
};

TEST_F(LayerEvaluatorTest, testInit)
{
  LayerEvaluator<TraceVoxel> trace_eval(trace_layer_ptr_.get());
  LayerEvaluator<InfoVoxel> info_eval;
  info_eval.setKernelLayer(info_layer_ptr_.get());

  const TraceLayer* lptr =  trace_eval.layerPtr();
  EXPECT_EQ(lptr, trace_layer_ptr_.get());
}

TEST_F(LayerEvaluatorTest, testNearestRead)
{
  LayerEvaluator<TraceVoxel> trace_eval(trace_layer_ptr_.get());
  Eigen::Vector3d rpt;
  rpt.setRandom();
  Eigen::Vector3d far(100, 100, 100);

  const TraceVoxel* tvox_ptr = trace_eval.getVoxelNearest(rpt);
  const TraceVoxel* far_tvox_ptr = trace_eval.getVoxelNearest(far);
  EXPECT_TRUE(far_tvox_ptr == nullptr);
  EXPECT_TRUE(tvox_ptr != nullptr);

  TraceBlock::Ptr blk_ptr = trace_layer_ptr_->getBlockPtrByCoordinates(rpt);
  EXPECT_TRUE(blk_ptr != nullptr);
  const TraceVoxel& tvox = blk_ptr->getVoxelByCoordinates(rpt);
  EXPECT_TRUE(voxblox::utils::isSameVoxel(*tvox_ptr, tvox));
}

RPG_COMMON_TEST_MAIN
{
}
