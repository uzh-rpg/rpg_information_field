//Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
//You can contact the author at <zzhang at ifi dot uzh dot ch>
//Unauthorized copying of this file, via any medium is strictly prohibited
//Proprietary and confidential
#include "voxblox/io/layer_io.h"
#include "voxblox/core/layer.h"

#include <rpg_common/test_main.h>
#include <rpg_common/fs.h>
#include <vi_utils/map.h>

#include "act_map/kernel_layer_integrator.h"
#include "act_map/sampler.h"

using namespace act_map;

class ProtobufTest : public ::testing::Test
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
    trace_layer_ptr_ = std::make_shared<TraceLayer>(ker_options.vox_size,
                                                    ker_options.vox_per_side);
    info_layer_ptr_ = std::make_shared<InfoLayer>(ker_options.vox_size,
                                                  ker_options.vox_per_side);
    rpg::PositionVec uni_points;
    utils::generateUniformPointsWithin(
        ker_options.vox_size, x_range_, y_range_, z_range_, &uni_points);
    for (const Eigen::Vector3d& pt : uni_points)
    {
      trace_layer_ptr_->allocateBlockPtrByCoordinates(pt);
      info_layer_ptr_->allocateBlockPtrByCoordinates(pt);
    }

    LayerOptions occ_options;
    occ_layer_ptr_ = std::make_shared<OccupancyLayer>(occ_options.vox_size,
                                                      occ_options.vox_per_side);
    utils::setPointsInOccupancyLayer(map_->points_, occ_layer_ptr_.get());
  }

  std::string abs_map_;
  vi::MapPtr map_;
  double x_range_ = 4.0;
  double y_range_ = 4.0;
  double z_range_ = 2.0;

  TraceLayer::Ptr trace_layer_ptr_;
  InfoLayer::Ptr info_layer_ptr_;
  OccupancyLayer::Ptr occ_layer_ptr_;
};

TEST_F(ProtobufTest, testSaveReadOccupancyLayer)
{
  const std::string fn("/tmp/occ_layer.protobuf");
  voxblox::io::SaveLayer(*occ_layer_ptr_, fn);

  OccupancyLayer::Ptr load_layer_ptr;
  voxblox::io::LoadLayer<OccupancyVoxel>(fn, &load_layer_ptr);

  EXPECT_TRUE(voxblox::utils::isSameLayer(*load_layer_ptr, *occ_layer_ptr_));
}

TEST_F(ProtobufTest, testSaveReadTraceLayer)
{
  KernelLayerIntegratorOptions options;
  KernelLayerIntegrator<TraceVoxel> integrator(
      options, occ_layer_ptr_, trace_layer_ptr_);
  voxblox::BlockIndexList cand_blks;
  trace_layer_ptr_->getAllAllocatedBlocks(&cand_blks);
  integrator.recomputeFromOccupancyLayer(cand_blks);

  const std::string fn("/tmp/trace_layer.protobuf");
  voxblox::io::SaveLayer(*trace_layer_ptr_, fn);

  TraceLayer::Ptr load_layer_ptr;
  voxblox::io::LoadLayer<TraceVoxel>(fn, &load_layer_ptr);

  EXPECT_TRUE(voxblox::utils::isSameLayer(*load_layer_ptr, *trace_layer_ptr_));
}

TEST_F(ProtobufTest, testSaveReadInfoLayer)
{
  KernelLayerIntegratorOptions options;
  KernelLayerIntegrator<InfoVoxel> integrator(
      options, occ_layer_ptr_, info_layer_ptr_);
  voxblox::BlockIndexList cand_blks;
  trace_layer_ptr_->getAllAllocatedBlocks(&cand_blks);
  integrator.recomputeFromOccupancyLayer(cand_blks);

  const std::string fn("/tmp/info_layer.protobuf");
  voxblox::io::SaveLayer(*info_layer_ptr_, fn);

  InfoLayer::Ptr load_layer_ptr;
  voxblox::io::LoadLayer<InfoVoxel>(fn, &load_layer_ptr);

  EXPECT_TRUE(voxblox::utils::isSameLayer(*load_layer_ptr, *info_layer_ptr_));
}

RPG_COMMON_TEST_MAIN
{
}
